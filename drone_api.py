"""드론 미션 엔드포인트.

1단계에서는 좌표 변환 결과만 노출한다 (드론 없이 검증 가능).
2단계에서 tello_driver 를 붙여 /drone/* 제어 엔드포인트를 추가한다.

hanriver.py 를 import 하지 않는다. 순환 import 를 피하려고
상태 조회 함수를 주입받는 구조로 만들었다.
"""

from fastapi import APIRouter, HTTPException, Query
from pydantic import BaseModel, Field

from tello_mission import (M_PER_DEG_LON, build_mission,
                           geo_to_enu, enu_to_map, map_to_tello_cm, in_map_bounds)
from tello_driver import driver

router = APIRouter()

_get_state = None


class ConnectIn(BaseModel):
    dry_run: bool = True        # 기본은 안전하게 시뮬레이션


class GotoIn(BaseModel):
    """수동 이동 목표. 화면에서 본 좌표를 그대로 보낸다."""
    lon:   float
    lat:   float
    speed: int = Field(30, ge=10, le=100)
    label: str = ""


class StartIn(BaseModel):
    # 범위를 벗어난 값이 기체까지 내려가면 비행 중에 터진다. 여기서 막는다.
    top_n:     int   = Field(3,   ge=1,  le=10)
    speed:     int   = Field(30,  ge=10, le=100)   # Tello go 속도 규격
    hover_sec: float = Field(3.0, ge=0.0, le=30.0)


def set_state_provider(fn):
    """sim_state 스냅샷을 반환하는 함수를 등록한다."""
    global _get_state
    _get_state = fn


def _current_mission(top_n):
    """현재 sim_state 에서 미션을 만든다 (호출 시점 스냅샷)."""
    if _get_state is None:
        raise HTTPException(503, "상태 제공자가 등록되지 않았습니다")

    state = _get_state()
    if not state:
        raise HTTPException(503, "시뮬레이션이 아직 준비되지 않았습니다")

    wps = state.get("waypoints") or []
    if not wps:
        raise HTTPException(409, "아직 waypoint 가 없습니다 (파티클 누적 대기 중)")

    m = state["map"]
    scale = m["scale"]
    # 원점(입수 지점)에서 지도 동쪽 끝까지의 거리 (지도 미터)
    east_margin_m = (m["lon_max"] - m["origin_lon"]) * M_PER_DEG_LON / scale

    mission = build_mission(
        wps,
        origin_lon=m["origin_lon"],
        origin_lat=m["origin_lat"],
        takeoff_lon=m.get("takeoff_lon"),
        takeoff_lat=m.get("takeoff_lat"),
        scale=scale,
        map_w_m=m["width_m"],
        map_h_m=m["height_m"],
        east_margin_m=east_margin_m,
        top_n=top_n,
    )
    mission["elapsed_sec"] = state.get("elapsed_sec")
    mission["in_river_count"] = state.get("in_river_count")
    return mission


@router.get("/mission")
async def get_mission(top_n: int = Query(3, ge=1, le=10)):
    """현재 waypoint 를 지도 위 Tello 좌표로 변환해서 반환한다.

    드론 없이 호출해서 좌표 변환·축 부호·지도 경계를 검증하는 용도.
    """
    return _current_mission(top_n)


# ─────────────────────────────────────────
# 드론 제어
# ─────────────────────────────────────────
@router.post("/drone/connect")
async def drone_connect(body: ConnectIn):
    """기체 연결. dry_run=true 면 실제 기체 없이 흉내만 낸다."""
    try:
        return driver.connect(dry_run=body.dry_run)
    except RuntimeError as e:
        raise HTTPException(409, str(e))        # 상태 충돌 (미션 실행 중 등)
    except Exception as e:
        raise HTTPException(502, f"연결 실패: {type(e).__name__}: {e}")


@router.post("/drone/start")
async def drone_start(body: StartIn):
    """현재 waypoint 로 미션을 만들어 즉시 실행한다.

    미션은 이 시점의 스냅샷으로 고정된다. 시뮬레이션이 계속 진행돼도
    비행 중에 목표가 바뀌지 않는다.
    """
    mission = _current_mission(body.top_n)
    try:
        st = driver.start(mission, speed=body.speed, hover_sec=body.hover_sec)
    except RuntimeError as e:
        raise HTTPException(409, str(e))
    return {"status": st, "mission": mission}


# ── 수동 모드 ──
# 자동 순회와 달리 이동 후에도 착륙하지 않고 떠 있는다.
# 지점을 하나씩 골라 보내는 운용 방식.
@router.post("/drone/takeoff")
async def drone_takeoff():
    """이륙만 한다. 이후 /drone/goto 로 지점을 하나씩 지정한다."""
    try:
        return driver.manual_takeoff()
    except RuntimeError as e:
        raise HTTPException(409, str(e))


@router.post("/drone/goto")
async def drone_goto(body: GotoIn):
    """지정한 좌표로 한 번 이동한다 (착륙 안 함)."""
    if _get_state is None:
        raise HTTPException(503, "상태 제공자가 등록되지 않았습니다")
    st = _get_state()
    if not st or not st.get("map"):
        raise HTTPException(503, "시뮬레이션이 아직 준비되지 않았습니다")

    m = st["map"]
    margin = (m["lon_max"] - m["origin_lon"]) * M_PER_DEG_LON / m["scale"]

    # 지도 경계 판정은 입수 지점 기준, 기체 좌표는 이륙 지점 기준
    e, n   = geo_to_enu(body.lon, body.lat, m["origin_lon"], m["origin_lat"])
    em, nm = enu_to_map(e, n, m["scale"])
    if not in_map_bounds(em, nm, m["width_m"], m["height_m"], margin):
        raise HTTPException(409, "목표가 지도 밖입니다")

    te, tn   = geo_to_enu(body.lon, body.lat, m["takeoff_lon"], m["takeoff_lat"])
    tem, tnm = enu_to_map(te, tn, m["scale"])
    x_cm, y_cm = map_to_tello_cm(tem, tnm)

    try:
        return driver.manual_goto(x_cm, y_cm, speed=body.speed, label=body.label)
    except RuntimeError as e:
        raise HTTPException(409, str(e))


@router.post("/drone/home")
async def drone_home(speed: int = Query(30, ge=10, le=100)):
    """이륙 지점(기체 좌표 0,0)으로 돌아간다. 착륙은 하지 않는다."""
    try:
        return driver.manual_goto(0.0, 0.0, speed=speed, label="이륙 지점 복귀")
    except RuntimeError as e:
        raise HTTPException(409, str(e))


@router.post("/drone/path/reset")
async def drone_path_reset():
    """화면에 표시되는 지나간 경로만 지운다. 비행에는 영향 없음."""
    return driver.clear_path()


@router.post("/drone/land")
async def drone_land():
    """즉시 착륙 (비상 정지). 실행 중인 미션은 중단된다."""
    return driver.land_now()


@router.get("/drone/status")
async def drone_status():
    return driver.status()
