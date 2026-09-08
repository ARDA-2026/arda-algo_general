"""드론 미션 엔드포인트.

1단계에서는 좌표 변환 결과만 노출한다 (드론 없이 검증 가능).
2단계에서 tello_driver 를 붙여 /drone/* 제어 엔드포인트를 추가한다.

hanriver.py 를 import 하지 않는다. 순환 import 를 피하려고
상태 조회 함수를 주입받는 구조로 만들었다.
"""

import time

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


class TrackIn(BaseModel):
    speed:        int   = Field(30,    ge=10,  le=100)
    # Tello 는 20cm 미만 이동을 거부한다. 이게 하한이고, 목표를 평활해뒀으므로
    # 여기까지 낮춰도 잡음을 쫓지 않는다.
    threshold_cm: float = Field(20.0,  ge=20.0, le=200.0)
    max_sec:      float = Field(180.0, ge=10.0, le=600.0)


class AutoTrackIn(TrackIn):
    """탐지 확정(sim_started) 시 자동으로 추적을 시작할지."""
    enabled: bool = True


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


# ── 추적 모드 ──
# 확률 1순위 좌표를 계속 쫓아간다. 탐지 확정 시 자동 시작도 지원한다.
#
# 목표 평활 계수. 1.0 = 평활 없음 = 현재 1순위 좌표를 그대로 추종.
#
# 예전에는 0.25(시정수 약 8초)를 썼다. 1:150 축척에서는 격자 한 칸이 종이
# 위 10cm 여서 argmax 가 한두 칸만 튀어도 이동 임계값 20cm 를 넘겨버렸기
# 때문이다. 지금은 1:1100 이라 한 칸이 3.6cm 이고, 20cm 를 넘으려면 5칸
# 넘게 튀어야 한다 — 임계값 자체가 잡음을 걸러준다.
#
# 그래서 평활은 지연만 남기게 됐다. 끄면 "지금 1순위" 를 곧바로 쫓는다.
# 만약 확률이 비슷한 두 봉우리 사이에서 1순위가 왕복해 드론이 오간다면,
# 이 값을 0.3~0.5 로 낮추거나 TrackIn.threshold_cm 을 올리면 된다.
TRACK_EMA_ALPHA = 1.0

_autotrack = {"enabled": False, "speed": 30, "threshold_cm": 20.0, "max_sec": 180.0}


def _track_fns():
    """추적에 쓸 목표/경계 판정 함수를 만든다.

    목표는 waypoints[0] — NMS 가 최고 확률 칸을 항상 먼저 채택하므로
    이게 곧 '확률 1순위 좌표'다.

    TRACK_EMA_ALPHA=1.0 이면 평활 없이 현재 1순위를 그대로 쫓는다.
    격자 이산화 잡음은 이동 임계값(20cm)이 걸러준다 — 자세한 근거는
    TRACK_EMA_ALPHA 주석 참조.
    """
    sm = {"x": None, "y": None}     # 추적 1회분 평활 상태

    def target_fn():
        st = _get_state() if _get_state else None
        if not st or not st.get("map") or not st.get("waypoints"):
            return None
        wp = st["waypoints"][0]
        m  = st["map"]
        te, tn   = geo_to_enu(wp["lon"], wp["lat"], m["takeoff_lon"], m["takeoff_lat"])
        tem, tnm = enu_to_map(te, tn, m["scale"])
        x, y = map_to_tello_cm(tem, tnm)

        if sm["x"] is None:
            sm["x"], sm["y"] = x, y
        else:
            sm["x"] += (x - sm["x"]) * TRACK_EMA_ALPHA
            sm["y"] += (y - sm["y"]) * TRACK_EMA_ALPHA
        return sm["x"], sm["y"]

    def bounds_fn(x_cm, y_cm):
        st = _get_state() if _get_state else None
        if not st or not st.get("map"):
            return False
        m = st["map"]
        margin = (m["lon_max"] - m["origin_lon"]) * M_PER_DEG_LON / m["scale"]
        # 기체 좌표(이륙점 기준) -> 지도 좌표(입수점 기준) 로 되돌려 판정
        east_t, north_t = -y_cm / 100.0, x_cm / 100.0
        de, dn = geo_to_enu(m["takeoff_lon"], m["takeoff_lat"],
                            m["origin_lon"], m["origin_lat"])
        dem, dnm = enu_to_map(de, dn, m["scale"])
        return in_map_bounds(east_t + dem, north_t + dnm,
                             m["width_m"], m["height_m"], margin)

    return target_fn, bounds_fn


@router.post("/drone/track")
async def drone_track(body: TrackIn):
    """확률 1순위 좌표를 계속 따라간다. 이륙부터 착륙까지 자동."""
    if _get_state is None:
        raise HTTPException(503, "상태 제공자가 등록되지 않았습니다")
    target_fn, bounds_fn = _track_fns()
    try:
        return driver.start_tracking(target_fn, bounds_fn, speed=body.speed,
                                     threshold_cm=body.threshold_cm,
                                     max_sec=body.max_sec)
    except RuntimeError as e:
        raise HTTPException(409, str(e))


@router.post("/drone/autotrack")
async def drone_autotrack(body: AutoTrackIn):
    """탐지 확정 시 자동으로 이륙+추적을 시작할지 설정한다."""
    _autotrack.update(enabled=body.enabled, speed=body.speed,
                      threshold_cm=body.threshold_cm, max_sec=body.max_sec)
    return {"ok": True, **_autotrack}


@router.get("/drone/autotrack")
async def drone_autotrack_get():
    return dict(_autotrack)


# 탐지 확정 즉시 이륙한다. 1순위 좌표는 아직 없어도 되고,
# 생기면 드라이버가 알아서 추종을 시작한다.
# 기체가 아직 연결 안 됐을 때만 잠깐 대기한다.
_pending_until = 0.0
AUTOTRACK_WAIT_SEC = 60.0


def _try_start_track():
    """조건이 되면 추적을 시작한다. 성공 여부를 반환."""
    s = driver.status()
    if s["running"] or s["airborne"]:
        return True                 # 이미 떠 있으면 관여하지 않는다
    if not s["connected"]:
        return False
    target_fn, bounds_fn = _track_fns()
    try:
        driver.start_tracking(target_fn, bounds_fn,
                              speed=_autotrack["speed"],
                              threshold_cm=_autotrack["threshold_cm"],
                              max_sec=_autotrack["max_sec"])
        return True
    except Exception as e:
        driver._ev(f"[자동추적] 시작 실패 - {type(e).__name__}: {e}")
        return True                 # 재시도하지 않는다 (배터리 부족 등)


def notify_sim_started():
    """hanriver 가 sim_started 를 False→True 로 올릴 때 호출한다.

    실패해도 시뮬레이션은 계속 돌아야 하므로 예외를 밖으로 내보내지 않는다.
    """
    global _pending_until
    if not _autotrack["enabled"]:
        return
    driver._ev("[자동추적] 탐지 확정 - 이륙")
    if not _try_start_track():
        # 기체 미연결. 연결되면 바로 뜨도록 잠깐 기다린다.
        _pending_until = time.time() + AUTOTRACK_WAIT_SEC
        driver._ev(f"[자동추적] 기체 미연결 - {AUTOTRACK_WAIT_SEC:.0f}초 안에 연결되면 이륙")


def tick_autotrack():
    """시뮬 스텝마다 호출된다. 연결 대기 중일 때만 일을 한다."""
    global _pending_until
    if not _pending_until or not _autotrack["enabled"]:
        return
    if time.time() > _pending_until:
        _pending_until = 0.0
        driver._ev("[자동추적] 기체가 연결되지 않아 취소")
        return
    if _try_start_track():
        _pending_until = 0.0


@router.post("/drone/manual")
async def drone_manual():
    """자율비행 중 수동 조종으로 전환한다 (착륙하지 않음).

    진행 중인 이동이 끝나야 실제로 넘어가므로, 응답의 handover 가 true 면
    아직 전환 대기 중이다. status 를 폴링해서 mode=="manual" 이 되면 완료.
    """
    try:
        return driver.take_manual()
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
