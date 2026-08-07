from datetime import datetime
import os
import numpy as np
import threading
import time
import json
import asyncio
import requests
import xml.etree.ElementTree as ET
from dotenv import load_dotenv

load_dotenv()

VERBOSE = True          # False 로 바꾸면 모든 콘솔 출력 중단
def log(*args):
    if VERBOSE: print(*args)

from shapely.ops import unary_union
from shapely import contains_xy
import osmnx as ox
from fastapi import FastAPI, HTTPException, WebSocket, WebSocketDisconnect
from fastapi.responses import HTMLResponse
from fastapi.staticfiles import StaticFiles
from pydantic import BaseModel, Field
import uvicorn

import drone_api

log("=== Han River Real-time Drift Simulation (FastAPI) ===")

# ─────────────────────────────────────────
# [1] 유속 API
# ─────────────────────────────────────────
API_KEY  = os.getenv("API_KEY", "")
OBS_CODE = "1018683"

def get_velocity():
    url = f"https://api.hrfco.go.kr/{API_KEY}/waterlevel/list/10M/{OBS_CODE}.xml"
    try:
        response = requests.get(url, timeout=10)
        root = ET.fromstring(response.content)
        item = root.find('.//Waterlevel')
        fw    = float(item.find('fw').text)
        wl    = float(item.find('wl').text)
        ymdhm = item.find('ymdhm').text
        WIDTH, DEPTH = 900, 6
        velocity = fw / (WIDTH * DEPTH)
        log(f"[API] 시각: {ymdhm} | 수위: {wl}m | 유량: {fw}m³/s | 유속: {velocity:.4f}m/s")
        return -velocity, 0.0
    except Exception as e:
        log(f"[API 오류] {e} → 기본값 사용")
        return -0.05, 0.0

# ── 유속 설정 ──────────────────────────────
# 기본값: 고정 유속 (API 키 없이 바로 실행 가능)
velocity_x = -1.5   # m/s, 서쪽 방향 (한강 평균)
velocity_y =  0.05  # m/s, 남쪽 방향 (거의 0)

# HRFCO API 실시간 유속으로 전환하려면:
#   1. .env 파일에 API_KEY 입력
#   2. 위 두 줄을 주석 처리하고 아래 줄 주석 해제
# velocity_x, velocity_y = get_velocity()
# ────────────────────────────────────────────

log(f"[INIT] velocity_x={velocity_x:.4f} m/s")

# ─────────────────────────────────────────
# [2] OSM 한강 폴리곤
# ─────────────────────────────────────────
log("Loading Han River polygon...")
hangang = ox.features_from_place(
    "Seoul, South Korea",
    tags={"natural": "water", "water": "river"}
)

lon_min, lon_max = 126.900, 126.918
lat_min, lat_max = 37.537, 37.546

hangang_mapo = hangang.cx[lon_min:lon_max, lat_min:lat_max]
hangang_mapo = hangang_mapo[hangang_mapo.geometry.area > 0.00005]

log("Building river polygon union...")
hangang_union = unary_union(hangang_mapo.geometry)

river_geojson = json.loads(hangang_mapo.geometry.to_json())

# ─────────────────────────────────────────
# [3] 파티클 초기화
# ─────────────────────────────────────────
N = 200
MAPO_LAT = 37.540
MAPO_LON = 126.907

RADIUS_DEG = 10 / 111000
np.random.seed(42)
angles = np.random.uniform(0, 2 * np.pi, N)
radii  = np.random.uniform(0, RADIUS_DEG, N)

particles_lon = MAPO_LON + radii * np.cos(angles)
particles_lat = MAPO_LAT + radii * np.sin(angles)

TURBULENCE = 0.3
particle_vx = velocity_x + np.random.normal(0, abs(velocity_x) * TURBULENCE, N)
particle_vy = velocity_y + np.random.normal(0, abs(velocity_x) * TURBULENCE, N)

DT    = 0.1
# SPEED = 1프레임당 시뮬레이션 스텝 수.
# 드론 실기 연동에는 1 (약 1.5배속) 을 쓴다. 60이면 약 90배속이라
# 드론이 이륙하기도 전에 파티클이 지도를 벗어난다.
SPEED = 1
pvlon = particle_vx / 88000
pvlat = particle_vy / 111000
DIFFUSIVITY = 2.0 / 88000 * np.sqrt(2 * DT)
elapsed_sec = 0.0

PRINT_INTERVAL    = 180
last_printed_time = -PRINT_INTERVAL

# ─────────────────────────────────────────
# [3-1] 실물 축소 지도 영역 (3m × 2m, 축척 1:150)
# ─────────────────────────────────────────
M_PER_DEG_LON = 88000    # 위도 37.5° 기준
M_PER_DEG_LAT = 111000

MAP_SCALE   = 150     # 1:150
MAP_PRINT_W = 3.0     # 실물 지도 가로 m (동서)
MAP_PRINT_H = 2.0     # 실물 지도 세로 m (남북)
MAP_EAST_M  = 50.0    # 입수 지점에서 동쪽 여유 (나머지는 서쪽 표류 구간)

GRID_CELL_M = 15.0    # 격자 한 칸이 덮을 실제 거리 (칸이 정사각형에 가깝게 유지됨)

# 아래 값들은 전부 _rebuild_map() 이 채운다. 지도 설정이 바뀌면 다시 계산된다.
MAP_W_M = MAP_H_M = 0.0
map_lon_min = map_lon_max = map_lat_min = map_lat_max = 0.0
GRID_NX = GRID_NY = 0
GRID_XEDGES = GRID_YEDGES = GRID_XCENT = GRID_YCENT = None
NMS_MIN_DIST_M = 100.0
accumulated_hist = None
new_map_cfg = None    # POST /map 에서 설정

# ─────────────────────────────────────────
# [3-2] 드론 이륙 지점
# ─────────────────────────────────────────
# 입수 지점과 분리해야 하는 이유:
#   - 입수 지점은 열화상 카메라가 감지하는 값이라 사고마다 달라진다.
#     이륙 자리는 물리적으로 고정돼 있어야 한다.
#   - 둘이 겹치면 시작 직후 WP1 이 이륙 지점과 붙어버려서
#     Tello 최소 이동거리(20cm) 미만이 되어 "수색 우선순위 1번"이 스킵된다.
#   - 드론이 지도의 입수 지점 표식(★)을 깔고 앉는다.
#
# 기본값: 지도 동쪽 변에서 실물 30cm 바깥 (강변 베이스에서 출격하는 모양)
TAKEOFF_MARGIN_MAP_M = 0.30   # 지도 밖으로 나갈 거리 (실물 m)

takeoff_lon = takeoff_lat = 0.0
takeoff_is_default = True     # 사용자가 직접 찍었으면 False. 지도가 바뀌어도 유지한다.
new_takeoff = None            # POST /takeoff 에서 설정

NMS_MAX_COUNT = 10


def default_takeoff():
    """지도 동쪽 변 바깥 TAKEOFF_MARGIN_MAP_M 지점."""
    east = MAP_EAST_M + TAKEOFF_MARGIN_MAP_M * MAP_SCALE
    return MAPO_LON + east / M_PER_DEG_LON, MAPO_LAT


def _rebuild_map(print_w=None, print_h=None, scale=None, east_m=None):
    """지도 설정을 바꾸고 거기에 딸린 것들을 전부 다시 계산한다.

    누적 격자는 지도 영역에 "고정" 되어야 한다. 매 프레임 파티클 무게중심으로
    재중심을 잡으면 서로 다른 좌표계의 카운트를 더하게 되어 누적이 뭉개진다.
    따라서 지도가 바뀌면 격자도 통째로 새로 만들고 누적을 리셋해야 한다.
    """
    global MAP_SCALE, MAP_PRINT_W, MAP_PRINT_H, MAP_EAST_M, MAP_W_M, MAP_H_M
    global map_lon_min, map_lon_max, map_lat_min, map_lat_max
    global GRID_NX, GRID_NY, GRID_XEDGES, GRID_YEDGES, GRID_XCENT, GRID_YCENT
    global NMS_MIN_DIST_M, accumulated_hist, takeoff_lon, takeoff_lat

    if scale   is not None: MAP_SCALE   = float(scale)
    if print_w is not None: MAP_PRINT_W = float(print_w)
    if print_h is not None: MAP_PRINT_H = float(print_h)
    if east_m  is not None: MAP_EAST_M  = float(east_m)

    MAP_W_M = MAP_PRINT_W * MAP_SCALE          # 지도가 덮는 실제 거리 (동서)
    MAP_H_M = MAP_PRINT_H * MAP_SCALE          # 동일 (남북)

    map_lon_max = MAPO_LON + MAP_EAST_M / M_PER_DEG_LON
    map_lon_min = map_lon_max - MAP_W_M / M_PER_DEG_LON
    map_lat_max = MAPO_LAT + (MAP_H_M / 2) / M_PER_DEG_LAT
    map_lat_min = MAPO_LAT - (MAP_H_M / 2) / M_PER_DEG_LAT

    # 칸 크기를 먼저 정한 뒤 나눈다. 칸 수를 각각 자르면 큰 지도에서
    # 상한(80)에 걸려 칸이 직사각형으로 찌그러진다.
    GRID_MAX = 80
    cell = max(GRID_CELL_M, MAP_W_M / GRID_MAX, MAP_H_M / GRID_MAX)
    GRID_NX = int(max(5, min(GRID_MAX, round(MAP_W_M / cell))))
    GRID_NY = int(max(5, min(GRID_MAX, round(MAP_H_M / cell))))
    GRID_XEDGES = np.linspace(map_lon_min, map_lon_max, GRID_NX + 1)
    GRID_YEDGES = np.linspace(map_lat_min, map_lat_max, GRID_NY + 1)
    GRID_XCENT  = (GRID_XEDGES[:-1] + GRID_XEDGES[1:]) / 2
    GRID_YCENT  = (GRID_YEDGES[:-1] + GRID_YEDGES[1:]) / 2

    # waypoint 간 최소 간격은 지도 크기에 비례해야 한다.
    # 고정 100m 로 두면 작은 지도에서는 지도 폭보다 커져 waypoint 가 1개만 남는다.
    # 기본 지도(450m)에서 100m 가 되도록 잡은 비율.
    NMS_MIN_DIST_M = MAP_W_M / 4.5

    accumulated_hist = np.zeros((GRID_NX, GRID_NY))

    if takeoff_is_default:
        takeoff_lon, takeoff_lat = default_takeoff()


_rebuild_map()

# 누적 히트맵의 망각 계수.
# 단순 합계로 누적하면 초기에 파티클 200개가 몰려 있던 입수 지점 칸이
# 영원히 1위가 되어, 표류가 진행돼도 수색 우선순위가 갱신되지 않는다.
# 지수 감쇠를 걸어 "최근 체류 시간" 기준으로 만든다.
HIST_HALF_LIFE_SEC = 20.0    # 이 시간이 지나면 과거 기여도가 절반
HIST_DECAY = 0.5 ** ((DT * SPEED) / HIST_HALF_LIFE_SEC)

best_trail_lons  = [MAPO_LON]
best_trail_lats  = [MAPO_LAT]
stranded_lons    = []
stranded_lats    = []
observation      = None
obs_history      = []
_step            = 0

def filter_in_river(lons, lats):
    return contains_xy(hangang_union, lons, lats)


def select_spaced(sorted_wps, min_dist_m, max_count):
    """확률 내림차순 waypoint 에서 서로 min_dist_m 이상 떨어진 것만 골라낸다.

    비최대 억제(NMS). 상위 확률 칸은 서로 인접해 있어서 그대로 쓰면
    수색 구역이 한 점에 뭉친다.
    """
    picked = []
    for wp in sorted_wps:
        too_close = False
        for p in picked:
            de = (wp["lon"] - p["lon"]) * M_PER_DEG_LON
            dn = (wp["lat"] - p["lat"]) * M_PER_DEG_LAT
            if de * de + dn * dn < min_dist_m * min_dist_m:
                too_close = True
                break
        if not too_close:
            picked.append(wp)
            if len(picked) >= max_count:
                break
    return picked

log("Computing initial river mask...")
in_river      = filter_in_river(particles_lon, particles_lat)
in_river_prev = in_river.copy()

# ─────────────────────────────────────────
# 공유 상태 (시뮬 스레드 ↔ FastAPI)
# ─────────────────────────────────────────
sim_lock  = threading.Lock()
sim_state: dict = {}
new_observation = None   # POST /observation 에서 설정

# ─────────────────────────────────────────
# 시뮬레이션 스텝 
# ─────────────────────────────────────────
def simulation_step():
    global particles_lon, particles_lat, in_river, in_river_prev
    global elapsed_sec, accumulated_hist
    global best_trail_lons, best_trail_lats
    global pvlon, pvlat
    global stranded_lons, stranded_lats
    global observation, obs_history, new_observation
    global _step, last_printed_time
    global takeoff_lon, takeoff_lat, new_takeoff
    global new_map_cfg, takeoff_is_default

    # 지도 설정 변경 (POST /map). 격자가 통째로 바뀌므로 시뮬 스레드에서 적용한다.
    if new_map_cfg is not None:
        cfg, new_map_cfg = new_map_cfg, None
        _rebuild_map(**cfg)
        best_trail_lons.clear(); best_trail_lats.clear()
        log(f"[MAP] {MAP_PRINT_W}x{MAP_PRINT_H}m 1:{MAP_SCALE:.0f} "
            f"= 실제 {MAP_W_M:.0f}x{MAP_H_M:.0f}m, 격자 {GRID_NX}x{GRID_NY}")

    # 새 이륙 지점 적용 (툴바에서 지정 → POST /takeoff)
    if new_takeoff is not None:
        takeoff_lon, takeoff_lat = new_takeoff
        new_takeoff = None
        takeoff_is_default = False

    # 새 관측값 적용 (마우스 클릭 → POST /observation)
    if new_observation is not None:
        obs_lon, obs_lat = new_observation
        new_observation  = None
        observation = (obs_lon, obs_lat)
        obs_history.append((obs_lon, obs_lat))

        RADIUS_OBS = 5 / 111000
        a = np.random.uniform(0, 2 * np.pi, N)
        r = np.random.uniform(0, RADIUS_OBS, N)
        particles_lon[:] = obs_lon + r * np.cos(a)
        particles_lat[:] = obs_lat + r * np.sin(a)
        pvlon[:] = velocity_x / 88000 + np.random.normal(0, abs(velocity_x) * TURBULENCE / 88000,  N)
        pvlat[:] = velocity_y / 111000 + np.random.normal(0, abs(velocity_x) * TURBULENCE / 111000, N)
        accumulated_hist[:] = 0

    max_vlon = abs(velocity_x) * 2 / 88000
    max_vlat = abs(velocity_x) * 1 / 111000

    # ── SPEED 루프  ──
    for _ in range(SPEED):
        particles_lon += pvlon * DT + np.random.normal(0, DIFFUSIVITY, N)
        particles_lat += pvlat * DT + np.random.normal(0, DIFFUSIVITY, N)

        pvlon += np.random.normal(0, abs(velocity_x) * 0.05 / 88000,  N)
        pvlat += np.random.normal(0, abs(velocity_x) * 0.05 / 111000, N)

        pvlon = np.clip(pvlon, -max_vlon, -max_vlon * 0.05)
        pvlat = np.clip(pvlat, -max_vlat,  max_vlat)

        elapsed_sec += DT
        _step       += 1

        if _step % 5 == 0:
            in_river_prev  = in_river.copy()
            in_river       = filter_in_river(particles_lon, particles_lat)
            newly_stranded = in_river_prev & ~in_river
            if newly_stranded.any():
                stranded_lons.extend(particles_lon[newly_stranded].tolist())
                stranded_lats.extend(particles_lat[newly_stranded].tolist())

    # ── 히트맵 / 웨이포인트  ──
    lons_v = particles_lon[in_river]
    lats_v = particles_lat[in_river]

    heatmap        = []
    heatmap_extent = [lon_min, lon_max, lat_min, lat_max]
    waypoints      = []
    best_lon       = best_trail_lons[-1] if best_trail_lons else MAPO_LON
    best_lat       = best_trail_lats[-1] if best_trail_lats else MAPO_LAT

    # 강 안 파티클을 "고정" 격자에 누적한다 (격자는 지도 영역에 못박혀 있음).
    # 매 스텝 과거분을 감쇠시켜, 오래된 체류 기록이 현재 우선순위를 가리지 않게 한다.
    if len(lons_v) > 0:
        hist, _, _ = np.histogram2d(lons_v, lats_v, bins=[GRID_XEDGES, GRID_YEDGES])
        accumulated_hist *= HIST_DECAY
        accumulated_hist += hist

    # 누적이 남아 있으면 파티클이 전부 강 밖으로 나가도 waypoint 를 계속 제공한다
    total = accumulated_hist.sum()
    if total > 0:
        hist_prob = accumulated_hist / total * 100

        heatmap        = hist_prob.T.tolist()
        heatmap_extent = [map_lon_min, map_lon_max, map_lat_min, map_lat_max]

        max_idx  = np.unravel_index(hist_prob.argmax(), hist_prob.shape)
        best_lon = float(GRID_XCENT[max_idx[0]])
        best_lat = float(GRID_YCENT[max_idx[1]])
        best_trail_lons.append(best_lon)
        best_trail_lats.append(best_lat)

        raw = [
            {"lon": float(GRID_XCENT[i]), "lat": float(GRID_YCENT[j]),
             "prob": float(hist_prob[i, j])}
            for i, j in zip(*np.nonzero(hist_prob))
        ]
        raw.sort(key=lambda w: w["prob"], reverse=True)
        waypoints = select_spaced(raw, NMS_MIN_DIST_M, NMS_MAX_COUNT)

    # 콘솔 waypoint 출력
    if elapsed_sec - last_printed_time >= PRINT_INTERVAL and waypoints:
        last_printed_time = elapsed_sec
        mins = int(elapsed_sec // 60)
        secs = int(elapsed_sec % 60)
        log(f"\n[T+{mins}m{secs:02d}s] In-river={int(np.sum(in_river))} Stranded={len(stranded_lons)}")
        for idx, wp in enumerate(waypoints[:3], 1):
            log(f"  Top{idx}: ({wp['lat']:.5f}, {wp['lon']:.5f})  {wp['prob']:.1f}%")

    # 공유 상태 갱신
    with sim_lock:
        sim_state.update({
            "elapsed_sec":     float(elapsed_sec),
            "particles_lon":   lons_v.tolist(),
            "particles_lat":   lats_v.tolist(),
            "heatmap":         heatmap,
            "heatmap_extent":  heatmap_extent,
            "best_lon":        float(best_lon),
            "best_lat":        float(best_lat),
            "best_trail_lons": best_trail_lons[-200:],
            "best_trail_lats": best_trail_lats[-200:],
            "waypoints":       waypoints[:10],
            # 좌초 지점과 관측 기록은 계속 쌓이는데 100ms 마다 통째로 전송된다.
            # best_trail 과 같은 방식으로 잘라 보내고, 총계는 따로 실어준다.
            "stranded_lons":   stranded_lons[-800:],
            "stranded_lats":   stranded_lats[-800:],
            "stranded_count":  len(stranded_lons),
            "observation":     list(observation) if observation else None,
            "obs_history":     obs_history[-50:],
            "in_river_count":  int(np.sum(in_river)),
            "n_particles":     N,
            "velocity_x":      float(velocity_x),
            "turbulence":      TURBULENCE,
            "entry_lon":       MAPO_LON,
            "entry_lat":       MAPO_LAT,
            "bounds": {
                "lon_min": lon_min, "lon_max": lon_max,
                "lat_min": lat_min, "lat_max": lat_max,
            },
            # 실물 축소 지도 (드론 미션 좌표 변환의 기준)
            "map": {
                "scale":    MAP_SCALE,
                "width_m":  MAP_PRINT_W,           # 지도 실물 가로 (m)
                "height_m": MAP_PRINT_H,           # 지도 실물 세로 (m)
                "real_w_m": MAP_W_M,               # 덮는 실제 거리 (동서)
                "real_h_m": MAP_H_M,               # 덮는 실제 거리 (남북)
                "east_m":   MAP_EAST_M,
                "grid_nx":  GRID_NX, "grid_ny": GRID_NY,
                "nms_m":    round(NMS_MIN_DIST_M, 1),
                "lon_min": map_lon_min, "lon_max": map_lon_max,
                "lat_min": map_lat_min, "lat_max": map_lat_max,
                # origin  = 입수 지점. 지도 경계 판정의 지리 기준
                # takeoff = 드론 이륙 자리. Tello 기체 좌표의 (0,0)
                "origin_lon": MAPO_LON, "origin_lat": MAPO_LAT,
                "takeoff_lon": float(takeoff_lon),
                "takeoff_lat": float(takeoff_lat),
            },
        })


def _sim_thread():
    while True:
        simulation_step()
        time.sleep(0.05)


threading.Thread(target=_sim_thread, daemon=True).start()
log("[SIM] Background simulation started")

# ─────────────────────────────────────────
# FastAPI
# ─────────────────────────────────────────
app = FastAPI(title="Han River Drift")
app.mount("/static", StaticFiles(directory="static"), name="static")

# 드론 미션 라우터 (/mission)
def _state_snapshot():
    with sim_lock:
        return dict(sim_state)

drone_api.set_state_provider(_state_snapshot)
app.include_router(drone_api.router)


class ObservationIn(BaseModel):
    lon: float
    lat: float


@app.get("/", response_class=HTMLResponse)
async def index():
    with open("static/index.html", encoding="utf-8") as f:
        # 캐시 금지. UI 를 고쳐도 브라우저가 옛 파일을 계속 쓰면
        # "고쳤는데 왜 그대로냐" 로 시간을 버린다.
        return HTMLResponse(f.read(), headers={
            "Cache-Control": "no-store, no-cache, must-revalidate",
            "Pragma": "no-cache",
        })


@app.get("/river-geojson")
async def get_river_geojson():
    return river_geojson


@app.get("/state")
async def get_state():
    with sim_lock:
        return dict(sim_state)


@app.post("/observation")
async def post_observation(obs: ObservationIn):
    global new_observation
    new_observation = (obs.lon, obs.lat)
    return {"ok": True}


@app.post("/takeoff")
async def post_takeoff(pt: ObservationIn):
    """드론 이륙 지점을 지정한다 (Tello 기체 좌표의 원점).

    비행 중에 바꾸면 지령 위치 누적이 어긋나므로 거부한다.
    """
    global new_takeoff
    if drone_api.driver.status()["running"]:
        raise HTTPException(409, "비행 중에는 이륙 지점을 바꿀 수 없습니다")
    new_takeoff = (pt.lon, pt.lat)
    return {"ok": True, "lon": pt.lon, "lat": pt.lat}


@app.post("/takeoff/reset")
async def post_takeoff_reset():
    """기본 이륙 지점(지도 동쪽 변 바깥 30cm)으로 되돌린다."""
    global new_takeoff, takeoff_is_default
    if drone_api.driver.status()["running"]:
        raise HTTPException(409, "비행 중에는 이륙 지점을 바꿀 수 없습니다")
    new_takeoff = default_takeoff()
    takeoff_is_default = True
    return {"ok": True}


class MapIn(BaseModel):
    # 실물 지도 크기(m)와 축척. 덮는 실제 거리는 둘의 곱으로 정해진다.
    width_m:  float = Field(3.0,   ge=0.3, le=20.0)
    height_m: float = Field(2.0,   ge=0.3, le=20.0)
    scale:    float = Field(150.0, ge=10.0, le=2000.0)
    east_m:   float = Field(50.0,  ge=0.0, le=5000.0)  # 입수 지점에서 동쪽 여유


@app.post("/map")
async def post_map(cfg: MapIn):
    """지도 영역을 바꾼다. 격자와 누적 히트맵이 리셋된다."""
    global new_map_cfg
    if drone_api.driver.status()["running"]:
        raise HTTPException(409, "비행 중에는 지도를 바꿀 수 없습니다")
    if cfg.east_m > cfg.width_m * cfg.scale:
        raise HTTPException(
            422, f"동쪽 여유({cfg.east_m:.0f}m)가 지도 가로"
                 f"({cfg.width_m * cfg.scale:.0f}m)보다 큽니다")
    new_map_cfg = {"print_w": cfg.width_m, "print_h": cfg.height_m,
                   "scale": cfg.scale, "east_m": cfg.east_m}
    return {"ok": True, **cfg.model_dump(),
            "real_w_m": cfg.width_m * cfg.scale,
            "real_h_m": cfg.height_m * cfg.scale}


@app.post("/map/reset")
async def post_map_reset():
    global new_map_cfg
    if drone_api.driver.status()["running"]:
        raise HTTPException(409, "비행 중에는 지도를 바꿀 수 없습니다")
    new_map_cfg = {"print_w": 3.0, "print_h": 2.0, "scale": 150.0, "east_m": 50.0}
    return {"ok": True}


@app.websocket("/ws")
async def websocket_endpoint(ws: WebSocket):
    await ws.accept()
    try:
        while True:
            with sim_lock:
                data = json.dumps(sim_state)
            await ws.send_text(data)
            await asyncio.sleep(0.1)
    except WebSocketDisconnect:
        pass


if __name__ == "__main__":
    log("[SERVER] http://localhost:8000  (Ctrl+C 로 종료)")
    uvicorn.run(app, host="0.0.0.0", port=8000, log_level="warning")
