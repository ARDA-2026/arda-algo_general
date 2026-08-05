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
from fastapi import FastAPI, WebSocket, WebSocketDisconnect
from fastapi.responses import HTMLResponse
from fastapi.staticfiles import StaticFiles
from pydantic import BaseModel
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

MAP_SCALE    = 150            # 1:150
MAP_W_M      = 3.0 * MAP_SCALE  # 450 m (동서, 지도 긴 변)
MAP_H_M      = 2.0 * MAP_SCALE  # 300 m (남북, 지도 짧은 변)
MAP_EAST_M   = 50.0           # 입수 지점에서 동쪽 여유 (나머지는 서쪽 표류 구간)

map_lon_max = MAPO_LON + MAP_EAST_M / M_PER_DEG_LON
map_lon_min = map_lon_max - MAP_W_M / M_PER_DEG_LON
map_lat_max = MAPO_LAT + (MAP_H_M / 2) / M_PER_DEG_LAT
map_lat_min = MAPO_LAT - (MAP_H_M / 2) / M_PER_DEG_LAT

# 누적 격자는 지도 영역에 "고정" 한다.
# (매 프레임 파티클 무게중심으로 재중심을 잡으면 서로 다른 좌표계의
#  카운트를 더하게 되어 누적 확률이 뭉개진다.)
GRID_NX = 30    # 450m / 30 = 15 m/칸
GRID_NY = 20    # 300m / 20 = 15 m/칸
GRID_XEDGES = np.linspace(map_lon_min, map_lon_max, GRID_NX + 1)
GRID_YEDGES = np.linspace(map_lat_min, map_lat_max, GRID_NY + 1)
GRID_XCENT  = (GRID_XEDGES[:-1] + GRID_XEDGES[1:]) / 2
GRID_YCENT  = (GRID_YEDGES[:-1] + GRID_YEDGES[1:]) / 2

# 수색 우선순위 waypoint 를 서로 떨어뜨리는 최소 간격.
# 확률 상위 N개를 그냥 뽑으면 인접 칸이 나와 드론이 제자리에서 도는 것처럼 보인다.
NMS_MIN_DIST_M = 100.0
NMS_MAX_COUNT  = 10

# 누적 히트맵의 망각 계수.
# 단순 합계로 누적하면 초기에 파티클 200개가 몰려 있던 입수 지점 칸이
# 영원히 1위가 되어, 표류가 진행돼도 수색 우선순위가 갱신되지 않는다.
# 지수 감쇠를 걸어 "최근 체류 시간" 기준으로 만든다.
HIST_HALF_LIFE_SEC = 20.0    # 이 시간이 지나면 과거 기여도가 절반
HIST_DECAY = 0.5 ** ((DT * SPEED) / HIST_HALF_LIFE_SEC)

accumulated_hist = np.zeros((GRID_NX, GRID_NY))
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
            "stranded_lons":   list(stranded_lons),
            "stranded_lats":   list(stranded_lats),
            "observation":     list(observation) if observation else None,
            "obs_history":     list(obs_history),
            "in_river_count":  int(np.sum(in_river)),
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
                "scale":   MAP_SCALE,
                "width_m":  MAP_W_M / MAP_SCALE,   # 지도 실물 가로 (m)
                "height_m": MAP_H_M / MAP_SCALE,   # 지도 실물 세로 (m)
                "lon_min": map_lon_min, "lon_max": map_lon_max,
                "lat_min": map_lat_min, "lat_max": map_lat_max,
                "origin_lon": MAPO_LON, "origin_lat": MAPO_LAT,
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
        return HTMLResponse(f.read())


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
