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
    if VERBOSE: log(*args)

from shapely.ops import unary_union
from shapely import contains_xy
import osmnx as ox
from fastapi import FastAPI, WebSocket, WebSocketDisconnect
from fastapi.responses import HTMLResponse
from fastapi.staticfiles import StaticFiles
from pydantic import BaseModel
import uvicorn

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

velocity_x, velocity_y = get_velocity()
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
SPEED = 60
pvlon = particle_vx / 88000
pvlat = particle_vy / 111000
DIFFUSIVITY = 2.0 / 88000 * np.sqrt(2 * DT)
elapsed_sec = 0.0

PRINT_INTERVAL    = 180
last_printed_time = -PRINT_INTERVAL

accumulated_hist = np.zeros((15, 15))
best_trail_lons  = [MAPO_LON]
best_trail_lats  = [MAPO_LAT]
stranded_lons    = []
stranded_lats    = []
observation      = None
obs_history      = []
_step            = 0

def filter_in_river(lons, lats):
    return contains_xy(hangang_union, lons, lats)

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

    if len(lons_v) > 1:
        center_lon = np.mean(lons_v)
        center_lat = np.mean(lats_v)
        spread = 0.0015

        hist, xedges, yedges = np.histogram2d(
            lons_v, lats_v, bins=15,
            range=[
                [center_lon - spread, center_lon + spread],
                [center_lat - spread, center_lat + spread]
            ]
        )
        accumulated_hist += hist
        hist_prob = accumulated_hist / accumulated_hist.sum() * 100

        heatmap        = hist_prob.T.tolist()
        heatmap_extent = [center_lon - spread, center_lon + spread,
                          center_lat - spread, center_lat + spread]

        max_idx  = np.unravel_index(hist_prob.argmax(), hist_prob.shape)
        best_lon = float((xedges[max_idx[0]] + xedges[max_idx[0] + 1]) / 2)
        best_lat = float((yedges[max_idx[1]] + yedges[max_idx[1] + 1]) / 2)
        best_trail_lons.append(best_lon)
        best_trail_lats.append(best_lat)

        for i in range(hist_prob.shape[0]):
            for j in range(hist_prob.shape[1]):
                p = hist_prob[i, j]
                if p > 0:
                    waypoints.append({
                        "lon":  float((xedges[i] + xedges[i + 1]) / 2),
                        "lat":  float((yedges[j] + yedges[j + 1]) / 2),
                        "prob": float(p),
                    })
        waypoints.sort(key=lambda x: x["prob"], reverse=True)

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
