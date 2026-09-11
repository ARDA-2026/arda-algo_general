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

# 네이버 지도 Client ID. 브라우저에 노출되는 값이라 비밀은 아니지만,
# 콘솔에 등록한 도메인 밖에서는 거부되므로 사실상 도메인 등록이 인증이다.
# 저장소에 박지 않고 .env 로 뺀 이유는 팀원마다 다른 키를 쓸 수 있어서다.
# 비어 있으면 프론트는 오프라인 배경(static/mapomap.jpg)으로 폴백한다.
NAVER_MAP_CLIENT_ID = os.getenv("NAVER_MAP_CLIENT_ID", "")

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
# 기본 입수 지점.
#
# 이 값은 인쇄할 배경 사진(static/mapomap.jpg)에서 실제 다리가 지나는
# 픽셀에 입수 지점 별표가 얹히도록 역산한 것이다 — MAP_EAST_M /
# MAP_SOUTH_M 주석 참고. 사진을 바꾸면 셋을 같이 다시 잡아야 한다.
#
# 폴리곤으로 검증한 사항 (표류는 서쪽으로 흐른다):
#   - 강 안이어야 한다. 시작하자마자 육지 판정이면 시뮬이 무의미하다.
#   - 서쪽으로 물이 이어져야 한다. 하류에서 강이 북으로 휘기 때문에
#     위도를 조금만 낮게 잡아도 파티클이 금방 남안에 부딪힌다
#     (실측: 37.5340 은 5분 만에 절반이 육지였다).
#
# 이 위치에 두는 이유: 아래 폴리곤 선별 박스가 이 좌표를 기준으로 잡히므로
# hangang.cx 호출보다 먼저 정의돼야 한다.
MAPO_LAT = 37.5336
MAPO_LON = 126.9364

log("Loading Han River polygon...")
hangang = ox.features_from_place(
    "Seoul, South Korea",
    tags={"natural": "water", "water": "river"}
)

# 입수 지점 주변만 남긴다. 좌표를 하드코딩해두면 MAPO 를 옮겼을 때
# 엉뚱한 지역의 폴리곤만 남아 강이 통째로 사라진다 — .cx 가 "박스에
# 걸치는 피처"를 고르는 방식이라 한강 본류가 한 덩어리인 동안에는
# 우연히 동작하지만, 기대서는 안 되는 성질이다.
#
# 여유 0.05도 = 동서 약 4.4km, 남북 약 5.5km. 지도를 1:800(2.4km)까지
# 키워도 덮는다.
POLY_MARGIN_DEG = 0.05
lon_min, lon_max = MAPO_LON - POLY_MARGIN_DEG, MAPO_LON + POLY_MARGIN_DEG
lat_min, lat_max = MAPO_LAT - POLY_MARGIN_DEG, MAPO_LAT + POLY_MARGIN_DEG

hangang_mapo = hangang.cx[lon_min:lon_max, lat_min:lat_max]
hangang_mapo = hangang_mapo[hangang_mapo.geometry.area > 0.00005]

log("Building river polygon union...")
hangang_union = unary_union(hangang_mapo.geometry)

river_geojson = json.loads(hangang_mapo.geometry.to_json())

# ─────────────────────────────────────────
# [3] 파티클 초기화
# ─────────────────────────────────────────
N = 200
# POST /reset이 되돌아갈 기본 입수 지점 — MAPO_LON/LAT은 낙하 확정이 올
# 때마다 그 지점으로 덮어써지므로, 원래 기본값을 따로 보존해둔다.
DEFAULT_MAPO_LON = MAPO_LON
DEFAULT_MAPO_LAT = MAPO_LAT

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
SPEED = 3
pvlon = particle_vx / 88000
pvlat = particle_vy / 111000
DIFFUSIVITY = 2.0 / 88000 * np.sqrt(2 * DT)
elapsed_sec = 0.0

PRINT_INTERVAL    = 180
last_printed_time = -PRINT_INTERVAL

# ─────────────────────────────────────────
# [3-1] 실물 축소 지도 영역 (3m × 2m, 축척 1:400)
# ─────────────────────────────────────────
M_PER_DEG_LON = 88000    # 위도 37.5° 기준
M_PER_DEG_LAT = 111000

# 축척·인쇄물 크기·여유값은 전부 배경 사진(static/mapomap.jpg)에 묶여 있다.
# 사진이 1273x849px = 비율 1.4994 이라 인쇄물도 3:2 여야 지형이 안 늘어난다.
MAP_SCALE   = 400     # 1:400
MAP_PRINT_W = 3.0     # 실물 지도 가로 m (동서)
MAP_PRINT_H = 2.0     # 실물 지도 세로 m (남북)
# 입수 지점(기기 좌표)에서 동/남쪽 여유 — 배경 사진(static/mapomap.jpg,
# 1273×849px)에서 실제 마포대교가 지나는 픽셀(약 x=1112, y=752, 사진
# 좌상단 기준)에 입수 지점 별표가 정확히 얹히도록 역산한 값이다:
#   east_m  = MAP_W_M  * (1 - 1112/1273) ≈ 151.8
#   south_m = MAP_H_M  * (1 -  752/849 ) ≈  91.4
# 사진이나 지도 크기(MAP_PRINT_W/H, MAP_SCALE)가 바뀌면 이 값도 다시
# 계산해야 한다 — 사진 속 실제 위치와 무관하게 나머지는 자동으로 안 맞음.
MAP_EAST_M  = 151.8    # 입수 지점에서 동쪽 여유 (나머지는 서쪽 표류 구간)
MAP_SOUTH_M = 91.4     # 입수 지점에서 남쪽 여유 (나머지는 북쪽 구간)

# POST /map/reset 이 돌아갈 자리. 위 값들을 그대로 기억해둔다.
# 예전에는 리셋이 1:150 / 50m / 150m 를 하드코딩해서, 누르면 이 프로젝트와
# 무관한 450x300m 짜리 지도로 바뀌어버렸다.
DEFAULT_MAP_CFG = {"print_w": MAP_PRINT_W, "print_h": MAP_PRINT_H,
                   "scale": MAP_SCALE, "east_m": MAP_EAST_M, "south_m": MAP_SOUTH_M}

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
# 저장은 위경도가 아니라 "지도 좌하단(남서) 모서리 기준 오프셋" 으로 한다.
#
#   왜 절대 위경도로 두면 안 되나:
#     바닥에 깔린 인쇄 지도와 그 옆의 드론은 물리적으로 붙어 있다. 그런데
#     입수 지점(=지도 원점)은 열화상 감지 결과라 POST /report 마다 옮겨간다.
#     이륙 지점을 절대 좌표로 박아두면 지도만 따라 움직이고 이륙 지점은
#     제자리에 남아서, 드론은 바닥에서 1cm 도 안 움직였는데 소프트웨어만
#     엉뚱한 자리를 (0,0) 으로 알게 된다.
#     오프셋으로 저장하면 지도가 어디로 가든 물리적 배치가 그대로 유지된다.
#
#   좌하단을 기준으로 잡은 이유: 설치 당일 줄자로 재는 기준이 지도 모서리다.
#   "왼쪽 아래 모서리에서 오른쪽 0.5m, 위로 0.2m" 가 바닥에서 바로 재진다.
TAKEOFF_OFF_LIMIT_M = 3.0     # 지도 밖으로 허용할 최대 거리 (실물 m)

takeoff_off_e = 0.0           # 좌하단에서 오른쪽(동)  실물 m — 기본은 모서리 그 자리
takeoff_off_n = 0.0           # 좌하단에서 위쪽(북)    실물 m
takeoff_lon = takeoff_lat = 0.0   # 위 오프셋에서 매번 유도되는 값
new_takeoff = None            # POST /takeoff* 에서 설정 — (east_m, north_m)

NMS_MAX_COUNT = 10


def takeoff_from_offset(east_m, north_m):
    """지도 좌하단(남서) 모서리 기준 실물 m 오프셋 -> 위경도."""
    return (map_lon_min + east_m  * MAP_SCALE / M_PER_DEG_LON,
            map_lat_min + north_m * MAP_SCALE / M_PER_DEG_LAT)


def offset_from_takeoff(lon, lat):
    """위경도 -> 지도 좌하단 기준 실물 m 오프셋 (지도 클릭용 역변환)."""
    return ((lon - map_lon_min) * M_PER_DEG_LON / MAP_SCALE,
            (lat - map_lat_min) * M_PER_DEG_LAT / MAP_SCALE)


def _rebuild_map(print_w=None, print_h=None, scale=None, east_m=None, south_m=None):
    """지도 설정을 바꾸고 거기에 딸린 것들을 전부 다시 계산한다.

    누적 격자는 지도 영역에 "고정" 되어야 한다. 매 프레임 파티클 무게중심으로
    재중심을 잡으면 서로 다른 좌표계의 카운트를 더하게 되어 누적이 뭉개진다.
    따라서 지도가 바뀌면 격자도 통째로 새로 만들고 누적을 리셋해야 한다.
    """
    global MAP_SCALE, MAP_PRINT_W, MAP_PRINT_H, MAP_EAST_M, MAP_SOUTH_M, MAP_W_M, MAP_H_M
    global map_lon_min, map_lon_max, map_lat_min, map_lat_max
    global GRID_NX, GRID_NY, GRID_XEDGES, GRID_YEDGES, GRID_XCENT, GRID_YCENT
    global NMS_MIN_DIST_M, accumulated_hist, takeoff_lon, takeoff_lat

    if scale   is not None: MAP_SCALE   = float(scale)
    if print_w is not None: MAP_PRINT_W = float(print_w)
    if print_h is not None: MAP_PRINT_H = float(print_h)
    if east_m  is not None: MAP_EAST_M  = float(east_m)
    if south_m is not None: MAP_SOUTH_M = float(south_m)

    MAP_W_M = MAP_PRINT_W * MAP_SCALE          # 지도가 덮는 실제 거리 (동서)
    MAP_H_M = MAP_PRINT_H * MAP_SCALE          # 동일 (남북)

    map_lon_max = MAPO_LON + MAP_EAST_M / M_PER_DEG_LON
    map_lon_min = map_lon_max - MAP_W_M / M_PER_DEG_LON
    # 예전에는 입수 지점이 남북으로 정중앙(±MAP_H_M/2)이었는데, 배경 사진
    # 위에서 실제 위치(마포대교 부근)에 별표가 얹히려면 남북도 동서
    # (MAP_EAST_M)처럼 비대칭 여유가 필요해 MAP_SOUTH_M을 추가했다.
    map_lat_min = MAPO_LAT - MAP_SOUTH_M / M_PER_DEG_LAT
    map_lat_max = map_lat_min + MAP_H_M / M_PER_DEG_LAT

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

    # waypoint 간 최소 간격. 지도 폭에 비례시키면 안 된다 — 그렇게 두면
    # 지도를 키울 때 간격이 파티클 구름보다 커져서 2·3순위가 확률 0 인
    # 자리에서 뽑힌다 (실측: 3.2km 지도에서 간격 706m -> 2·3순위가 끝까지 0.00%).
    #
    # 실제로 걸리는 제약은 "종이 위 거리" 다. 두 조건이 여기서 만난다:
    #   (1) Tello 최소 이동이 종이 20cm 라, 지점이 그보다 촘촘하면 자동 순회에서
    #       그 레그가 통째로 스킵된다 (실측: 19.3cm 간격 -> 16개 중 3개 스킵)
    #   (2) 파티클 구름(4분에 450m 남짓)보다 넓으면 (1) 의 문제가 생긴다
    # 종이 25cm 가 두 조건을 동시에 만족한다. 실측으로 스킵 0/16, 세 순위 모두
    # 처음부터 확률이 붙었다.
    NMS_MIN_DIST_M = 0.25 * MAP_SCALE

    accumulated_hist = np.zeros((GRID_NX, GRID_NY))

    # 이륙 지점은 지도에 붙어 다닌다. 지도가 옮겨가거나 크기가 바뀌면
    # 좌하단 모서리도 따라 움직이므로 여기서 항상 다시 유도한다.
    takeoff_lon, takeoff_lat = takeoff_from_offset(takeoff_off_e, takeoff_off_n)


_rebuild_map()

# 누적 히트맵의 망각 계수.
# 단순 합계로 누적하면 초기에 파티클 200개가 몰려 있던 입수 지점 칸이
# 영원히 1위가 되어, 표류가 진행돼도 수색 우선순위가 갱신되지 않는다.
# 지수 감쇠를 걸어 "최근 체류 시간" 기준으로 만든다.
HIST_HALF_LIFE_SEC = 20.0    # 이 시간이 지나면 과거 기여도가 절반
HIST_DECAY = 0.5 ** ((DT * SPEED) / HIST_HALF_LIFE_SEC)

best_trail_lons  = []
best_trail_lats  = []
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
new_observation = None   # POST /observation 또는 POST /report(낙하 확정 신호)에서 설정
new_reset       = None   # POST /reset 에서 설정 — 대기 상태로 되돌림
# 낙하 판정(POST /report의 확정 신호, 또는 대기 중 온 POST /observation)이
# 오기 전까지는 True가 되지 않는다 — False인 동안 simulation_step()은
# 파티클을 전혀 움직이지 않고 대기만 한다(젯슨이 아직 아무것도 안 보내도
# 파티클이 저절로 퍼지는 걸 막기 위함). POST /reset으로 다시 False로
# 되돌릴 수 있다.
sim_started = False


def _reset_to_origin(entry_lon: float, entry_lat: float) -> None:
    """입수 지점을 entry_lon/entry_lat으로 옮기고 파티클·지도·누적 이력을
    전부 초기화한다. 대기 상태에서 첫 낙하 판정이 왔을 때(POST /report,
    /observation)와 POST /reset(기본 지점으로) 둘 다 이 함수를 쓴다 —
    sim_started를 True/False 어느 쪽으로 할지는 호출부가 정한다."""
    global particles_lon, particles_lat, pvlon, pvlat, in_river_prev
    global elapsed_sec, best_trail_lons, best_trail_lats
    global stranded_lons, stranded_lats, observation, obs_history
    global new_observation, _step, last_printed_time
    global MAPO_LON, MAPO_LAT

    MAPO_LON, MAPO_LAT = entry_lon, entry_lat

    angles = np.random.uniform(0, 2 * np.pi, N)
    radii  = np.random.uniform(0, RADIUS_DEG, N)
    particles_lon[:] = entry_lon + radii * np.cos(angles)
    particles_lat[:] = entry_lat + radii * np.sin(angles)
    pvlon[:] = velocity_x / 88000 + np.random.normal(0, abs(velocity_x) * TURBULENCE / 88000,  N)
    pvlat[:] = velocity_y / 111000 + np.random.normal(0, abs(velocity_x) * TURBULENCE / 111000, N)

    # 지도 원점이 바뀌었으니 격자·누적 히트맵·(기본값이면) 이륙 지점을
    # 새 원점 기준으로 다시 계산한다.
    _rebuild_map()

    elapsed_sec = 0.0
    best_trail_lons = []
    best_trail_lats = []
    stranded_lons = []
    stranded_lats = []
    observation = None
    obs_history = []
    new_observation = None  # 리셋과 동시에 대기 중이던 재감지 관측값은 폐기
    _step = 0
    last_printed_time = -PRINT_INTERVAL
    in_river_prev = filter_in_river(particles_lon, particles_lat)


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
    global takeoff_off_e, takeoff_off_n, new_map_cfg
    global new_reset, sim_started, MAPO_LON, MAPO_LAT

    # 대기 상태로 리셋 (툴바 리셋 버튼 → POST /reset). 기본 입수 지점으로
    # 되돌리고 sim_started를 다시 False로 내려서, 다음 낙하 판정(POST /report
    # 의 확정 신호, 또는 대기 중 온 POST /observation)이 올 때까지 파티클을
    # 멈춘다.
    if new_reset:
        new_reset = None
        _reset_to_origin(DEFAULT_MAPO_LON, DEFAULT_MAPO_LAT)
        sim_started = False
        log("[RESET] 대기 상태로 복귀 — 다음 낙하 판정을 기다립니다")

    # 지도 설정 변경 (POST /map). 격자가 통째로 바뀌므로 시뮬 스레드에서 적용한다.
    if new_map_cfg is not None:
        cfg, new_map_cfg = new_map_cfg, None
        _rebuild_map(**cfg)
        best_trail_lons.clear(); best_trail_lats.clear()
        log(f"[MAP] {MAP_PRINT_W}x{MAP_PRINT_H}m 1:{MAP_SCALE:.0f} "
            f"= 실제 {MAP_W_M:.0f}x{MAP_H_M:.0f}m, 격자 {GRID_NX}x{GRID_NY}")

    # 새 이륙 지점 적용 (툴바 지도 클릭 → POST /takeoff, 숫자 입력 → /takeoff/offset)
    if new_takeoff is not None:
        takeoff_off_e, takeoff_off_n = new_takeoff
        new_takeoff = None
        takeoff_lon, takeoff_lat = takeoff_from_offset(takeoff_off_e, takeoff_off_n)
        log(f"[TAKEOFF] 지도 좌하단 기준 동{takeoff_off_e * 100:.0f} "
            f"북{takeoff_off_n * 100:.0f} cm (실물)")

    # 새 관측값 적용 — 브라우저 클릭(POST /observation) 또는 arda-bringup의
    # 낙하 확정/재감지(POST /report)가 여기로 들어온다. 아직 대기 상태
    # (sim_started=False)에서 처음 오는 값은 입수 지점으로 삼아 시뮬레이션을
    # 처음부터 시작하고(지도 원점·격자·누적 이력까지 리셋), 이미 시작된
    # 뒤에는 파티클만 그 지점으로 재수렴시킨다(이력 유지) — drift_node의
    # "첫 관측값을 낙하 지점으로 삼는다" 원칙과 동일.
    if new_observation is not None:
        obs_lon, obs_lat = new_observation
        new_observation = None

        if not sim_started:
            _reset_to_origin(obs_lon, obs_lat)
            sim_started = True
            log(f"[START] 낙하 판정 수신 — 시뮬레이션 시작: lat={obs_lat:.6f} lon={obs_lon:.6f}")
            # 자동 추적이 켜져 있으면 드론을 띄운다. 이 시점엔 아직 waypoint 가
            # 없으므로 곧바로 뜨지는 않고, 목표가 생길 때까지 대기했다 뜬다.
            drone_api.notify_sim_started()
        else:
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

    if not sim_started:
        # 대기 상태 — 파티클을 전혀 움직이지 않고 빈 상태만 내보낸다.
        # thermal_image_* 는 여기서 건드리지 않는다 — /report로 오는 열화상
        # 스트리밍은 sim_started와 무관하게 항상 최신 이미지를 보여줘야 한다.
        with sim_lock:
            sim_state.update({
                "sim_started":     False,
                "elapsed_sec":     0.0,
                "particles_lon":   [],
                "particles_lat":   [],
                "heatmap":         [],
                "heatmap_extent":  [lon_min, lon_max, lat_min, lat_max],
                "best_lon":        None,
                "best_lat":        None,
                "best_trail_lons": [],
                "best_trail_lats": [],
                "waypoints":       [],
                "stranded_lons":   stranded_lons[-800:],
                "stranded_lats":   stranded_lats[-800:],
                "stranded_count":  len(stranded_lons),
                "observation":     list(observation) if observation else None,
                "obs_history":     obs_history[-50:],
                "in_river_count":  0,
                "n_particles":     N,
                "velocity_x":      float(velocity_x),
                "turbulence":      TURBULENCE,
                "entry_lon":       MAPO_LON,
                "entry_lat":       MAPO_LAT,
                "bounds": {
                    "lon_min": lon_min, "lon_max": lon_max,
                    "lat_min": lat_min, "lat_max": lat_max,
                },
                "map": {
                    "scale":    MAP_SCALE,
                    "width_m":  MAP_PRINT_W,
                    "height_m": MAP_PRINT_H,
                    "real_w_m": MAP_W_M,
                    "real_h_m": MAP_H_M,
                    "east_m":   MAP_EAST_M,
                    "south_m":  MAP_SOUTH_M,
                    "grid_nx":  GRID_NX, "grid_ny": GRID_NY,
                    "nms_m":    round(NMS_MIN_DIST_M, 1),
                    "lon_min": map_lon_min, "lon_max": map_lon_max,
                    "lat_min": map_lat_min, "lat_max": map_lat_max,
                    "origin_lon": MAPO_LON, "origin_lat": MAPO_LAT,
                    "takeoff_lon": float(takeoff_lon),
                    "takeoff_lat": float(takeoff_lat),
                    # 지도 좌하단(남서) 모서리 기준 실물 m — 바닥에서 재는 값
                    "takeoff_off_e": float(takeoff_off_e),
                    "takeoff_off_n": float(takeoff_off_n),
                },
            })
        return

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
            "sim_started":     True,
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
                "south_m":  MAP_SOUTH_M,
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


def _autotrack_tick():
    """기체 연결을 기다리는 중이면 처리한다. 평소엔 즉시 반환."""
    try:
        drone_api.tick_autotrack()
    except Exception as e:
        log(f"[자동추적] tick 오류: {e}")


def _sim_thread():
    while True:
        simulation_step()
        _autotrack_tick()
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


class ReportIn(BaseModel):
    """arda-bringup의 기존 send_fall_report() payload 그대로. image_jpeg
    없이 부르면 lat/lon/timestamp만, 있으면 thermal_image_base64/confirmed도
    같이 온다. thermal_image_yolo_base64는 실제 온도 그대로인
    thermal_image_base64와 별개로, YOLO가 보는 배경-상대 보정 이미지를
    같이 실어 보낼 때만 채워진다 — 웹에서 재요청 없이 토글로 어느 쪽을
    볼지 고르기 위함(arda-radar/arda/utils/web_report.py 참고)."""
    lat: float
    lon: float
    timestamp: str | None = None
    thermal_image_base64: str | None = None
    thermal_image_yolo_base64: str | None = None
    confirmed: bool = False


@app.get("/", response_class=HTMLResponse)
async def index():
    with open("static/index.html", encoding="utf-8") as f:
        # 캐시 금지. UI 를 고쳐도 브라우저가 옛 파일을 계속 쓰면
        # "고쳤는데 왜 그대로냐" 로 시간을 버린다.
        return HTMLResponse(f.read(), headers={
            "Cache-Control": "no-store, no-cache, must-revalidate",
            "Pragma": "no-cache",
        })


@app.get("/config")
async def get_config():
    """프론트가 부팅 때 필요한 설정. 지금은 네이버 지도 키뿐이다."""
    return {"naver_map_client_id": NAVER_MAP_CLIENT_ID}


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


@app.post("/report")
async def post_report(r: ReportIn):
    """arda-bringup이 이미 쓰던 report_url 메커니즘(arda.utils.send_fall_report)
    이 그대로 호출하는 엔드포인트 — 별도 브릿지 함수 없이 기존 payload를
    그대로 받는다.

    - thermal_image_base64가 없는 호출(radar_worker.py가 열화상 확정
      순간 1회 보내는 것)이거나 confirmed=True인 호출(thermal_worker.py가
      확정 프레임에 함께 보내는 것)은 "낙하 판정" 이벤트로 취급해 POST
      /observation과 똑같이 처리한다 — 대기 중이면 그 지점에서 시뮬레이션을
      시작하고, 이미 시작됐으면 파티클만 재수렴시킨다.
    - thermal_image_base64가 있는 모든 호출(관찰/대기 중 상시 스트리밍
      프레임 포함, confirmed 여부 무관)은 최신 열화상 이미지로 저장해
      시각화에 쓴다 — sim_started·파티클과 무관하게 항상 갱신된다.
      thermal_image_yolo_base64(YOLO가 보는 배경-상대 보정 이미지)도 같이
      오면 함께 저장한다 — 실제 온도 이미지와 별개 필드라 웹에서 재요청
      없이 토글로 어느 쪽을 볼지 고를 수 있다(static/index.html의
      updateThermalCard 참고).
    """
    global new_observation

    if r.thermal_image_base64 is not None:
        with sim_lock:
            sim_state["thermal_image_base64"] = r.thermal_image_base64
            sim_state["thermal_image_yolo_base64"] = r.thermal_image_yolo_base64
            sim_state["thermal_image_confirmed"] = r.confirmed
            sim_state["thermal_image_ts"] = r.timestamp

    if r.thermal_image_base64 is None or r.confirmed:
        new_observation = (r.lon, r.lat)

    return {"ok": True}


@app.post("/reset")
async def post_reset():
    """시뮬레이션을 대기 상태로 되돌린다 — 기본 입수 지점으로 복원하고
    파티클을 멈춘 뒤, 다음 낙하 판정(POST /report 또는 대기 중 /observation)
    까지 아무것도 계산하지 않는다.

    /map, /takeoff와 같은 이유로 비행 중에는 거부한다.
    """
    global new_reset
    if drone_api.driver.status()["running"]:
        raise HTTPException(409, "비행 중에는 리셋할 수 없습니다")
    new_reset = True
    return {"ok": True}


class TakeoffOffsetIn(BaseModel):
    """지도 좌하단(남서) 모서리 기준 실물 m 오프셋."""
    east_m:  float = Field(..., ge=-TAKEOFF_OFF_LIMIT_M)
    north_m: float = Field(..., ge=-TAKEOFF_OFF_LIMIT_M)


def _stage_takeoff(east_m: float, north_m: float):
    """이륙 지점 오프셋을 검증하고 시뮬 스레드가 집어가도록 올려둔다.

    지도 밖으로 조금 나가는 건 정상이지만(강변 베이스에서 출격) 너무 멀면
    지도 안 waypoint 까지의 거리가 Tello 한 번 이동 상한(500cm)을 넘겨
    미션이 통째로 거부된다. 그럴 바엔 여기서 막는 게 낫다.
    """
    global new_takeoff
    if drone_api.driver.status()["running"]:
        raise HTTPException(409, "비행 중에는 이륙 지점을 바꿀 수 없습니다")
    lim = TAKEOFF_OFF_LIMIT_M
    if not (-lim <= east_m <= MAP_PRINT_W + lim) or \
       not (-lim <= north_m <= MAP_PRINT_H + lim):
        raise HTTPException(
            400,
            f"이륙 지점이 지도에서 너무 멉니다 — 좌하단 기준 "
            f"동 {-lim:.1f}~{MAP_PRINT_W + lim:.1f} m, "
            f"북 {-lim:.1f}~{MAP_PRINT_H + lim:.1f} m 안이어야 합니다")
    new_takeoff = (east_m, north_m)
    return {"ok": True, "east_m": east_m, "north_m": north_m}


@app.post("/sim/start")
async def post_sim_start():
    """[테스트] 낙하 판정이 온 것처럼 시뮬레이션을 시작한다.

    arda-bringup(잭슨) 쪽 신호 없이 우리 쪽만 돌려볼 때 쓴다.

    직접 sim_started 를 True 로 세우지 않고 new_observation 을 넣는다.
    그러면 simulation_step 의 상승 엣지 처리를 그대로 타므로 파티클 초기화,
    지도/격자 재구성, 자동 모드 이륙 통보까지 실제 흐름과 똑같이 재현된다.
    플래그만 뒤집으면 그 중 아무것도 안 일어나서 테스트가 무의미해진다.
    """
    global new_observation
    if sim_started:
        return {"ok": True, "already_started": True}
    new_observation = (MAPO_LON, MAPO_LAT)
    log("[TEST] 수동으로 낙하 판정 모의 — 시뮬레이션 시작")
    return {"ok": True, "already_started": False}


@app.post("/takeoff")
async def post_takeoff(pt: ObservationIn):
    """지도를 클릭해 이륙 지점을 지정한다 (Tello 기체 좌표의 원점).

    받은 위경도는 곧바로 좌하단 기준 오프셋으로 바꿔 저장한다 — 절대
    좌표로 두면 입수 지점이 옮겨갈 때 이륙 지점만 뒤에 남는다.
    비행 중에 바꾸면 지령 위치 누적이 어긋나므로 거부한다.
    """
    return _stage_takeoff(*offset_from_takeoff(pt.lon, pt.lat))


@app.post("/takeoff/offset")
async def post_takeoff_offset(off: TakeoffOffsetIn):
    """이륙 지점을 숫자로 지정한다 — 지도 좌하단 모서리에서 실물 m.

    설치 당일 바닥에서 줄자로 재는 값을 그대로 넣는 용도.
    """
    return _stage_takeoff(off.east_m, off.north_m)


@app.post("/takeoff/reset")
async def post_takeoff_reset():
    """기본 이륙 지점(지도 좌하단 모서리)으로 되돌린다."""
    return _stage_takeoff(0.0, 0.0)


class MapIn(BaseModel):
    # 실물 지도 크기(m)와 축척. 덮는 실제 거리는 둘의 곱으로 정해진다.
    # east_m/south_m 기본값은 배경 사진(static/mapomap.jpg) 속 마포대교
    # 위치에 입수 지점 별표가 얹히도록 맞춘 값이다 — MAP_EAST_M/MAP_SOUTH_M
    # 정의부(위쪽) 주석 참고.
    width_m:  float = Field(3.0,   ge=0.3, le=20.0)
    height_m: float = Field(2.0,   ge=0.3, le=20.0)
    scale:    float = Field(150.0, ge=10.0, le=2000.0)
    east_m:   float = Field(151.8, ge=0.0, le=5000.0)  # 입수 지점에서 동쪽 여유
    south_m:  float = Field(91.4,  ge=0.0, le=5000.0)  # 입수 지점에서 남쪽 여유


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
    if cfg.south_m > cfg.height_m * cfg.scale:
        raise HTTPException(
            422, f"남쪽 여유({cfg.south_m:.0f}m)가 지도 세로"
                 f"({cfg.height_m * cfg.scale:.0f}m)보다 큽니다")
    new_map_cfg = {"print_w": cfg.width_m, "print_h": cfg.height_m,
                   "scale": cfg.scale, "east_m": cfg.east_m, "south_m": cfg.south_m}
    return {"ok": True, **cfg.model_dump(),
            "real_w_m": cfg.width_m * cfg.scale,
            "real_h_m": cfg.height_m * cfg.scale}


@app.post("/map/reset")
async def post_map_reset():
    global new_map_cfg
    if drone_api.driver.status()["running"]:
        raise HTTPException(409, "비행 중에는 지도를 바꿀 수 없습니다")
    # 파일 상단에서 정의한 기본값으로 되돌린다. 여기에 숫자를 또 적으면
    # 상단 값과 어긋나서, 리셋이 "기본으로 복귀"가 아니라 "엉뚱한 값으로 변경"이 된다.
    new_map_cfg = dict(DEFAULT_MAP_CFG)
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
