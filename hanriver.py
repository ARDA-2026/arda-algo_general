from datetime import datetime
import numpy as np
import matplotlib
import matplotlib.cm as cm
matplotlib.use('QtAgg')
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
from matplotlib.ticker import ScalarFormatter
from shapely.geometry import Point
from shapely.ops import unary_union
import osmnx as ox
import requests
import xml.etree.ElementTree as ET

print("=== Han River Real-time Drift Simulation ===")

# ─────────────────────────────────────────
# HRFCO API
# ─────────────────────────────────────────
API_KEY  = ""
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
        print(f"[API] 시각: {ymdhm} | 수위: {wl}m | 유량: {fw}m³/s | 유속: {velocity:.4f}m/s")
        return -velocity, 0.0
    except Exception as e:
        print(f"[API 오류] {e} → 기본값 사용")
        return -0.05, 0.0

# 테스트용 유속
# velocity_x = -3.0
# velocity_y =  1.5
# print(f"[TEST] 유속: {abs(velocity_x)} m/s (서쪽)")

velocity_x, velocity_y = get_velocity()


# ─────────────────────────────────────────
# 한강 폴리곤
# ─────────────────────────────────────────
print("Loading Han River polygon...")
hangang = ox.features_from_place(
    "Seoul, South Korea",
    tags={"natural": "water", "water": "river"}
)

lon_min, lon_max = 126.900, 126.918
lat_min, lat_max = 37.537, 37.546

hangang_mapo = hangang.cx[lon_min:lon_max, lat_min:lat_max]
hangang_mapo = hangang_mapo[hangang_mapo.geometry.area > 0.00005]

print("Building river polygon union...")
hangang_union = unary_union(hangang_mapo.geometry)

# ─────────────────────────────────────────
# 파티클 초기화
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

TURBULENCE = 2.0
particle_vx = velocity_x + np.random.normal(0, abs(velocity_x) * TURBULENCE, N)
particle_vy = velocity_y + np.random.normal(0, abs(velocity_x) * TURBULENCE, N)

# ─────────────────────────────────────────
# 물리 파라미터
# ─────────────────────────────────────────
DT = 0.1
pvlon = particle_vx / 88000
pvlat = particle_vy / 111000
DIFFUSIVITY = 2.0 / 88000 * np.sqrt(2 * DT)
elapsed_sec = 0

accumulated_hist = np.zeros((15, 15))
best_trail_lons = [MAPO_LON]
best_trail_lats = [MAPO_LAT]

# 육지 도달 포인트
stranded_lons = []
stranded_lats = []

# ─────────────────────────────────────────
# 한강 내부 필터링
# ─────────────────────────────────────────
def filter_in_river(lons, lats):
    return np.array([
        hangang_union.contains(Point(lo, la))
        for lo, la in zip(lons, lats)
    ])

print("Computing initial river mask...")
in_river = filter_in_river(particles_lon, particles_lat)
in_river_prev = in_river.copy()

# ─────────────────────────────────────────
# 히트맵 컬러맵
# ─────────────────────────────────────────
heatmap_cmap = cm.hot_r
heatmap_cmap.set_under('none')

# ─────────────────────────────────────────
# 애니메이션
# ─────────────────────────────────────────
fig, ax = plt.subplots(figsize=(22, 10))
hangang_mapo.plot(ax=ax, color='steelblue', alpha=0.4)

ax.set_xlim(lon_min, lon_max)
ax.set_ylim(lat_min, lat_max)
ax.xaxis.set_major_formatter(ScalarFormatter(useOffset=False))
ax.yaxis.set_major_formatter(ScalarFormatter(useOffset=False))
ax.set_xlabel('Longitude', fontsize=12)
ax.set_ylabel('Latitude', fontsize=12)
ax.grid(True, alpha=0.3)

ax.scatter(MAPO_LON, MAPO_LAT, c='red', s=500, marker='*',
           zorder=10, label='Entry point (Mapo Bridge)')

scat = ax.scatter([], [], c='orange', s=15, alpha=0.6,
                  zorder=4, label='Particles')

heatmap_img = ax.imshow(
    np.zeros((15, 15)), origin='lower',
    extent=[MAPO_LON - 0.0015, MAPO_LON + 0.0015,
            MAPO_LAT - 0.0015, MAPO_LAT + 0.0015],
    cmap=heatmap_cmap, aspect='auto', alpha=0.5,
    vmin=0.01, vmax=10, zorder=3,
    interpolation='gaussian'
)

best_scat = ax.scatter([], [], c='cyan', s=80, marker='D',
                       zorder=8, label='Highest probability')

trail_line, = ax.plot([], [], c='cyan', linewidth=1.5,
                      alpha=0.7, linestyle='--', zorder=7,
                      label='Best point trail')

connect_line, = ax.plot([], [], c='red', linewidth=1.0,
                        alpha=0.5, linestyle=':', zorder=6)

# 육지 도달 포인트 마커
stranded_scat = ax.scatter([], [], c='purple', s=60, marker='X',
                           zorder=9, alpha=0.8,
                           label='Stranded on land')

top3_scats = []
top3_texts = []
for i in range(3):
    colors = ['#00FF00', '#FFFF00', '#FF8800']
    s = ax.scatter([], [], c=colors[i], s=150, marker='o',
                   zorder=7, label=f'Top {i+1}')
    t = ax.text(0, 0, '', fontsize=9, color='white', fontweight='bold',
                ha='center', va='center', zorder=9)
    top3_scats.append(s)
    top3_texts.append(t)

velocity_text = ax.text(
    0.02, 0.88,
    f'Velocity: {abs(velocity_x):.4f} m/s (West) | Turbulence: {TURBULENCE}',
    transform=ax.transAxes, fontsize=11, color='navy',
    bbox=dict(boxstyle='round', facecolor='lightblue', alpha=0.7)
)

time_text = ax.text(
    0.02, 0.95, '', transform=ax.transAxes,
    fontsize=12, color='black',
    bbox=dict(boxstyle='round', facecolor='white', alpha=0.7)
)

prob_text = ax.text(
    0.02, 0.81, '', transform=ax.transAxes,
    fontsize=10, color='darkgreen',
    bbox=dict(boxstyle='round', facecolor='lightyellow', alpha=0.7)
)

stranded_text = ax.text(
    0.02, 0.74, '', transform=ax.transAxes,
    fontsize=10, color='purple',
    bbox=dict(boxstyle='round', facecolor='lavender', alpha=0.7)
)

ax.legend(fontsize=9, loc='upper right')
ax.set_title('Han River Real-time Drift (Mapo Bridge)', fontsize=14)

def update(frame):
    global particles_lon, particles_lat, in_river, in_river_prev
    global elapsed_sec, accumulated_hist
    global best_trail_lons, best_trail_lats
    global pvlon, pvlat
    global stranded_lons, stranded_lats

    # 파티클 이동
    particles_lon += pvlon * DT + np.random.normal(0, DIFFUSIVITY, N)
    particles_lat += pvlat * DT + np.random.normal(0, DIFFUSIVITY, N)

    # 난류
    pvlon += np.random.normal(0, abs(velocity_x) * 0.05 / 88000, N)
    pvlat += np.random.normal(0, abs(velocity_x) * 0.05 / 111000, N)

    max_vlon = abs(velocity_x) * 2 / 88000
    max_vlat = abs(velocity_x) * 1 / 111000
    pvlon = np.clip(pvlon, -max_vlon, -max_vlon * 0.05)
    pvlat = np.clip(pvlat, -max_vlat, max_vlat)

    # 5프레임마다 river mask 업데이트
    if frame % 5 == 0:
        in_river_prev = in_river.copy()
        in_river = filter_in_river(particles_lon, particles_lat)

        # 강 안 → 밖으로 나간 파티클 = 육지 도달
        newly_stranded = in_river_prev & ~in_river
        if newly_stranded.any():
            stranded_lons.extend(particles_lon[newly_stranded].tolist())
            stranded_lats.extend(particles_lat[newly_stranded].tolist())

    lons_v = particles_lon[in_river]
    lats_v = particles_lat[in_river]

    if len(lons_v) > 0:
        scat.set_offsets(np.column_stack([lons_v, lats_v]))
    else:
        scat.set_offsets(np.empty((0, 2)))

    # 육지 도달 포인트 업데이트
    if len(stranded_lons) > 0:
        stranded_scat.set_offsets(
            np.column_stack([stranded_lons, stranded_lats])
        )
        stranded_text.set_text(f'Stranded points: {len(stranded_lons)}')
    else:
        stranded_scat.set_offsets(np.empty((0, 2)))
        stranded_text.set_text('')

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

        heatmap_img.set_data(hist_prob.T)
        heatmap_img.set_extent([
            center_lon - spread, center_lon + spread,
            center_lat - spread, center_lat + spread
        ])

        max_idx = np.unravel_index(hist_prob.argmax(), hist_prob.shape)
        best_lon = (xedges[max_idx[0]] + xedges[max_idx[0]+1]) / 2
        best_lat = (yedges[max_idx[1]] + yedges[max_idx[1]+1]) / 2
        best_scat.set_offsets([[best_lon, best_lat]])

        best_trail_lons.append(best_lon)
        best_trail_lats.append(best_lat)
        trail_line.set_data(best_trail_lons, best_trail_lats)
        connect_line.set_data([MAPO_LON, best_lon], [MAPO_LAT, best_lat])

        waypoints = []
        for i in range(hist_prob.shape[0]):
            for j in range(hist_prob.shape[1]):
                if hist_prob[i, j] > 0:
                    waypoints.append((
                        (xedges[i] + xedges[i+1]) / 2,
                        (yedges[j] + yedges[j+1]) / 2,
                        hist_prob[i, j]
                    ))
        waypoints.sort(key=lambda x: x[2], reverse=True)

        for i, (s, t) in enumerate(zip(top3_scats, top3_texts)):
            if i < len(waypoints):
                wlon, wlat, wprob = waypoints[i]
                s.set_offsets([[wlon, wlat]])
                t.set_position((wlon, wlat))
                t.set_text(str(i+1))
            else:
                s.set_offsets(np.empty((0, 2)))
                t.set_text('')

        prob_text.set_text(
            f'Best: ({best_lat:.5f}, {best_lon:.5f})\n'
            f'Prob: {hist_prob.max():.2f}%'
        )

    elapsed_sec += DT
    mins = int(elapsed_sec // 60)
    secs = int(elapsed_sec % 60)
    time_text.set_text(
        f'T + {mins}m {secs:02d}s | '
        f'Particles in river: {len(lons_v)}'
    )

    return (scat, heatmap_img, time_text, velocity_text,
            best_scat, prob_text, trail_line, connect_line,
            stranded_scat, stranded_text,
            *top3_scats, *top3_texts)

ani = FuncAnimation(
    fig, update,
    interval=100,
    blit=True,
    cache_frame_data=False
)

plt.tight_layout()
plt.show()