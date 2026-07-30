from datetime import datetime
import numpy as np
import matplotlib
import matplotlib.cm as cm
matplotlib.use('QtAgg')
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
from matplotlib.ticker import ScalarFormatter
from shapely.ops import unary_union
from shapely import contains_xy
import osmnx as ox
import requests
import xml.etree.ElementTree as ET

plt.rcParams['toolbar'] = 'None'  # 툴바 제거

print("=== Han River Real-time Drift Simulation ===")

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

# velocity_x = -1.5   # 한강 평균 유속 (서쪽, m/s)
# velocity_y =  0.05  # 한강 남북 성분은 거의 0 (약한 남쪽 성분)
velocity_x, velocity_y = get_velocity()
print(f"[TEST] 유속: {abs(velocity_x)} m/s (서쪽)")

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

DT = 0.1
SPEED = 60  # 1프레임당 시뮬레이션 스텝 수 (60 = 6초/프레임, 30분을 ~30초에 확인)
pvlon = particle_vx / 88000
pvlat = particle_vy / 111000
DIFFUSIVITY = 2.0 / 88000 * np.sqrt(2 * DT)
elapsed_sec = 0

# wawypoint 출력 | 원하는 출력 주기(초) 설정
PRINT_INTERVAL = 180       # 예: 10초마다 콘솔 출력 (5, 30, 60 등 자유롭게 변경)
last_printed_time = -PRINT_INTERVAL


accumulated_hist = np.zeros((15, 15))
best_trail_lons = [MAPO_LON]
best_trail_lats = [MAPO_LAT]
stranded_lons = []
stranded_lats = []
observation = None
obs_history = []
_mouse_pressed_pos = None
_step = 0

def filter_in_river(lons, lats):
    return contains_xy(hangang_union, lons, lats)

print("Computing initial river mask...")
in_river = filter_in_river(particles_lon, particles_lat)
in_river_prev = in_river.copy()

heatmap_cmap = cm.hot_r
heatmap_cmap.set_under('none')

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

stranded_scat = ax.scatter([], [], c='purple', s=60, marker='X',
                           zorder=9, alpha=0.8, label='Stranded on land')

obs_scat = ax.scatter([], [], c='lime', s=200, marker='^',
                      zorder=11, label='Observation (click)')

obs_hist_scat = ax.scatter([], [], c='lime', s=80, marker='^',
                           zorder=10, alpha=0.4)

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

top3_coord_text = ax.text(
    0.02, 0.60, '', transform=ax.transAxes,
    fontsize=9, color='white',
    bbox=dict(boxstyle='round', facecolor='dimgray', alpha=0.7)
)

stranded_text = ax.text(
    0.02, 0.74, '', transform=ax.transAxes,
    fontsize=10, color='purple',
    bbox=dict(boxstyle='round', facecolor='lavender', alpha=0.7)
)

obs_text = ax.text(
    0.02, 0.67, 'Click: observation | Arrow: move | +/-: zoom',
    transform=ax.transAxes, fontsize=10, color='darkgreen',
    bbox=dict(boxstyle='round', facecolor='honeydew', alpha=0.7)
)

ax.legend(fontsize=9, loc='upper right')
ax.set_title('Han River Real-time Drift (Mapo Bridge) | Click: obs | Arrow: move | +/-: zoom',
             fontsize=14)

# ─────────────────────────────────────────
# 키보드 이동/줌
# ─────────────────────────────────────────
def on_key(event):
    step = 0.001
    xlim = ax.get_xlim()
    ylim = ax.get_ylim()

    if event.key == 'left':
        ax.set_xlim(xlim[0] - step, xlim[1] - step)
    elif event.key == 'right':
        ax.set_xlim(xlim[0] + step, xlim[1] + step)
    elif event.key == 'up':
        ax.set_ylim(ylim[0] + step, ylim[1] + step)
    elif event.key == 'down':
        ax.set_ylim(ylim[0] - step, ylim[1] - step)
    elif event.key == '+' or event.key == '=':
        cx = (xlim[0] + xlim[1]) / 2
        cy = (ylim[0] + ylim[1]) / 2
        xr = (xlim[1] - xlim[0]) * 0.4
        yr = (ylim[1] - ylim[0]) * 0.4
        ax.set_xlim(cx - xr, cx + xr)
        ax.set_ylim(cy - yr, cy + yr)
    elif event.key == '-':
        cx = (xlim[0] + xlim[1]) / 2
        cy = (ylim[0] + ylim[1]) / 2
        xr = (xlim[1] - xlim[0]) * 0.6
        yr = (ylim[1] - ylim[0]) * 0.6
        ax.set_xlim(cx - xr, cx + xr)
        ax.set_ylim(cy - yr, cy + yr)

    fig.canvas.draw_idle()

fig.canvas.mpl_connect('key_press_event', on_key)

# ─────────────────────────────────────────
# 마우스 클릭 (드래그 구분)
# ─────────────────────────────────────────
def on_press(event):
    global _mouse_pressed_pos
    if event.inaxes != ax:
        return
    _mouse_pressed_pos = (event.xdata, event.ydata)

def on_release(event):
    global _mouse_pressed_pos, observation, accumulated_hist
    global particles_lon, particles_lat, pvlon, pvlat

    if event.button != 1:
        _mouse_pressed_pos = None
        return
    if _mouse_pressed_pos is None:
        return
    if event.inaxes != ax or event.xdata is None or event.ydata is None:
        _mouse_pressed_pos = None
        return

    dx = abs(event.xdata - _mouse_pressed_pos[0])
    dy = abs(event.ydata - _mouse_pressed_pos[1])
    _mouse_pressed_pos = None

    if dx > 0.0005 or dy > 0.0005:
        return

    obs_lon = event.xdata
    obs_lat = event.ydata

    observation = (obs_lon, obs_lat)
    obs_history.append((obs_lon, obs_lat))

    RADIUS_OBS = 5 / 111000
    new_angles = np.random.uniform(0, 2 * np.pi, N)
    new_radii  = np.random.uniform(0, RADIUS_OBS, N)
    particles_lon[:] = obs_lon + new_radii * np.cos(new_angles)
    particles_lat[:] = obs_lat + new_radii * np.sin(new_angles)

    pvlon[:] = velocity_x / 88000 + np.random.normal(
        0, abs(velocity_x) * TURBULENCE / 88000, N)
    pvlat[:] = velocity_y / 111000 + np.random.normal(
        0, abs(velocity_x) * TURBULENCE / 111000, N)

    accumulated_hist[:] = 0
    last_printed_time = -PRINT_INTERVAL  # waypoint 출력 | 클릭 시 출력 타이머 리셋!
    obs_text.set_text(f'OBS: ({obs_lat:.5f}, {obs_lon:.5f})')
    print(f"[CLICK] 관측값 입력 → 파티클 재생성: ({obs_lat:.6f}, {obs_lon:.6f})")

fig.canvas.mpl_connect('button_press_event', on_press)
fig.canvas.mpl_connect('button_release_event', on_release)

# ─────────────────────────────────────────
# 애니메이션 업데이트
# ─────────────────────────────────────────
def update(frame):
    global particles_lon, particles_lat, in_river, in_river_prev
    global elapsed_sec, accumulated_hist
    global best_trail_lons, best_trail_lats
    global pvlon, pvlat
    global stranded_lons, stranded_lats
    global _step
    global last_printed_time  # waypoiint 출력

    max_vlon = abs(velocity_x) * 2 / 88000
    max_vlat = abs(velocity_x) * 1 / 111000

    for _ in range(SPEED):
        particles_lon += pvlon * DT + np.random.normal(0, DIFFUSIVITY, N)
        particles_lat += pvlat * DT + np.random.normal(0, DIFFUSIVITY, N)

        pvlon += np.random.normal(0, abs(velocity_x) * 0.05 / 88000, N)
        pvlat += np.random.normal(0, abs(velocity_x) * 0.05 / 111000, N)

        pvlon = np.clip(pvlon, -max_vlon, -max_vlon * 0.05)
        pvlat = np.clip(pvlat, -max_vlat, max_vlat)

        elapsed_sec += DT
        _step += 1

        if _step % 5 == 0:
            in_river_prev = in_river.copy()
            in_river = filter_in_river(particles_lon, particles_lat)
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

    if len(stranded_lons) > 0:
        stranded_scat.set_offsets(
            np.column_stack([stranded_lons, stranded_lats])
        )
        stranded_text.set_text(f'Stranded points: {len(stranded_lons)}')
    else:
        stranded_scat.set_offsets(np.empty((0, 2)))

    if observation:
        obs_scat.set_offsets([[observation[0], observation[1]]])
    else:
        obs_scat.set_offsets(np.empty((0, 2)))

    if len(obs_history) > 1:
        obs_hist_scat.set_offsets(
            np.array([[o[0], o[1]] for o in obs_history[:-1]])
        )
    else:
        obs_hist_scat.set_offsets(np.empty((0, 2)))

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

        if elapsed_sec - last_printed_time >= PRINT_INTERVAL: # waypoint 출력
            last_printed_time = elapsed_sec
            mins = int(elapsed_sec // 60)
            secs = int(elapsed_sec % 60)
            
            print(f"\n========================================")
            print(f"⏱️ [시뮬레이션 시간: {mins}분 {secs:02d}초 (T+{int(elapsed_sec)}s)]")
            if waypoints:
                for idx, wp in enumerate(waypoints[:3]):
                    wlon, wlat, wprob = wp
                    print(f" 📍 Top {idx+1}: 위도 {wlat:.6f}, 경도 {wlon:.6f} | 확률: {wprob:.2f}%")
            else:
                print(" ⚠️ 유효한 Waypoint가 없습니다.")
            print(f"========================================\n")

        coord_lines = []
        for i, wp in enumerate(waypoints[:3]):
            wlon, wlat, wprob = wp
            coord_lines.append(f'Top{i+1}: ({wlat:.5f}, {wlon:.5f}) {wprob:.1f}%')
        top3_coord_text.set_text('\n'.join(coord_lines))

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

    mins = int(elapsed_sec // 60)
    secs = int(elapsed_sec % 60)
    time_text.set_text(
        f'T + {mins}m {secs:02d}s | '
        f'Particles in river: {len(lons_v)}'
    )

    return (scat, heatmap_img, time_text, velocity_text,
            best_scat, prob_text, trail_line, connect_line,
            stranded_scat, stranded_text,
            obs_scat, obs_hist_scat, obs_text,
            top3_coord_text,
            *top3_scats, *top3_texts)

ani = FuncAnimation(
    fig, update,
    interval=100,
    blit=True,
    cache_frame_data=False
)

plt.tight_layout()
plt.show()