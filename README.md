# Person in distress Drift Prediction System (Han River)

> 창의적 종합설계 | 인천대학교 임베디드시스템공학과  
> Team ARDA | 2026

---

## 프로젝트 개요

익수자 발생 시 **실시간 표류 위치를 예측**하여 수색·구조 효율을 높이는 시스템입니다.

기존 수난 사고 대응은 구조대원의 경험에 의존하는 경우가 많아, 익수자 위치 특정에 어려움이 있습니다. 따라서 한강 실시간 유속 데이터와 파티클 필터 알고리즘을 결합하여 익수자가 있을 확률이 높은 구역을 히트맵으로 시각화하고, 수색 우선순위 Waypoint를 제공하는 모델입니다.

해당 모델을 한강 - 마포대교 기준으로 진행합니다.

---

## 시스템 동작 흐름

```
[입수 감지]
열화상 카메라로 입수 순간 좌표 포착

↓

[초기 예측]
입수 좌표 + 실시간 유속 데이터 → 파티클 기반 표류 예측 시작

↓

[분기] 카메라/레이더가 재감지 성공?

├── YES → 브라우저 캔버스 클릭으로 관측값 입력 (POST /observation)
│         → 파티클 재수렴 (확률 수렴)
│         → 다시 분기로
│
└── NO  → 파티클이 퍼지면서 불확실성 증가
          → 히트맵 + Waypoint로 수색대 or 드론 유도
```

---

## 주요 기능

| 기능 | 설명 |
|------|------|
| 실시간 유속 수신 | 한강홍수통제소 API (한강대교 관측소 `1018683`) |
| 유량 → 유속 역산 | `유속 = 유량 / (폭 × 수심)` (마포대교 단면 기준) |
| 파티클 표류 시뮬레이션 | 유속 + 난류(Turbulence) + 확산 계수 적용 |
| 한강 폴리곤 마스킹 | OSM 데이터 기반 한강 본류 폴리곤 추출 (Shapely) |
| 육지 도달 감지 | 파티클이 한강 밖으로 나가는 순간 위치 기록 |
| 누적 히트맵 | 표류 확률 분포를 시간 누적으로 안정적으로 시각화 |
| 수색 우선순위 TOP3 | 확률 높은 구역 순서대로 Waypoint 생성 |
| 관측값 입력 | 브라우저 캔버스 클릭 위치를 파티클 필터 관측값으로 업데이트 |
| 베스트 포인트 궤적 | 가장 확률 높은 지점의 이동 경로 시각화 |
| 육지 도달 포인트 마커 | 익수자가 표류 후 쓰러져 있을 수 있는 육지 지점 표시 |
| 배속 시뮬레이션 | `SPEED` 파라미터로 N분 후 위치를 빠르게 예측 |
| **드론 수색 연동** | Waypoint를 축소 지도 위 Tello 좌표로 변환해 실기체 자율 비행 |

> **히트맵 계산 방식**  
> 축소 지도 영역(실제 450m × 300m)에 **고정된** 가로 30칸 × 세로 20칸 격자(칸당 15m)를 씁니다.  
> 매 스텝 강물 위 파티클을 격자에 누적하되, 과거 기여도를 지수 감쇠(반감기 20초)시켜 **최근 체류 시간** 분포를 만듭니다.
>
> 격자를 파티클 무게중심에 맞춰 매번 재생성하면 서로 다른 좌표계의 카운트를 더하게 되어 누적 확률이 뭉개집니다. 그래서 격자를 지도에 못박았습니다.

> **Waypoint 형식**  
> `{"lon":…, "lat":…, "prob":…}` 형태로 확률 내림차순 정렬합니다.  
> `prob` 은 **누적 확률이 아니라 최근 체류 비율(%)** 입니다.
>
> 확률 상위 칸은 서로 인접해 있어 그대로 쓰면 수색 구역이 한 점에 뭉칩니다.  
> 비최대 억제(NMS)로 **서로 100m 이상 떨어진 지점만** 골라냅니다.

---

## 기술 스택

| 분류 | 기술 |
|------|------|
| 파티클 시뮬레이션 | NumPy 기반 커스텀 파티클 필터 |
| 한강 맵 | OSM (osmnx) + Shapely / GeoPandas |
| 실시간 유속 | 한강홍수통제소 Open API (HRFCO) |
| 백엔드 서버 | **FastAPI + Uvicorn** |
| 실시간 통신 | **WebSocket** (100ms 주기 상태 전송) |
| 프론트엔드 | **HTML5 Canvas + Vanilla JS** (브라우저) |
| 탐지 하드웨어 | 열화상 카메라 + 서보 모터 |
| 좌표 변환 | OpenCV Homography (픽셀 → 실제 좌표) |
| 통합 아키텍처 | ROS2 (예정) |

---

## 프로젝트 구조

```
arda-algo_general/
├── hanriver.py          # FastAPI 서버 + 시뮬레이션 엔진
├── drone_api.py         # 드론 엔드포인트 (/mission, /drone/*)
├── tello_mission.py     # 위경도 → 축소 지도 → Tello 기체좌표 변환
├── tello_driver.py      # Tello 제어 (djitellopy) + DRY-RUN 모드
├── fly.py               # ★ 드론 데모 실행 파일 (이것 하나만 실행)
├── drone_cli.py         # 개별 명령 CLI (비상 착륙 등)
├── static/
│   └── index.html       # 브라우저 시각화 (Canvas + WebSocket)
├── cache/               # OSM 폴리곤 캐시 (삭제 금지 — 오프라인 실행에 필요)
├── requirements.txt     # 패키지 목록
└── .venv/               # 가상환경 (Python 3.10)
```

> ⚠️ `cache/` 를 지우면 인터넷 없이 서버가 뜨지 않습니다.  
> Tello WiFi에 연결하면 인터넷이 끊기므로 이 캐시가 필수입니다.

---

## 설치 방법

### 환경 요구사항

- Python 3.10

### 가상환경 구축

```powershell
# 1. 가상환경 생성 (Python 3.10)
py -3.10 -m venv .venv

# 2. 가상환경 활성화
.\.venv\Scripts\Activate.ps1

# 3. pip 업그레이드
python -m pip install --upgrade pip

# 4. 패키지 설치
pip install numpy matplotlib "shapely>=2.0" osmnx geopandas fiona pyproj requests networkx pyqt5 fastapi "uvicorn[standard]" python-dotenv djitellopy
```

또는 requirements.txt로 한 번에 설치:

```powershell
pip install -r requirements.txt
```

---

## 실행 방법

```powershell
# 1. 프로젝트 폴더로 이동
cd c:\Users\kmmjj\SS\arda-algo_general

# 2. 가상환경 활성화
.\.venv\Scripts\Activate.ps1

# 3. 서버 실행
python hanriver.py
```

서버 기동 후 브라우저에서 접속:

```
http://localhost:8000
```

### 종료 방법

| 상황 | 방법 |
|------|------|
| 서버 종료 | 터미널에서 `Ctrl + C` |
| 가상환경 비활성화 | 터미널에서 `deactivate` 입력 |

---

## 실행 전 설정

`hanriver.py` 에서 다음 값을 필요에 따라 변경하세요:

```python
# 난류 강도 (0.0 ~ 1.0)
TURBULENCE = 0.3

# 배속 (1프레임당 시뮬레이션 스텝 수)
# SPEED = 1   →  약 1.5~2배속 (드론 실기 연동용 기본값)
# SPEED = 60  →  약 90배속 (화면으로 빠르게 훑어볼 때)
#
# 드론 연동 시 반드시 1 로 두세요.
# 60 이면 드론이 이륙하기도 전에 파티클이 지도를 벗어납니다.
SPEED = 1

# 입수 지점 (실제: 열화상 카메라 감지 좌표)
MAPO_LAT = 37.540
MAPO_LON = 126.907
```

---

### 유속 모드 전환

#### 기본값 — 고정 유속 (git clone 직후 바로 실행 가능)

API 키 없이도 즉시 실행됩니다.

```python
# hanriver.py 의 유속 설정 부분 (기본 상태)
velocity_x = -1.5   # m/s, 서쪽 방향 (한강 평균)
velocity_y =  0.05  # m/s, 남쪽 방향

# velocity_x, velocity_y = get_velocity()  ← 주석 처리된 상태
```

#### API 모드 — HRFCO 실시간 유속

**Step 1.** 한강홍수통제소에서 API 키 발급  
→ [https://www.hrfco.go.kr/web/openapiPage/openApi.do](https://www.hrfco.go.kr/web/openapiPage/openApi.do)

**Step 2.** `.env` 파일에 키 입력

```bash
# .env
API_KEY=발급받은키를여기에입력
```

**Step 3.** `hanriver.py` 유속 설정 부분을 아래처럼 변경

```python
# velocity_x = -1.5   ← 이 두 줄을 주석 처리
# velocity_y =  0.05

velocity_x, velocity_y = get_velocity()  # ← 이 줄 주석 해제
```

> API 호출 실패 시 자동으로 기본값(`-0.05 m/s`)으로 대체되어 시뮬레이션은 계속 실행됩니다.

---

## API 엔드포인트

| 메서드 | 경로 | 설명 |
|--------|------|------|
| `GET` | `/` | 브라우저 시각화 페이지 |
| `GET` | `/state` | 현재 시뮬레이션 상태 (JSON) |
| `GET` | `/river-geojson` | 한강 폴리곤 GeoJSON |
| `POST` | `/observation` | 관측값 입력 `{"lon": ..., "lat": ...}` |
| `WS` | `/ws` | WebSocket 실시간 상태 스트림 |
| `GET` | `/mission` | Waypoint를 Tello 기체좌표로 변환 (`?top_n=3`) |
| `POST` | `/drone/connect` | 기체 연결 `{"dry_run": true}` |
| `POST` | `/drone/start` | 미션 실행 `{"top_n":3,"speed":30,"hover_sec":3}` |
| `POST` | `/drone/land` | 즉시 착륙 (비상 정지) |
| `GET` | `/drone/status` | 배터리·진행률·종료 사유 |

---

## 조작 및 지도 제어 방법

| 구분 | 조작 / 단축키 | 기능 |
|------|----------------|------|
| **관측값 입력** | 클릭 (드래그 제외) | 클릭 위치를 관측값으로 입력 → 파티클 재수렴 |
| **지도 이동** | 마우스 드래그 | 지도 화면 자유 이동 (무제한) |
| **지도 줌** | 스크롤 휠 | 마우스 커서 위치 기준 줌 인/아웃 |
| **초기화** | 🏠 초기화 버튼 / `R` 키 | 원래 마포대교 범위로 카메라 복귀 |
| **따라가기** | 📍 따라가기 버튼 / `F` 키 | 최고 확률 지점을 화면 중앙에 자동 유지 (트래킹) |
| **버튼 줌** | `＋` / `－` 버튼 / `+`, `-` 키 | 화면 중앙 기준 확대 / 축소 |
| **패닝** | 화살표 키 (`↑`, `↓`, `←`, `→`) | 화면 15% 단위 영역 이동 |

---


## 화면 구성

| 요소 | 색상/모양 | 설명 |
|------|----------|------|
| 입수 지점 | 빨간 별 ★ | 익수자 입수 좌표 |
| 파티클 | 주황 점 | 표류 예측 파티클 (200개) |
| 히트맵 | hot_r 컬러맵 | 누적 표류 확률 분포 |
| 최고 확률 지점 | 청록 다이아몬드 ◆ | 현재 가장 확률 높은 위치 |
| 베스트 포인트 궤적 | 청록 점선 | 최고 확률 지점 이동 경로 |
| 수색 우선순위 | 초록/노랑/주황 원 | TOP 1/2/3 수색 구역 |
| 육지 도달 포인트 | 보라 X | 파티클이 육지에 닿은 지점 |
| 관측값 | 라임 삼각형 ▲ | 캔버스 클릭으로 입력한 관측값 |

---

## 파라미터 설명

| 파라미터 | 기본값 | 설명 |
|----------|--------|------|
| `N` | 200 | 파티클 수 (많을수록 정확하나 느림) |
| `TURBULENCE` | 0.3 | 난류 강도 (0=층류, 1.0=최대 난류) |
| `DIFFUSIVITY` | 2.0 | 확산 계수 (클수록 넓게 퍼짐) |
| `DT` | 0.1 | 시뮬레이션 타임스텝 (초) |
| `SPEED` | 1 | 배속 (1프레임당 스텝 수). **드론 연동 시 1 고정** |
| `WIDTH` | 900m | 마포대교 구간 하천 폭 |
| `DEPTH` | 6m | 마포대교 구간 평균 수심 |
| `PRINT_INTERVAL` | 180초 | 콘솔 Waypoint 출력 주기 |

### 지도 · 격자 · Waypoint

| 파라미터 | 기본값 | 설명 |
|----------|--------|------|
| `MAP_SCALE` | 150 | 축척 1:150 |
| `MAP_W_M` / `MAP_H_M` | 450m / 300m | 지도가 담는 실제 영역 (실물 3m × 2m) |
| `MAP_EAST_M` | 50m | 입수 지점을 지도 동쪽 끝에서 안쪽으로 둘 거리 |
| `GRID_NX` / `GRID_NY` | 30 / 20 | 누적 격자 칸 수 (칸당 15m) |
| `HIST_HALF_LIFE_SEC` | 20초 | 히트맵 망각 반감기 |
| `NMS_MIN_DIST_M` | 100m | Waypoint 간 최소 간격 (지도상 67cm) |
| `NMS_MAX_COUNT` | 10 | Waypoint 최대 개수 |

---

## 콘솔 출력 제어

`hanriver.py` 상단의 `VERBOSE` 플래그 하나로 모든 콘솔 출력을 켜고 끌 수 있습니다.

```python
VERBOSE = True   # 기본값: 시작 로그 + 주기적 Waypoint 출력
VERBOSE = False  # 모든 print 완전 중단
```

| `VERBOSE` | 출력되는 내용 |
|-----------|--------------|
| `True` | 서버 시작 로그, API 유속 수신, 폴리곤 로드, 주기적 Waypoint 콘솔 출력 |
| `False` | 아무것도 출력 안 됨 (uvicorn 내부 로그도 `log_level="warning"` 으로 최소화) |

> **Waypoint 주기** 는 `PRINT_INTERVAL = 180` (초) 로 조정합니다. `60` 이면 1분마다 출력.

---

## Waypoint 외부 연동

서버가 실행 중이면 아래 방법으로 Waypoint 데이터를 꺼내 쓸 수 있습니다.

### Waypoint JSON 형식

```json
[
  { "lon": 126.90430, "lat": 37.53980, "prob": 8.5 },
  { "lon": 126.90360, "lat": 37.54010, "prob": 6.2 },
  { "lon": 126.90210, "lat": 37.53950, "prob": 5.1 }
]
```

- `lon` / `lat`: 경도 / 위도 (WGS84)  
- `prob`: 해당 격자 칸의 누적 표류 확률 (%)  
- 확률 내림차순 정렬, 최대 10개 제공

---

### 방법 1 — HTTP GET (가장 간단)

```python
import requests

data = requests.get("http://localhost:8000/state").json()
waypoints = data["waypoints"]          # 리스트, 확률 내림차순

for i, wp in enumerate(waypoints[:3], 1):
    print(f"Top{i}: ({wp['lat']:.5f}, {wp['lon']:.5f})  {wp['prob']:.1f}%")
```

---

### 방법 2 — WebSocket (실시간 스트림)

매 100ms마다 최신 상태를 수신합니다.

```python
import asyncio, json, websockets

async def watch_waypoints():
    async with websockets.connect("ws://localhost:8000/ws") as ws:
        while True:
            data = json.loads(await ws.recv())
            waypoints = data["waypoints"]
            print(waypoints[:3])

asyncio.run(watch_waypoints())
```

```bash
pip install websockets
```

---

### 방법 3 — ROS2 연동 예시

```python
import requests
from geometry_msgs.msg import PoseArray, Pose

def publish_waypoints(pub):
    data = requests.get("http://localhost:8000/state").json()
    pose_array = PoseArray()
    for wp in data["waypoints"][:10]:
        pose = Pose()
        pose.position.x = wp["lon"]
        pose.position.y = wp["lat"]
        pose_array.poses.append(pose)
    pub.publish(pose_array)
```

---

## 실제 시스템 연동 시 수정 포인트

| 항목 | 현재 | 실제 연동 시 |
|------|------|-------------|
| 유속 | HRFCO API 자동 수신 (실패 시 고정값) | HRFCO API 안정적 수신 |
| 입수 지점 | 하드코딩 (마포대교) | 열화상 카메라 감지 좌표 |
| 관측값 | 브라우저 캔버스 클릭 → POST /observation | 열화상 카메라 재감지 좌표 자동 전송 |
| Waypoint 출력 | 브라우저 패널 + 콘솔 출력 | ROS2 PoseArray 토픽 발행 |
| 좌표 변환 | 없음 | OpenCV Homography |

---

## 드론 수색 연동 (DJI Tello)

예측된 Waypoint로 **Tello가 축소 지도 위를 실제로 비행**합니다.

### 왜 축소 지도인가

Tello에는 **GPS가 없습니다.** `go x y z speed`(현재 위치 기준 상대 이동, cm)만 있습니다. 게다가 실제 탐색 영역은 수백 미터라 실물 크기로는 날 수 없습니다.

그래서 **실제 450m × 300m 를 3m × 2m 축소 지도(축척 1:150)로 옮겨** 그 위를 비행시킵니다.

### 실물 배치

```
                    지도 북쪽 (N)
      ┌──────────────────────────────────────────┐  ─┐
      │                                          │   │
      │                                     ▲    │  100cm
  서  │                                    (0,0) │  ─┤  2.0 m
 (W)  │                                   드론   │  100cm
      │                                          │   │
      └──────────────────────────────────────────┘  ─┘
      |←─────────────── 3.0 m ─────────────→|33.3cm|
                                                    동 (E)
```

| 항목 | 값 |
|------|-----|
| 실물 지도 | 3.0m (동서) × 2.0m (남북) |
| 축척 | 1:150 (실제 450m × 300m) |
| 원점(이륙 지점) | 지도 동쪽 끝에서 **33.3cm** 안쪽, 남북 **정중앙** |
| 기수 방향 | **지도 북쪽 고정. 비행 중 회전 금지** |
| 필요 공간 | 최소 4m × 3m, 권장 5m × 4m (천장 2.5m 이상) |

**기수 정렬이 가장 중요합니다.** 회전하면 yaw 오차가 이후 모든 이동에 누적되는데, 일반 Tello는 실제로 몇 도 돌았는지 알려주지 않아 보정이 불가능합니다. 지도에 `N` 화살표를 표시해두고 거기 맞춰 내려놓으세요.

> 인쇄 지도는 광류 센서에 유리합니다. 무늬가 풍부해 단색 바닥보다 위치 추정이 안정적입니다.

### 좌표계

```
원점 (0,0)  = 드론 이륙 지점 = 지도 위 입수 지점 (빨간 별 ★)

Tello 기체축 (기수가 북일 때)
    x (forward) = 북 (North)
    y (left)    = 서 (West)   ← 동쪽은 음수
```

`go` 명령의 y 부호는 Tello SDK에도 djitellopy에도 문서화되어 있지 않습니다. 방향이 명확한 `left` 명령을 기준자로 삼아 **실기 검증한 결과 좌(+) = 항공 표준 FLU** 가 맞았습니다.

### 실행

**창 1 — 서버**
```powershell
python hanriver.py
```

**창 2 — 드론**
```powershell
python fly.py            # 실기체 비행
python fly.py --dry      # 드론 없이 예행 연습
python fly.py --yes      # 확인 프롬프트 생략
```

| 옵션 | 기본값 | 설명 |
|------|--------|------|
| `--top` | 3 | 수색 지점 수 (1~10) |
| `--speed` | 30 | 비행 속도 cm/s (10~100) |
| `--hover` | 3.0 | 지점별 정지 시간 (초) |
| `--wait` | 100 | 파티클 확산 최대 대기 (초) |

`fly.py` 가 하는 일:

1. 서버 확인
2. **WP1이 이륙 지점에서 충분히 멀어질 때까지 대기** — 서버 켜자마자 날리면 파티클이 입수 지점에 몰려 있어 WP1이 원점과 겹치고, Tello 최소 이동거리 20cm 미만이라 최우선 수색 지점을 건너뛰게 됩니다
3. 기체 연결 + 배터리 확인
4. 미션 미리보기 + 경고 표시
5. 확인 후 비행, 진행 상황 실시간 출력
6. 종료 사유 판정

### 개별 명령

```powershell
python drone_cli.py status     # 상태
python drone_cli.py mission    # 좌표 변환만 확인 (드론 불필요)
python drone_cli.py land       # 비상 착륙
python drone_cli.py watch      # 실시간 감시
```

> **비상 착륙(`drone_cli.py land`)을 다른 창에 미리 쳐두고 Enter 대기 상태로 두세요.**  
> `fly.py` 의 `Ctrl+C` 는 그 프로세스가 살아있어야 동작합니다.

### 안전장치

| 조건 | 동작 |
|------|------|
| 배터리 30% 미만 | 이륙 거부 |
| 지도 밖 waypoint | **미션 거부** |
| 레그 500cm 초과 | 미션 거부 |
| 레그 20cm 미만 | 해당 레그 건너뜀 |
| 예외 발생 | `finally` 에서 **무조건 착륙** |
| `/drone/land` | 즉시 중단 후 착륙 |

### 미션 종료 사유

`GET /drone/status` 의 `result` 필드로 판정합니다.

| 값 | 의미 |
|----|------|
| `completed` | 정상 완료 — 모든 레그 실행 후 원점 복귀 |
| `aborted` | 사용자 중단 |
| `error` | 예외 발생 (착륙 실패 포함) |
| `incomplete` | 레그 일부 미실행 |

> ⚠️ `completed` 는 **"명령을 다 보냈다"**는 뜻이지 **"정확한 좌표에 도달했다"**는 뜻이 아닙니다.  
> 일반 Tello는 절대 위치 센서가 없어 실제 오차를 알 수도, 보정할 수도 없습니다.  
> 실내 광류 드리프트는 통상 ±20~50cm이며, Waypoint 간 최소 간격(지도상 67cm)보다 작아 구역 구분에는 지장이 없습니다.

### 네트워크

Tello는 자체 WiFi AP를 띄웁니다. 노트북을 `TELLO-XXXXXX` 에 연결하면 **인터넷이 끊기지만** 다음 이유로 문제없습니다.

| 인터넷 필요 항목 | 상태 |
|------------------|------|
| OSM 폴리곤 다운로드 | `cache/` 에 캐시됨 — 오프라인 OK |
| HRFCO 실시간 유속 | 기본값이 고정 유속이라 무관 |
| FastAPI ↔ 브라우저 | `localhost` — 네트워크 무관 |

**노트북 1대, 내장 WiFi, 동글 없이 완결됩니다.** 브라우저 화면을 다른 기기에 띄우려면 그때만 USB WiFi 동글이 필요합니다.

---

## 참고

- [한강홍수통제소 Open API](https://www.hrfco.go.kr/web/openapiPage/openApi.do)
- [DJITelloPy](https://github.com/damiafuentes/DJITelloPy)
- [FastAPI 공식 문서](https://fastapi.tiangolo.com)
- [osmnx Documentation](https://osmnx.readthedocs.io)
- [Shapely Documentation](https://shapely.readthedocs.io)
