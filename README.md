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

> **히트맵 계산 방식**  
> 현재 강물 위에 있는 파티클(`lons_v`, `lats_v`)의 중심 위치를 기준으로 가로 15칸 × 세로 15칸의 격자를 생성합니다.  
> 각 격자 칸에 들어있는 파티클 수의 누적 비율(%)을 계산하여 `hist_prob` 배열에 저장합니다.

> **Waypoint 형식**  
> `(경도, 위도, 확률)` 튜플 형태로 `waypoints` 리스트에 담아 내림차순 정렬합니다.

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
├── static/
│   └── index.html       # 브라우저 시각화 (Canvas + WebSocket)
├── requirements.txt     # 패키지 목록
└── .venv/               # 가상환경 (Python 3.10)
```

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
pip install numpy matplotlib "shapely>=2.0" osmnx geopandas fiona pyproj requests networkx pyqt5 fastapi "uvicorn[standard]"
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
# SPEED = 60  →  1프레임 = 6초 시뮬레이션 (30분을 약 30초에 확인)
# SPEED = 300 →  1프레임 = 30초 시뮬레이션 (30분을 약 6초에 확인)
SPEED = 60

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
| `SPEED` | 60 | 배속 (1프레임당 스텝 수, 60 = 6초/프레임) |
| `spread` | 0.0015 | 히트맵 범위 (위경도 단위) |
| `WIDTH` | 900m | 마포대교 구간 하천 폭 |
| `DEPTH` | 6m | 마포대교 구간 평균 수심 |
| `PRINT_INTERVAL` | 180초 | 콘솔 Waypoint 출력 주기 |

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

## 참고

- [한강홍수통제소 Open API](https://www.hrfco.go.kr/web/openapiPage/openApi.do)
- [FastAPI 공식 문서](https://fastapi.tiangolo.com)
- [osmnx Documentation](https://osmnx.readthedocs.io)
- [Shapely Documentation](https://shapely.readthedocs.io)
