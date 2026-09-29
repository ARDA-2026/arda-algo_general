"""네이버 Static Map 을 '지도 사각형 bbox 그대로인 이미지' 로 구워서 캐시한다.

왜 이게 필요한가
────────────────
예전에는 화면 배경이 두 갈래였다.

    온라인 : 네이버 Web Dynamic Map(JS) 을 캔버스 뒤에 깔고, 매 프레임
             네이버 투영을 3점 샘플해 아핀을 맞춰 캔버스 좌표를 얻었다.
    오프라인: mapomap.jpg 를 지도 사각형에 늘려 채우고, 등장방형 근사
             (M_PER_DEG_LON/LAT) 로 좌표를 얻었다.

두 갈래는 투영이 다르다. 네이버 기본 투영은 UTMK(횡축 메르카토르)라 경도->x 가
선형이 아니고 격자 수렴각만큼 돌아가 있는데, 등장방형에는 그게 없다. 그래서
온/오프라인에서 같은 위경도가 다른 픽셀에 찍혔고, 파티클 좌초 판정(실제 위경도
기준 OSM 폴리곤)과 화면이 어긋나 보였다.

이 모듈은 갈래를 하나로 만든다. 서버가 미리 네이버 지도를 받아서
**bbox 에 정확히 맞는 등장방형(plate carree) 이미지** 로 다시 구워 디스크에
저장한다. 화면은 온/오프라인 구분 없이 항상 그 파일 하나를 지도 사각형에 채운다.
투영은 언제나 등장방형 하나뿐이고, 배경은 언제나 같은 파일이다. 차이가 생길
자리가 없어진다.

투영을 추측하지 않는 이유
─────────────────────────
Static Map 이 어떤 투영으로 렌더링되는지 문서에 없다. 웹 지도(JS v3)는
UTMK_NAVER 라고 적혀 있고, 모바일 SDK 문서는 Web Mercator 라고 적혀 있다.
둘은 같은 level 에서 m/px 가 크게 다르다. 잘못 고르면 지도가 통째로 어긋난다.

그래서 추측하지 않고 **잰다.** 마커 없는 이미지와 마커 하나만 찍은 이미지를
받아 픽셀 차이를 내면 그 마커의 위치가 나온다(PNG 라 두 응답은 마커 말고는
바이트가 같다). 마커를 중심에서 ±Δ 로 옮겨 두 번 재고 그 차를 쓰면, 마커
모양이나 앵커 위치를 몰라도 상쇄되어 "1도당 몇 픽셀" 이 정확히 남는다.
평행이동은 잴 필요가 없다 — center 파라미터가 곧 이미지 한가운데다.

측정 결과는 calib.json 에 남아서 한 번만 든다.
"""

from __future__ import annotations

import json
import math
import os
import threading
import time
from io import BytesIO
from pathlib import Path

import numpy as np
import requests
from PIL import Image

# ─────────────────────────────────────────
# 설정
# ─────────────────────────────────────────
CACHE_DIR = Path(__file__).with_name("cache") / "basemap"

# 신 엔드포인트가 먼저, 구 엔드포인트가 폴백. 계정 생성 시기에 따라 갈린다.
HOSTS = (
    "https://maps.apigw.ntruss.com/map-static/v2/raster",
    "https://naveropenapi.apigw.ntruss.com/map-static/v2/raster",
)

SRC_W = SRC_H = 1024      # 요청 크기 (API 상한 1024)
SRC_SCALE = 2             # 고해상도 — 실제로 받는 건 2048x2048
MAPTYPE = "basic"         # basic / satellite / satellite_base / terrain
CALIB_LEVEL = 14          # 측정용 level. 최종 level 은 bbox 크기에 맞춰 따로 고른다.
LEVEL_MIN, LEVEL_MAX = 6, 20
EDGE_MARGIN_PX = 8        # bbox 모서리를 원본 가장자리에서 이만큼 띄운다
OUT_MAX_PX = 2048         # 구워낸 이미지 최대 변 길이
TIMEOUT = 12

# 측정용 Δ 후보 (도). 큰 것부터 줄여가며, 화면 안에 적당한 간격으로
# 잡히는 첫 값을 쓴다. 투영을 모르니 m/px 를 모르고, 그래서 훑는다.
PROBE_DELTAS = (0.02, 0.006, 0.0018, 0.0005, 0.00015, 0.00004)
PROBE_MIN_PX = 60         # 이보다 가까우면 정밀도가 안 나온다
PROBE_MAX_FRAC = 0.60     # 이보다 멀면 마커가 이미지 밖으로 나간다

_lock = threading.Lock()
# 굽기 직렬화. warm() 스레드와 GET /map/background 가 같은 영역을 동시에
# 요청하면 네이버 왕복을 두 배로 쓰고, 같은 파일에 동시에 쓴다.
_build_lock = threading.Lock()
_last_fail = 0.0          # 실패 뒤 재시도 간격 (시뮬 리셋마다 두드리지 않도록)
RETRY_SEC = 60.0
_status: dict = {"state": "idle", "detail": "", "file": "", "level": None,
                 "m_per_px": None, "built_at": 0.0, "calibrated": False}


def _keys() -> tuple[str, str]:
    return (os.getenv("NAVER_MAP_STATIC_KEY_ID", "").strip(),
            os.getenv("NAVER_MAP_STATIC_KEY", "").strip())


def status() -> dict:
    """/config 로 내려보낼 현재 상태."""
    with _lock:
        s = dict(_status)
    kid, key = _keys()
    s["has_key"] = bool(kid and key)
    return s


def _set(**kw) -> None:
    with _lock:
        _status.update(kw)


# ─────────────────────────────────────────
# HTTP
# ─────────────────────────────────────────
def _fetch(center_lon: float, center_lat: float, level: int,
           marker: tuple[float, float] | None = None) -> np.ndarray:
    """Static Map 한 장을 받아 RGB 배열로 돌려준다.

    format=png 로 고정한다. jpg 면 압축 잡음 때문에 '마커 있는 장 - 없는 장'
    차이가 이미지 전체에 흩어져서 마커를 못 집는다. png 는 같은 요청이면
    픽셀이 완전히 같아서 차이가 곧 마커다.
    """
    kid, key = _keys()
    if not kid or not key:
        raise RuntimeError("NAVER_MAP_STATIC_KEY_ID / NAVER_MAP_STATIC_KEY 가 비어 있음")

    params = {
        "w": SRC_W, "h": SRC_H, "scale": SRC_SCALE,
        "center": f"{center_lon:.7f},{center_lat:.7f}",
        "level": int(level),
        "maptype": MAPTYPE,
        "format": "png",
        "lang": "ko",
    }
    if marker is not None:
        # type:d 는 기본 핀. 앵커 위치는 몰라도 된다 — 두 번 재서 차를 쓰므로 상쇄된다.
        params["markers"] = f"type:d|size:mid|pos:{marker[0]:.7f} {marker[1]:.7f}"

    # 두 가지 헤더 표기를 모두 보낸다. 게이트웨이 세대에 따라 받는 쪽이 다르다.
    headers = {
        "X-NCP-APIGW-API-KEY-ID": kid, "X-NCP-APIGW-API-KEY": key,
        "x-ncp-apigw-api-key-id": kid, "x-ncp-apigw-api-key": key,
        "Accept": "image/png",
    }

    last = None
    for host in HOSTS:
        try:
            r = requests.get(host, params=params, headers=headers, timeout=TIMEOUT)
        except Exception as e:                       # 오프라인
            last = f"{type(e).__name__}: {e}"
            continue
        if r.status_code == 200 and r.content[:4] == b"\x89PNG":
            return np.asarray(Image.open(BytesIO(r.content)).convert("RGB"))
        # 본문을 그대로 남긴다. 권한 미추가/키 오류를 화면에서 바로 읽을 수 있어야 한다.
        last = f"HTTP {r.status_code} {r.text[:200]}"
    raise RuntimeError(last or "요청 실패")


# ─────────────────────────────────────────
# 측정
# ─────────────────────────────────────────
def _marker_px(base: np.ndarray, shot: np.ndarray) -> tuple[float, float] | None:
    """마커 있는 장과 없는 장의 차이에서 마커 위치(픽셀)를 집는다.

    마커가 이미지 밖이면 차이가 없으므로 None. 가장자리에 닿으면 잘려서
    무게중심이 틀어지므로 역시 버린다.
    """
    if base.shape != shot.shape:
        return None
    d = np.abs(base.astype(np.int16) - shot.astype(np.int16)).max(axis=2)
    mask = d > 24
    if int(mask.sum()) < 40:                         # 마커가 안 보임
        return None
    ys, xs = np.nonzero(mask)
    h, w = mask.shape
    if xs.min() <= 1 or ys.min() <= 1 or xs.max() >= w - 2 or ys.max() >= h - 2:
        return None                                  # 잘렸다
    return (float(xs.mean()), float(ys.mean()))


def _measure(lon: float, lat: float, level: int, base: np.ndarray,
             axis: str, delta: float) -> tuple[float, float] | None:
    """center ±delta 두 곳에 마커를 찍어, 1도당 픽셀 변화량을 잰다.

    ±양쪽을 다 재고 차를 쓰는 이유: 마커 앵커가 핀 끝인지 한가운데인지
    모르는데, 같은 마커를 쓰는 한 그 치우침은 두 장에 똑같이 들어가서
    뺄 때 사라진다. 평행이동을 안 재도 되는 것도 같은 이유다.
    """
    dlon, dlat = (delta, 0.0) if axis == "lon" else (0.0, delta)
    pa = _marker_px(base, _fetch(lon, lat, level, (lon + dlon, lat + dlat)))
    pb = _marker_px(base, _fetch(lon, lat, level, (lon - dlon, lat - dlat)))
    if pa is None or pb is None:
        return None
    dx, dy = pa[0] - pb[0], pa[1] - pb[1]
    if math.hypot(dx, dy) < PROBE_MIN_PX:
        return None
    if max(abs(dx), abs(dy)) > PROBE_MAX_FRAC * base.shape[1]:
        return None
    return (dx / (2 * delta), dy / (2 * delta))      # 1도당 픽셀


def calibrate(lon: float, lat: float, force: bool = False) -> dict:
    """CALIB_LEVEL 에서 '1도당 픽셀' 2x2 를 실측한다. 결과는 파일에 남는다."""
    CACHE_DIR.mkdir(parents=True, exist_ok=True)
    f = CACHE_DIR / "calib.json"
    if f.exists() and not force:
        try:
            c = json.loads(f.read_text(encoding="utf-8"))
            if c.get("level") == CALIB_LEVEL and abs(c.get("lat", 99) - lat) < 0.5:
                _set(calibrated=True)
                return c
        except Exception:
            pass

    _set(state="calibrating", detail="네이버 투영 실측 중")
    base = _fetch(lon, lat, CALIB_LEVEL)

    lat_col = lon_col = None
    for d in PROBE_DELTAS:                            # 위도축 먼저 — 여기서 축척이 정해진다
        lat_col = _measure(lon, lat, CALIB_LEVEL, base, "lat", d)
        if lat_col:
            break
    if not lat_col:
        raise RuntimeError("위도축 측정 실패 — 마커가 잡히지 않음")

    # 경도축 Δ 는 이제 계산해서 고를 수 있다. 위도 1도당 픽셀을 알고,
    # 경도 1도는 대략 cos(lat) 배이므로 목표 간격을 바로 역산한다.
    per_lat = math.hypot(*lat_col)
    target = 0.30 * base.shape[1]
    d_lon = target / max(per_lat * math.cos(math.radians(lat)), 1e-9)
    for k in (1.0, 0.5, 2.0, 0.25, 4.0):
        lon_col = _measure(lon, lat, CALIB_LEVEL, base, "lon", d_lon * k)
        if lon_col:
            break
    if not lon_col:
        raise RuntimeError("경도축 측정 실패 — 마커가 잡히지 않음")

    # 자기 검증. 어떤 정각투영이든 국지적으로는 회전+등배라, 경도축 길이는
    # 위도축 길이의 cos(lat) 배여야 한다. 크게 벗어나면 마커를 잘못 집은 것이다.
    ratio = math.hypot(*lon_col) / max(per_lat, 1e-9)
    expect = math.cos(math.radians(lat))
    calib = {
        "level": CALIB_LEVEL, "lat": lat, "lon": lon,
        "xLon": lon_col[0], "yLon": lon_col[1],
        "xLat": lat_col[0], "yLat": lat_col[1],
        "src_px": int(base.shape[1]),
        "aspect": ratio, "aspect_expected": expect,
        "measured_at": time.time(),
    }
    if abs(ratio - expect) > 0.12 * expect:
        raise RuntimeError(
            f"측정 검증 실패 — 경도/위도 축척비 {ratio:.3f}, 기대 {expect:.3f}")

    f.write_text(json.dumps(calib, ensure_ascii=False, indent=2), encoding="utf-8")
    _set(calibrated=True, detail="")
    return calib


# ─────────────────────────────────────────
# 굽기
# ─────────────────────────────────────────
def _key(lon_min, lat_min, lon_max, lat_max) -> str:
    return (f"{MAPTYPE}_{lon_min:.6f}_{lat_min:.6f}_{lon_max:.6f}_{lat_max:.6f}"
            .replace(".", "p").replace("-", "m"))


def cached_path(lon_min, lat_min, lon_max, lat_max) -> Path | None:
    p = CACHE_DIR / (_key(lon_min, lat_min, lon_max, lat_max) + ".png")
    return p if p.exists() else None


def build(lon_min, lat_min, lon_max, lat_max, force: bool = False) -> Path:
    """bbox 에 정확히 맞는 등장방형 배경을 만들어 캐시에 넣고 경로를 준다."""
    out = CACHE_DIR / (_key(lon_min, lat_min, lon_max, lat_max) + ".png")
    if out.exists() and not force:
        return out

    lon_c = (lon_min + lon_max) / 2
    lat_c = (lat_min + lat_max) / 2
    calib = calibrate(lon_c, lat_c, force=force)

    dlon = lon_max - lon_min
    dlat = lat_max - lat_min
    src_px = calib["src_px"]

    # bbox 네 모서리가 원본 안에 다 들어오는 가장 촘촘한 level 을 고른다.
    # 타일 피라미드는 한 단계에 2배이므로 CALIB_LEVEL 기준으로 환산한다.
    #
    # 원본은 center 를 한가운데 두므로, 판정은 "중심에서 가장 먼 모서리까지의
    # 거리가 반폭 안쪽인가" 다. 비율(0.9 같은 값)로 어림하면 한 단계 낮은
    # level 이 뽑혀 해상도를 절반 버린다. 회전이 있어도 네 모서리를 다 보므로
    # 이 판정이 곧 정확한 포함 조건이다.
    lim = src_px / 2.0 - EDGE_MARGIN_PX
    chosen = None
    for lv in range(LEVEL_MAX, LEVEL_MIN - 1, -1):
        s = 2.0 ** (lv - calib["level"])
        fits = True
        for a in (-dlon / 2, dlon / 2):
            for b in (-dlat / 2, dlat / 2):
                x = s * (calib["xLon"] * a + calib["xLat"] * b)
                y = s * (calib["yLon"] * a + calib["yLat"] * b)
                if abs(x) > lim or abs(y) > lim:
                    fits = False
        if fits:
            chosen = lv
            break
    if chosen is None:
        raise RuntimeError("지도 영역이 너무 넓어 한 장에 담기지 않음")

    _set(state="fetching", detail=f"level {chosen}")
    src = _fetch(lon_c, lat_c, chosen)
    s = 2.0 ** (chosen - calib["level"])
    xLon, yLon = calib["xLon"] * s, calib["yLon"] * s
    xLat, yLat = calib["xLat"] * s, calib["yLat"] * s

    # 출력 크기 — 원본의 해상도를 그대로 살리되 상한을 둔다.
    ow = int(min(OUT_MAX_PX, max(256, round(abs(xLon) * dlon))))
    oh = int(min(OUT_MAX_PX, max(256, round(ow * (abs(yLat) * dlat) /
                                            max(abs(xLon) * dlon, 1e-9)))))

    # 출력 픽셀 (x,y) -> 위경도 -> 원본 픽셀 (u,v) 를 하나의 아핀으로 합친다.
    #   lon = lon_min + (x+0.5)*px_lon      lat = lat_max - (y+0.5)*px_lat
    #   u   = cx + xLon*(lon-lon_c) + xLat*(lat-lat_c)
    #   v   = cy + yLon*(lon-lon_c) + yLat*(lat-lat_c)
    px_lon, px_lat = dlon / ow, dlat / oh
    cx, cy = src.shape[1] / 2.0, src.shape[0] / 2.0
    L0 = (lon_min - lon_c) + 0.5 * px_lon
    T0 = (lat_max - lat_c) - 0.5 * px_lat
    coeffs = (
        xLon * px_lon, -xLat * px_lat, cx + xLon * L0 + xLat * T0,
        yLon * px_lon, -yLat * px_lat, cy + yLon * L0 + yLat * T0,
    )
    img = Image.fromarray(src).transform(
        (ow, oh), Image.AFFINE, coeffs, resample=Image.BICUBIC)

    CACHE_DIR.mkdir(parents=True, exist_ok=True)
    tmp = out.with_suffix(".tmp.png")
    img.save(tmp, format="PNG", optimize=True)
    tmp.replace(out)

    # 참고용 실제 축척. 위도 1도 = 111km 로 환산한 세로 기준이 가장 덜 흔들린다.
    m_per_px = (dlat * 111000.0) / oh
    (CACHE_DIR / (out.stem + ".json")).write_text(json.dumps({
        "bbox": [lon_min, lat_min, lon_max, lat_max],
        "level": chosen, "out": [ow, oh], "m_per_px": m_per_px,
        "maptype": MAPTYPE, "calib": calib, "built_at": time.time(),
    }, ensure_ascii=False, indent=2), encoding="utf-8")

    _set(state="ready", detail="", file=out.name, level=chosen,
         m_per_px=m_per_px, built_at=time.time())
    return out


def ensure(lon_min, lat_min, lon_max, lat_max, force: bool = False) -> Path | None:
    """있으면 그대로, 없으면 받아서 만든다. 실패해도 예외를 밖으로 내지 않는다.

    오프라인에서 캐시가 이미 있으면 네트워크를 아예 건드리지 않는다.
    """
    global _last_fail
    if not force:
        p = cached_path(lon_min, lat_min, lon_max, lat_max)
        if p:
            _set(state="ready", detail="캐시", file=p.name)
            return p
    with _build_lock:
        # 기다리는 동안 다른 쪽이 이미 구웠을 수 있다.
        if not force:
            p = cached_path(lon_min, lat_min, lon_max, lat_max)
            if p:
                _set(state="ready", detail="캐시", file=p.name)
                return p
        try:
            return build(lon_min, lat_min, lon_max, lat_max, force=force)
        except Exception as ex:
            _last_fail = time.time()
            _set(state="error", detail=f"{type(ex).__name__}: {ex}")
            return None


def warm(lon_min, lat_min, lon_max, lat_max) -> None:
    """서버 기동/지도 변경 직후에 백그라운드로 미리 받아 둔다.

    시연장에서는 인터넷이 없을 수 있다. 온라인일 때 미리 구워 놓아야
    오프라인에서 같은 그림이 그대로 나온다.
    """
    if cached_path(lon_min, lat_min, lon_max, lat_max):
        _set(state="ready", detail="캐시")
        return
    if not all(_keys()):
        _set(state="nokey",
             detail="NAVER_MAP_STATIC_KEY_ID / NAVER_MAP_STATIC_KEY 가 .env 에 없음")
        return
    # 시뮬은 낙하 판정마다 _rebuild_map() 을 부른다. 오프라인이면 그때마다
    # 네이버로 나가려다 타임아웃까지 붙들리므로 간격을 둔다.
    if time.time() - _last_fail < RETRY_SEC or _build_lock.locked():
        return
    threading.Thread(target=ensure, args=(lon_min, lat_min, lon_max, lat_max),
                     daemon=True).start()


if __name__ == "__main__":       # python basemap.py <lon_min> <lat_min> <lon_max> <lat_max>
    import sys
    from dotenv import load_dotenv

    load_dotenv()
    if len(sys.argv) != 5:
        print("usage: python basemap.py <lon_min> <lat_min> <lon_max> <lat_max>")
        raise SystemExit(2)
    box = [float(v) for v in sys.argv[1:5]]
    p = ensure(*box, force=True)
    print(json.dumps(status(), ensure_ascii=False, indent=2))
    raise SystemExit(0 if p else 1)
