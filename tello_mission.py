"""위경도 waypoint → 실물 축소 지도 위 Tello 기체 좌표 변환.

드론 라이브러리에 의존하지 않는 순수 계산 모듈이다.
드론을 켜지 않고도 `python tello_mission.py` 로 변환 결과를 확인할 수 있다.

────────────────────────────────────────────────────────────
좌표계 정의
────────────────────────────────────────────────────────────
  원점 (0, 0)   드론 이륙 지점 = 지도 위 입수 지점 (빨간 별 ★)
  기수(heading)  이륙 시 정북 고정. 비행 중 회전(yaw) 금지.
                 회전하면 yaw 오차가 이후 모든 이동에 누적된다.

  Tello 기체축 (기수가 북일 때)
      x (forward) = 북 (North)
      y (left)    = 서 (West)   ← 동쪽은 음수
      z (up)      = 고도

  ※ go 명령의 y 부호(좌가 +인지 우가 +인지)는 Tello SDK 에도
    djitellopy 에도 문서화되어 있지 않다. 실기로 검증한 결과 좌(+),
    즉 항공 표준 FLU 가 맞았다.

    [검증 방법] 방향이 명확한 `left` 명령을 기준자로 삼는다.
        takeoff -> move_left(50) -> go_xyz_speed(0, 50, 0, 30) -> land
        출발점에서 왼쪽 약 100cm  → 두 명령이 같은 방향 → y = 좌(+)  [확인됨]
        출발점 부근 (약 0cm)      → 서로 상쇄          → y = 우(+)

    이 부호 오류는 미션의 원점 복귀 검사로는 잡히지 않는다.
    y 를 전부 뒤집어도 경로가 좌우 거울상이 될 뿐 합은 여전히 0 이기 때문.

  ENU (실제 세계, 미터)      →  지도 미터  =  ENU / MAP_SCALE
────────────────────────────────────────────────────────────
"""

M_PER_DEG_LON = 88000    # 위도 37.5° 부근
M_PER_DEG_LAT = 111000

# Tello SDK 의 go 명령 제약 (cm)
GO_MIN_CM = 20     # 세 축 모두 20 미만이면 명령이 거부된다
GO_MAX_CM = 500


def geo_to_enu(lon, lat, origin_lon, origin_lat):
    """위경도 → 원점 기준 실제 ENU 미터 (east, north)."""
    east  = (lon - origin_lon) * M_PER_DEG_LON
    north = (lat - origin_lat) * M_PER_DEG_LAT
    return east, north


def enu_to_map(east_m, north_m, scale):
    """실제 미터 → 축소 지도 위 미터."""
    return east_m / scale, north_m / scale


def map_to_tello_cm(east_map_m, north_map_m):
    """지도 미터 → Tello 기체 좌표 cm (기수 북쪽 기준).

    x = 전진 = 북,  y = 좌 = 서 이므로 동쪽 성분은 부호가 뒤집힌다.
    """
    x_cm = north_map_m * 100.0
    y_cm = -east_map_m * 100.0
    return x_cm, y_cm


def in_map_bounds(east_map_m, north_map_m, map_w_m, map_h_m, east_margin_m):
    """지도 밖으로 나가는 좌표인지 판정.

    원점(입수 지점)은 지도 동쪽 끝에서 east_margin_m 안쪽에 있다고 본다.
    """
    x_max = east_margin_m
    x_min = east_margin_m - map_w_m
    y_max = map_h_m / 2
    y_min = -map_h_m / 2
    return (x_min <= east_map_m <= x_max) and (y_min <= north_map_m <= y_max)


def build_mission(
    waypoints,
    *,
    origin_lon,
    origin_lat,
    scale,
    map_w_m,
    map_h_m,
    east_margin_m,
    takeoff_lon=None,
    takeoff_lat=None,
    top_n=3,
):
    """waypoint 리스트를 드론이 실행할 미션으로 변환한다.

    waypoints : [{"lon":..., "lat":..., "prob":...}, ...] 확률 내림차순

    두 개의 기준점을 쓴다. 섞으면 안 된다.
      origin  = 입수 지점. 지도 사각형이 이 점을 기준으로 정의되므로
                "지도 밖인가" 판정은 반드시 이 기준으로 한다.
      takeoff = 드론이 실제로 이륙하는 자리. Tello 는 이륙 지점을 (0,0) 으로
                삼는 상대 좌표계라, go 명령용 좌표는 이 기준이어야 한다.

    반환값의 legs 는 "지령 위치" 기준 상대 이동량이다.
    Tello 는 절대 위치를 모르므로 실측이 아닌 지령 위치를 누적 추적해야 한다.
    """
    if takeoff_lon is None:
        takeoff_lon, takeoff_lat = origin_lon, origin_lat

    points   = []
    warnings = []

    for rank, wp in enumerate(waypoints[:top_n], 1):
        # ① 지도 경계 판정 — 입수 지점 기준
        east_real, north_real = geo_to_enu(wp["lon"], wp["lat"], origin_lon, origin_lat)
        east_map,  north_map  = enu_to_map(east_real, north_real, scale)
        ok = in_map_bounds(east_map, north_map, map_w_m, map_h_m, east_margin_m)

        # ② 기체 좌표 — 이륙 지점 기준
        te, tn = geo_to_enu(wp["lon"], wp["lat"], takeoff_lon, takeoff_lat)
        tem, tnm = enu_to_map(te, tn, scale)
        x_cm, y_cm = map_to_tello_cm(tem, tnm)

        if not ok:
            warnings.append(
                f"WP{rank} 이 지도 밖입니다 "
                f"(지도상 east={east_map:.2f}m north={north_map:.2f}m)"
            )

        points.append({
            "rank":        rank,
            "lon":         wp["lon"],
            "lat":         wp["lat"],
            "prob":        wp.get("prob"),
            "east_real_m":  round(east_real, 1),
            "north_real_m": round(north_real, 1),
            "map_east_m":   round(east_map, 3),
            "map_north_m":  round(north_map, 3),
            "tello_x_cm":   round(x_cm, 1),
            "tello_y_cm":   round(y_cm, 1),
            "in_bounds":    ok,
        })

    # ── 레그 생성: 원점 → WP1 → WP2 → WP3 ──
    legs = []
    cur_x, cur_y = 0.0, 0.0        # 지령 위치 (cm), 원점에서 시작
    for p in points:
        dx = p["tello_x_cm"] - cur_x
        dy = p["tello_y_cm"] - cur_y
        dist = (dx * dx + dy * dy) ** 0.5

        leg = {
            "seq":     len(legs) + 1,
            "to_rank": p["rank"],
            "dx_cm":   round(dx, 1),
            "dy_cm":   round(dy, 1),
            "dist_cm": round(dist, 1),
            "skip":    False,
            "reason":  "",
        }

        if max(abs(dx), abs(dy)) < GO_MIN_CM:
            leg["skip"]   = True
            leg["reason"] = f"이동량이 Tello 최소 {GO_MIN_CM}cm 미만"
            warnings.append(f"WP{p['rank']} 레그가 너무 짧아 건너뜁니다 ({dist:.0f}cm)")
        elif abs(dx) > GO_MAX_CM or abs(dy) > GO_MAX_CM:
            leg["skip"]   = True
            leg["reason"] = f"이동량이 Tello 최대 {GO_MAX_CM}cm 초과"
            warnings.append(f"WP{p['rank']} 레그가 {GO_MAX_CM}cm 를 넘습니다 ({dist:.0f}cm)")
        else:
            cur_x, cur_y = p["tello_x_cm"], p["tello_y_cm"]

        legs.append(leg)

    # 마지막에 이륙 지점으로 복귀 (기체 좌표 0,0)
    back_dist = (cur_x ** 2 + cur_y ** 2) ** 0.5
    legs.append({
        "seq":     len(legs) + 1,
        "to_rank": 0,
        "dx_cm":   round(-cur_x, 1),
        "dy_cm":   round(-cur_y, 1),
        "dist_cm": round(back_dist, 1),
        "skip":    back_dist < GO_MIN_CM,
        "reason":  "이미 이륙 지점 부근" if back_dist < GO_MIN_CM else "",
    })

    return {
        "origin":   {"lon": origin_lon,  "lat": origin_lat},
        "takeoff":  {"lon": takeoff_lon, "lat": takeoff_lat},
        "scale":    scale,
        "map":      {"width_m": map_w_m, "height_m": map_h_m,
                     "east_margin_m": east_margin_m},
        "points":   points,
        "legs":     legs,
        "warnings": warnings,
    }


if __name__ == "__main__":
    # 드론 없이 변환식만 눈으로 확인하는 용도.
    # 입수 지점에서 정서 100m / 정북 100m 떨어진 가상의 점 두 개.
    import json

    ORIGIN_LON, ORIGIN_LAT = 126.907, 37.540
    demo = [
        {"lon": ORIGIN_LON, "lat": ORIGIN_LAT, "prob": 9.0},                       # 원점
        {"lon": ORIGIN_LON - 100 / M_PER_DEG_LON, "lat": ORIGIN_LAT, "prob": 6.0}, # 서쪽 100m
        {"lon": ORIGIN_LON, "lat": ORIGIN_LAT + 100 / M_PER_DEG_LAT, "prob": 3.0}, # 북쪽 100m
    ]
    m = build_mission(
        demo,
        origin_lon=ORIGIN_LON, origin_lat=ORIGIN_LAT,
        scale=150, map_w_m=3.0, map_h_m=2.0, east_margin_m=50 / 150,
    )
    print(json.dumps(m, indent=2, ensure_ascii=False))
