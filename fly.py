"""드론 데모 실행 파일 - 이것 하나만 실행하면 된다.

    python fly.py            실기체 비행
    python fly.py --dry      드론 없이 예행 연습
    python fly.py --yes      확인 프롬프트 없이 바로 (리허설 끝난 뒤)

────────────────────────────────────────────────────────────
하는 일
────────────────────────────────────────────────────────────
  1. 서버(hanriver.py) 가 떠 있는지 확인
  2. WP1 이 이륙 지점에서 충분히 멀어질 때까지 대기
     (서버 켜자마자 날리면 파티클이 입수 지점에 몰려 있어서
      WP1 이 원점과 겹치고, Tello 최소 이동거리 20cm 미만이라
      "수색 우선순위 1번"을 건너뛰게 된다)
  3. 기체 연결 + 배터리 확인
  4. 미션 미리보기 출력, 경고 있으면 표시
  5. 사용자 확인 후 비행
  6. 진행 상황 실시간 출력, 종료 사유 판정

  Ctrl+C 를 누르면 즉시 착륙 명령을 보낸다.

────────────────────────────────────────────────────────────
사전 준비
────────────────────────────────────────────────────────────
  - 다른 창에서  python hanriver.py  실행 중
  - 노트북 WiFi 가 TELLO-XXXXXX 에 연결됨 (실기체일 때)
  - 지도 위 원점에 드론 배치, 기수는 지도 북쪽
"""

import argparse
import sys
import time

import requests

BASE = "http://localhost:8000"

# 파티클이 지도 서쪽 끝에 닿기 전까지가 데모 유효 구간이다.
# 이 시간을 넘겨서까지 기다리지 않는다.
MAX_WAIT_SEC = 100


def pr(*a):
    """콘솔 인코딩(cp949)에서 죽지 않는 print."""
    line = " ".join(str(x) for x in a)
    try:
        print(line)
    except Exception:
        enc = sys.stdout.encoding or "utf-8"
        print(line.encode(enc, "replace").decode(enc, "replace"))


def call(method, path, body=None, timeout=60):
    try:
        r = requests.request(method, BASE + path, json=body, timeout=timeout)
    except requests.exceptions.ConnectionError:
        pr("")
        pr("  [X] 서버에 연결할 수 없습니다.")
        pr("      다른 창에서 먼저 실행하세요:  python hanriver.py")
        sys.exit(1)
    try:
        data = r.json()
    except Exception:
        pr(f"  [X] HTTP {r.status_code}: {r.text[:200]}")
        sys.exit(1)
    if r.status_code >= 400:
        return None, data.get("detail", data)
    return data, None


def wait_for_spread(top_n, max_wait):
    """WP1 레그가 스킵되지 않을 때까지 대기한다."""
    pr("")
    pr("  [2] 파티클이 퍼지기를 기다립니다")
    pr("      (WP1 이 이륙 지점과 겹치면 Tello 최소 이동거리 미만이라 스킵됨)")

    t0 = time.time()
    while True:
        m, err = call("GET", f"/mission?top_n={top_n}")
        waited = time.time() - t0

        if m is not None:
            leg1 = m["legs"][0] if m["legs"] else None
            if leg1 and not leg1["skip"]:
                pr(f"      준비됨 ({waited:.0f}초 대기, WP1 까지 {leg1['dist_cm']:.0f}cm)")
                return m
            reason = f"WP1 거리 {leg1['dist_cm']:.0f}cm" if leg1 else "waypoint 없음"
        else:
            reason = str(err)[:40]

        if waited > max_wait:
            pr(f"      [!] {max_wait}초 초과. 현재 상태로 진행합니다.")
            pr(f"          (파티클이 이미 지도를 벗어났을 수 있습니다 - 서버 재시작 권장)")
            m, _ = call("GET", f"/mission?top_n={top_n}")
            return m

        pr(f"      대기 중... {waited:4.0f}초  ({reason})")
        time.sleep(5)


def show_mission(m):
    pr("")
    pr("  [4] 미션 미리보기")
    pr(f"      축척 1:{m['scale']}   지도 {m['map']['width_m']}m x {m['map']['height_m']}m")
    for p in m["points"]:
        flag = "" if p["in_bounds"] else "   << 지도 밖!"
        pr(f"      WP{p['rank']}  prob={p['prob']:5.2f}%  "
           f"Tello x={p['tello_x_cm']:+7.1f} y={p['tello_y_cm']:+7.1f} cm{flag}")
    pr("      ── 비행 경로 ──")
    for l in m["legs"]:
        dest = "원점복귀" if l["to_rank"] == 0 else f"WP{l['to_rank']}"
        tag = "SKIP" if l["skip"] else " GO "
        pr(f"      [{tag}] leg{l['seq']} -> {dest:8s} "
           f"go({l['dx_cm']:+7.1f}, {l['dy_cm']:+7.1f})  {l['dist_cm']:6.1f}cm")
    if m["warnings"]:
        pr("      ── 경고 ──")
        for w in m["warnings"]:
            pr(f"      ! {w}")
    return not any(not p["in_bounds"] for p in m["points"])


def monitor():
    """비행이 끝날 때까지 진행 상황을 출력한다."""
    pr("")
    pr("  [6] 비행 중  (Ctrl+C = 즉시 착륙)")
    last = None
    while True:
        st, err = call("GET", "/drone/status", timeout=10)
        if st is None:
            pr(f"      상태 조회 실패: {err}")
            time.sleep(1)
            continue

        key = (st["phase"], st["leg_done"])
        if key != last:
            pr(f"      {st['phase']:8s}  leg {st['leg_done']}/{st['leg_total']}  "
               f"지령 x={st['cur_x_cm']:+7.1f} y={st['cur_y_cm']:+7.1f} cm")
            last = key

        if not st["running"]:
            return st
        time.sleep(0.7)


def main():
    ap = argparse.ArgumentParser(description="한강 표류 예측 드론 데모")
    ap.add_argument("--dry",   action="store_true", help="드론 없이 예행 연습")
    ap.add_argument("--yes",   action="store_true", help="확인 프롬프트 생략")
    ap.add_argument("--top",   type=int,   default=3,  help="수색 지점 수 (1~10)")
    ap.add_argument("--speed", type=int,   default=30, help="비행 속도 cm/s (10~100)")
    ap.add_argument("--hover", type=float, default=3.0, help="지점별 정지 시간 초")
    ap.add_argument("--wait",  type=int,   default=MAX_WAIT_SEC, help="최대 대기 초")
    a = ap.parse_args()

    mode = "예행연습 (DRY-RUN)" if a.dry else "실기체 비행"
    pr("=" * 64)
    pr(f"  한강 표류 예측 - 드론 수색 데모   [{mode}]")
    pr("=" * 64)

    # [1] 서버 확인
    st, err = call("GET", "/state")
    if st is None:
        pr(f"  [X] 시뮬레이션 상태를 읽을 수 없습니다: {err}")
        sys.exit(1)
    pr(f"  [1] 서버 확인됨  (시뮬 T+{st['elapsed_sec']:.0f}초, "
       f"강 내 파티클 {st['in_river_count']}개)")

    # [2] 파티클이 퍼질 때까지 대기
    m = wait_for_spread(a.top, a.wait)
    if m is None:
        pr("  [X] 미션을 만들 수 없습니다. 서버를 재시작하세요.")
        sys.exit(1)

    # [3] 기체 연결
    pr("")
    pr(f"  [3] 기체 연결 중...  {'(DRY-RUN)' if a.dry else '(실패 시 약 25초 소요)'}")
    conn, err = call("POST", "/drone/connect", {"dry_run": a.dry})
    if conn is None:
        pr(f"      [X] 연결 실패: {err}")
        pr("          Tello 전원과 노트북 WiFi(TELLO-XXXXXX) 를 확인하세요.")
        sys.exit(1)
    pr(f"      연결됨. 배터리 {conn['battery']}%")

    # [4] 미션 미리보기
    ok = show_mission(m)
    if not ok:
        pr("")
        pr("  [X] 지도 밖 waypoint 가 있어 비행할 수 없습니다.")
        sys.exit(1)

    # [5] 확인
    if not a.yes:
        pr("")
        if not a.dry:
            pr("  드론 주변에 사람이 없는지, 기수가 지도 북쪽을 보는지 확인하세요.")
        try:
            ans = input("  [5] 비행하시겠습니까? (y/N) ").strip().lower()
        except KeyboardInterrupt:
            pr("\n      취소됨")
            sys.exit(0)
        if ans != "y":
            pr("      취소됨")
            sys.exit(0)

    # [6] 실행
    res, err = call("POST", "/drone/start",
                    {"top_n": a.top, "speed": a.speed, "hover_sec": a.hover})
    if res is None:
        pr(f"  [X] 미션 시작 실패: {err}")
        sys.exit(1)

    try:
        final = monitor()
    except KeyboardInterrupt:
        pr("")
        pr("  [!] 중단 요청 - 착륙 명령 전송")
        call("POST", "/drone/land")
        final = monitor()

    # [7] 결과
    label = {
        "completed":  "정상 완료 - 모든 지점 순회 후 원점 복귀",
        "aborted":    "사용자 중단",
        "error":      "실패",
        "incomplete": "미완 - 일부 지점 미방문",
    }
    pr("")
    pr("=" * 64)
    pr(f"  결과: {final['result']}  -  {label.get(final['result'], '')}")
    pr(f"  실행한 레그: {final['leg_done']}/{final['leg_total']}")
    if final["error"]:
        pr(f"  오류: {final['error']}")
    if final["result"] == "completed" and not a.dry:
        pr("")
        pr("  * 착륙 지점이 원점 표시에서 얼마나 벗어났는지 재두면")
        pr("    보고서에 쓸 정량 데이터가 됩니다.")
    pr("=" * 64)


if __name__ == "__main__":
    main()
