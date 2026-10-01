"""드론 제어 CLI.

PowerShell 에서 curl 따옴표 이스케이프로 씨름하지 않기 위한 래퍼.
(PowerShell 의 curl 은 Invoke-WebRequest 별칭이라 -X -H -d 를 못 받는다.)

사용법
    python drone_cli.py status
    python drone_cli.py mission [--top 3]
    python drone_cli.py connect            # DRY-RUN (기본, 안전)
    python drone_cli.py connect --real     # 실기체
    python drone_cli.py start [--top 3] [--speed 30] [--hover 3]
    python drone_cli.py land               # 비상 착륙
    python drone_cli.py watch              # 상태 실시간 감시 (Ctrl+C 로 종료)
"""

import argparse
import sys
import time

import requests

BASE = "http://localhost:8000"


def _pr(*a):
    """콘솔 인코딩(cp949)에서 죽지 않는 print."""
    line = " ".join(str(x) for x in a)
    try:
        print(line)
    except Exception:
        enc = sys.stdout.encoding or "utf-8"
        print(line.encode(enc, "replace").decode(enc, "replace"))


def _call(method, path, body=None, timeout=60):
    url = BASE + path
    try:
        r = requests.request(method, url, json=body, timeout=timeout)
    except requests.exceptions.ConnectionError:
        _pr(f"[!] 서버에 연결할 수 없습니다: {url}")
        _pr("    python hanriver.py 가 실행 중인지 확인하세요.")
        sys.exit(1)

    try:
        data = r.json()
    except Exception:
        _pr(f"[!] HTTP {r.status_code} (JSON 아님): {r.text[:200]}")
        sys.exit(1)

    if r.status_code >= 400:
        _pr(f"[!] HTTP {r.status_code}: {data.get('detail', data)}")
        sys.exit(1)
    return data


def show_status(st):
    _pr(f"  모드      : {'DRY-RUN (기체 없음)' if st['dry_run'] else '실기체'}")
    _pr(f"  연결      : {st['connected']}")
    _pr(f"  단계      : {st['phase']}")
    _pr(f"  배터리    : {st['battery']}%" if st["battery"] is not None else "  배터리    : -")
    _pr(f"  진행      : leg {st['leg_done']}/{st['leg_total']}  (실행중={st['running']})")
    _pr(f"  지령 위치 : x={st['cur_x_cm']:+.1f}cm  y={st['cur_y_cm']:+.1f}cm")
    label = {
        "completed":  "정상 완료 (모든 레그 실행 + 착륙)",
        "aborted":    "사용자 중단 (land 요청)",
        "error":      "실패 (예외 발생)",
        "incomplete": "미완 (레그 일부 미실행)",
    }
    if st.get("result"):
        _pr(f"  종료 사유 : {st['result']} - {label.get(st['result'], '')}")
    if st["error"]:
        _pr(f"  오류      : {st['error']}")


def show_mission(m):
    _pr(f"  축척 1:{m['scale']}   지도 {m['map']['width_m']}m x {m['map']['height_m']}m")
    _pr("  ── 목표 지점 ──")
    for p in m["points"]:
        flag = "" if p["in_bounds"] else "   << 지도 밖!"
        _pr(f"    WP{p['rank']}  prob={p['prob']:6.2f}%  "
            f"실제 E{p['east_real_m']:+8.1f}m N{p['north_real_m']:+8.1f}m  "
            f"Tello x={p['tello_x_cm']:+7.1f} y={p['tello_y_cm']:+7.1f} cm{flag}")
    _pr("  ── 비행 레그 ──")
    for l in m["legs"]:
        dest = "원점복귀" if l["to_rank"] == 0 else f"WP{l['to_rank']}"
        tag = "SKIP" if l["skip"] else " GO "
        _pr(f"    [{tag}] leg{l['seq']} -> {dest:8s} "
            f"go({l['dx_cm']:+7.1f}, {l['dy_cm']:+7.1f})  {l['dist_cm']:6.1f}cm  {l['reason']}")
    if m["warnings"]:
        _pr("  ── 경고 ──")
        for w in m["warnings"]:
            _pr(f"    ! {w}")


def main():
    ap = argparse.ArgumentParser(description="Tello 드론 제어 CLI")
    sub = ap.add_subparsers(dest="cmd", required=True)

    sub.add_parser("status", help="현재 드론 상태")
    sub.add_parser("land", help="즉시 착륙 (비상 정지)")

    p = sub.add_parser("mission", help="미션 좌표 변환 결과만 확인 (드론 불필요)")
    p.add_argument("--top", type=int, default=3)

    p = sub.add_parser("connect", help="기체 연결")
    p.add_argument("--real", action="store_true", help="실기체에 연결 (기본은 DRY-RUN)")

    p = sub.add_parser("start", help="미션 실행")
    p.add_argument("--top",   type=int,   default=3, help="목표 지점 수 (1~10)")
    p.add_argument("--speed", type=int,   default=30, help="비행 속도 cm/s (10~100)")
    p.add_argument("--hover", type=float, default=3.0, help="지점별 정지 시간 초")

    sub.add_parser("watch", help="상태 실시간 감시")

    a = ap.parse_args()

    if a.cmd == "status":
        _pr("[상태]")
        show_status(_call("GET", "/drone/status"))

    elif a.cmd == "mission":
        _pr(f"[미션 미리보기] top_n={a.top}")
        show_mission(_call("GET", f"/mission?top_n={a.top}"))

    elif a.cmd == "connect":
        mode = "실기체" if a.real else "DRY-RUN"
        _pr(f"[연결] {mode} ... (실기체는 실패 시 약 25초 소요)")
        show_status(_call("POST", "/drone/connect", {"dry_run": not a.real}))

    elif a.cmd == "start":
        _pr(f"[미션 시작] top={a.top} speed={a.speed}cm/s hover={a.hover}s")
        d = _call("POST", "/drone/start",
                  {"top_n": a.top, "speed": a.speed, "hover_sec": a.hover})
        show_mission(d["mission"])
        _pr("")
        _pr("  실행 중. 중단하려면:  python drone_cli.py land")

    elif a.cmd == "land":
        _pr("[착륙 요청]")
        show_status(_call("POST", "/drone/land"))

    elif a.cmd == "watch":
        _pr("[감시] Ctrl+C 로 종료")
        try:
            while True:
                st = _call("GET", "/drone/status", timeout=10)
                _pr(f"  {time.strftime('%H:%M:%S')}  {st['phase']:8s} "
                    f"leg {st['leg_done']}/{st['leg_total']}  "
                    f"x={st['cur_x_cm']:+7.1f} y={st['cur_y_cm']:+7.1f}  "
                    f"batt={st['battery']}")
                time.sleep(1.0)
        except KeyboardInterrupt:
            _pr("  종료")


if __name__ == "__main__":
    main()
