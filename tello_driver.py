"""Tello 미션 실행기.

tello_mission.build_mission() 이 만든 legs 를 실제 기체에 흘려보낸다.

────────────────────────────────────────────────────────────
안전 원칙 (실기체를 다루므로 전부 지킬 것)
────────────────────────────────────────────────────────────
  1. 이륙 전 배터리 확인. MIN_BATTERY 미만이면 거부.
  2. 어떤 예외가 나도 finally 에서 반드시 land() 를 시도한다.
  3. abort 플래그로 언제든 중단 가능 (POST /drone/land).
  4. 지도 밖 waypoint 가 하나라도 있으면 미션을 거부한다.
  5. DRY_RUN 모드에서는 기체에 연결하지 않고 명령만 기록한다.

────────────────────────────────────────────────────────────
좌표 추적
────────────────────────────────────────────────────────────
  Tello 는 절대 위치를 모른다 (미션패드 없는 일반 기체).
  따라서 "지령 위치"를 소프트웨어가 누적 추적한다.
  실제 위치는 드리프트로 조금씩 어긋나지만, 우리 계산 오차는 쌓이지 않는다.
"""

import math
import sys
import threading
import time

MIN_BATTERY   = 30     # % - 이보다 낮으면 이륙 거부
DEFAULT_SPEED = 30     # cm/s - go 명령 속도 (10~100)
HOVER_SEC     = 3.0    # 각 waypoint 도착 후 정지 시간
GO_MIN_CM     = 20
GO_MAX_CM     = 500

# ── 수동 모드 ──
# 지점을 골라 하나씩 이동시키는 모드. 이동이 끝나도 착륙하지 않고 떠 있는다.
# 떠 있는 채로 방치하면 배터리가 다해 추락하므로 유휴 시간 상한을 둔다.
IDLE_LAND_SEC = 90.0

# ── 추적 모드 ──
# 확률 1순위 좌표를 계속 쫓아간다. 목표는 표류를 따라 서쪽으로 흘러가는데
# Tello 최소 이동이 20cm 라 매번 갱신할 수 없다. 임계값을 넘을 때만 홉한다.
TRACK_THRESHOLD_CM = 20.0   # 이만큼 벌어져야 이동 명령 (Tello 최소 이동과 동일)
TRACK_POLL_SEC     = 2.0    # 목표 재조회 주기
TRACK_MAX_SEC      = 180.0  # 추적 최대 시간
TRACK_MAX_HOPS     = 60     # 폭주 방지 상한

# Tello 는 15초 동안 아무 명령도 못 받으면 스스로 착륙한다 (SDK 2.0 사양).
# 추적 모드는 목표가 20cm 이상 움직여야 go 를 보내므로 표류가 느리면
# 30초 넘게 아무것도 안 보내는 구간이 생긴다. 그러면 기체가 착륙해버리고
# 다음 go 는 "error Motor stop" 으로 실패한다 — 실제로 겪은 증상이다.
#
# djitellopy 의 get_battery()/get_height() 는 상태 스트림(UDP 8890)을 읽을
# 뿐 패킷을 보내지 않는다. 즉 배터리를 계속 조회해도 타이머는 안 풀린다.
KEEPALIVE_SEC = 5.0

# ── 대기 중 제자리 회전 ──
# 목표가 아직 없거나 임계값만큼 안 벌어져서 이동할 게 없을 때, 가만히 떠
# 있는 대신 한 바퀴 돈다. 수색 중이라는 게 눈에 보이고, 회전도 명령이므로
# keepalive 를 겸한다.
#
# 반드시 360도씩 도는 이유:
#   go 는 기체 좌표계(FLU) 명령이다. 기수가 90도 틀어진 채 go(100,0) 을
#   보내면 북쪽이 아니라 동쪽으로 간다. 지도 <-> 기체 좌표 변환이 통째로
#   무너지고, 기체는 절대 방위를 모르니 스스로 복구도 못 한다.
#   한 바퀴면 명목상 기수가 제자리로 돌아오므로 보정 자체가 필요 없다.
#   (cw 가 yaw 를 늘리는지 줄이는지는 SDK 에도 djitellopy 에도 없다.
#    부호를 모르는 채로 "되돌리는" 보정을 넣으면 틀렸을 때 오차가 배가 된다.)
SPIN_TURN_DEG   = 360
YAW_DRIFT_LIMIT = 25    # 누적 기수 오차가 이만큼 넘으면 회전을 그만둔다



class TelloDriver:
    def __init__(self):
        self._lock    = threading.Lock()
        self._tello   = None
        self._thread  = None
        self._abort   = threading.Event()
        # 자율비행 -> 수동 전환 요청. _abort 와 달리 착륙시키지 않고
        # 실행 스레드만 빠져나오게 한다 (기체는 그 자리에 떠 있는다).
        self._handover = threading.Event()
        # 기체로 나가는 패킷을 한 줄로 세운다. keepalive 는 비행 스레드가 아닌
        # _idle_watch 에서도 나가므로, 락이 없으면 djitellopy 의 기체당 응답
        # 큐에서 두 명령의 응답이 뒤섞인다.
        self._tx      = threading.Lock()
        self._last_tx = 0.0         # 마지막으로 기체에 뭔가 보낸 시각
        # 대기 중 제자리 회전
        self.spins     = 0          # 이번 비행에서 돈 바퀴 수
        self.yaw_drift = 0          # 한 바퀴마다 남는 기수 오차의 누적 (도)
        self._spin_ok  = True       # 오차가 커지면 스스로 끈다

        self.dry_run   = True
        self.connected = False
        self.phase     = "idle"     # idle|connecting|ready|takeoff|flying|landing|error
        self.battery   = None
        self.cur_x     = 0.0        # 지령 위치 (cm, 원점 기준)
        self.cur_y     = 0.0
        self.leg_done  = 0
        self.leg_total = 0
        self.mission   = None
        self.events    = []         # 최근 로그
        self.error     = None
        # 직전 미션의 종료 사유: None|completed|aborted|error|incomplete
        self.result    = None
        # 수동 모드용. 이동이 끝나도 착륙하지 않고 떠 있는 상태를 추적한다.
        self.airborne  = False
        self.mode      = None       # auto(자동 순회) | manual(수동 이동)
        self._last_cmd = 0.0        # 유휴 자동착륙 판단용
        # 지령 위치의 자취. 수동 모드는 경로가 미리 정해지지 않으므로
        # 화면에 "예정 경로" 대신 "실제 지나간 경로"를 그려야 한다.
        self.path      = [(0.0, 0.0)]

        # 수동 모드에서 떠 있는 채로 방치되는 걸 막는 감시 스레드
        threading.Thread(target=self._idle_watch, daemon=True).start()

    # ── 내부 유틸 ───────────────────────────────────────────
    def _ev(self, msg):
        """이벤트 기록. 콘솔 인코딩 때문에 절대 예외를 던지지 않는다.

        Windows 기본 콘솔은 cp949 라 '-' 나 '->' 같은 문자에서
        UnicodeEncodeError 가 난다. 로그 한 줄 때문에 미션이 죽으면 안 된다.
        """
        with self._lock:
            self.events.append(f"[{time.strftime('%H:%M:%S')}] {msg}")
            del self.events[:-40]
        line = f"[TELLO] {msg}"
        try:
            print(line)
        except Exception:
            enc = sys.stdout.encoding or "utf-8"
            try:
                print(line.encode(enc, "replace").decode(enc, "replace"))
            except Exception:
                pass

    def _busy(self):
        return self._thread is not None and self._thread.is_alive()

    def _stop_requested(self):
        """실행 루프를 빠져나가야 하는가. 착륙 여부는 finally 가 정한다."""
        return self._abort.is_set() or self._handover.is_set()

    def _mark_path(self):
        """지령 위치를 자취에 남긴다. 화면에 실제 경로를 그리는 근거."""
        self.path.append((round(self.cur_x, 1), round(self.cur_y, 1)))
        del self.path[:-60]

    def clear_path(self):
        """자취만 지운다. 비행 상태나 지령 위치는 건드리지 않는다.

        현재 위치를 새 시작점으로 삼아야 다음 이동이 허공에서 이어지지 않는다.
        """
        self.path = [(round(self.cur_x, 1), round(self.cur_y, 1))]
        self._ev("자취 지움")
        return self.status()

    # ── 기체 통신 ───────────────────────────────────────────
    def _send(self, fn, *a, **kw):
        """기체로 나가는 명령은 전부 이걸 거친다. 직렬화 + 송신 시각 기록.

        시각은 호출이 끝난 뒤에 찍는다. go 는 기체가 실제로 다 움직인 뒤에
        응답하므로, 시작 시각으로 재면 비행 시간만큼 타이머를 잘못 앞당긴다.
        """
        with self._tx:
            try:
                return fn(*a, **kw)
            finally:
                self._last_tx = time.time()

    def _keepalive(self):
        """15초 자동 착륙을 막는다. 마지막 송신 후 KEEPALIVE_SEC 지났을 때만.

        djitellopy 의 send_keepalive() 는 쓰지 않는다. 그건 send_control_command
        라서 'ok' 를 못 받으면 7초 타임아웃 × 3회 = 21초를 블로킹한 뒤 예외를
        던진다 (tello.py:474, RESPONSE_TIMEOUT=7, RETRY_COUNT=3). 그동안 _tx
        락을 쥐고 있으니 정작 필요한 go 가 21초 밀린다. 심장박동 하나가
        비행을 멈춰 세우면 안 된다.

        대신 응답을 안 기다리는 raw 전송을 쓴다. 5초마다 보내므로 두 번
        연속 유실돼도 15초 안에 다음 게 나간다.
        """
        if self.dry_run or self._tello is None or not self.airborne:
            return
        if time.time() - self._last_tx < KEEPALIVE_SEC:
            return
        try:
            self._send(self._tello.send_command_without_return, "keepalive")
        except Exception as e:
            # 실패해도 비행은 계속돼야 한다. 다음 주기에 다시 시도한다.
            self._ev(f"[keepalive] 실패 - {type(e).__name__}: {e}")

    def _yaw(self):
        """상태 스트림의 기수각(도). 패킷을 보내지 않는다."""
        try:
            return int(self._tello.get_yaw())
        except Exception:
            return None

    def _spin_once(self):
        """대기 중 한 바퀴. keepalive 주기에 맞춰 돈다.

        한 바퀴마다 실제 기수 오차를 재서 누적한다. YAW_DRIFT_LIMIT 를 넘으면
        스스로 회전을 끄고 제자리 유지로 내려온다 — 좌표계가 조용히 어긋난
        채로 계속 나는 것보다 낫다.
        """
        if time.time() - self._last_tx < KEEPALIVE_SEC:
            return
        if self.dry_run:
            self.spins += 1
            self._last_tx = time.time()
            self._ev(f"[대기] 제자리 1회전 (#{self.spins})")
            time.sleep(1.0)
            return

        y0 = self._yaw()
        try:
            self._send(self._tello.rotate_clockwise, SPIN_TURN_DEG)
        except Exception as e:
            self._spin_ok = False
            self._ev(f"[대기] 회전 실패 - 이후 제자리 유지만 한다: {type(e).__name__}: {e}")
            return
        self.spins += 1

        y1 = self._yaw()
        if y0 is None or y1 is None:
            self._ev(f"[대기] 제자리 1회전 (#{self.spins})")
            return
        drift = ((y1 - y0 + 180) % 360) - 180      # -180~180 으로 정규화
        self.yaw_drift += drift
        self._ev(f"[대기] 제자리 1회전 (#{self.spins}) "
                 f"기수오차 {drift:+d}도, 누적 {self.yaw_drift:+d}도")
        if abs(self.yaw_drift) > YAW_DRIFT_LIMIT:
            self._spin_ok = False
            self._ev(f"[대기] 누적 기수오차 {self.yaw_drift:+d}도 - 회전 중단 "
                     f"(이동 방향이 어긋나는 것을 막는다)")

    def _hold(self, sec, spin=False):
        """제자리 대기. 그냥 sleep 하면 기체가 15초 뒤 착륙한다.

        spin=True 면 가만히 있는 대신 한 바퀴씩 돈다 (수색 대기 표현).
        """
        end = time.time() + sec
        while not self._stop_requested():
            if spin and self._spin_ok and self.airborne:
                self._spin_once()
            else:
                self._keepalive()
            left = end - time.time()
            if left <= 0:
                break
            time.sleep(min(0.5, left))

    # ── 연결 ────────────────────────────────────────────────
    def connect(self, dry_run=True):
        if self._busy():
            raise RuntimeError("미션 실행 중에는 연결을 바꿀 수 없습니다")

        self.dry_run = dry_run
        self.error   = None

        if dry_run:
            self.connected = True
            self.battery   = 100
            self.phase     = "ready"
            self._ev("DRY-RUN 모드로 연결 (실제 기체 없음)")
            return self.status()

        self.phase = "connecting"
        try:
            from djitellopy import Tello

            # 이전 인스턴스를 반드시 먼저 정리한다.
            # djitellopy 는 Tello() 생성 시 전역 drones[host] 에 등록하고
            # __del__ -> end() 에서 그 항목을 삭제한다 (tello.py:127, 1024).
            # 정리 없이 self._tello = Tello() 로 재대입하면
            #   ① 새 인스턴스가 drones 등록
            #   ② 옛 인스턴스 참조 소멸 -> __del__ -> 방금 등록한 항목 삭제
            # 순서가 되어, 응답 수신 스레드가 "address not in drones" 로
            # 모든 응답을 버린다. 즉 두 번째 연결부터 무조건 타임아웃난다.
            old, self._tello = self._tello, None
            if old is not None:
                try:
                    old.end()
                except Exception:
                    pass
                del old            # __del__ 을 여기서 끝내고 새 인스턴스를 만든다

            self._tello = Tello()
            self._send(self._tello.connect)                    # "command" 전송 후 ok 대기
            self.battery   = self._tello.get_battery()
            self.connected = True
            self.phase     = "ready"
            self._ev(f"기체 연결됨. 배터리 {self.battery}%")
        except Exception as e:
            self.connected = False
            self.phase     = "error"
            self.error     = f"{type(e).__name__}: {e}"
            self._ev(f"연결 실패 - {self.error}")
            raise
        return self.status()

    # ── 미션 시작 ───────────────────────────────────────────
    def start(self, mission, speed=DEFAULT_SPEED, hover_sec=HOVER_SEC):
        if not self.connected:
            raise RuntimeError("먼저 /drone/connect 로 연결하세요")
        if self._busy():
            raise RuntimeError("이미 미션이 실행 중입니다")

        # 안전 검사 ①: 지도 밖 waypoint
        out = [p["rank"] for p in mission["points"] if not p["in_bounds"]]
        if out:
            raise RuntimeError(f"지도 밖 waypoint 가 있어 거부합니다: WP{out}")

        # 안전 검사 ②: 배터리
        if not self.dry_run:
            self.battery = self._tello.get_battery()
        if self.battery is not None and self.battery < MIN_BATTERY:
            raise RuntimeError(f"배터리 부족 {self.battery}% (최소 {MIN_BATTERY}%)")

        # 안전 검사 ③: 레그 범위
        for leg in mission["legs"]:
            if leg["skip"]:
                continue
            if max(abs(leg["dx_cm"]), abs(leg["dy_cm"])) > GO_MAX_CM:
                raise RuntimeError(f"leg{leg['seq']} 이동량이 {GO_MAX_CM}cm 초과")

        self.mission   = mission
        self.mode      = "auto"
        self.leg_total = sum(1 for l in mission["legs"] if not l["skip"])
        self.leg_done  = 0
        self.cur_x = self.cur_y = 0.0
        self.path   = [(0.0, 0.0)]
        self.error  = None
        self.result = None
        self._abort.clear()
        self._handover.clear()

        self._thread = threading.Thread(
            target=self._run, args=(speed, hover_sec), daemon=True)
        self._thread.start()
        self._ev(f"미션 시작 - 실행할 레그 {self.leg_total}개, 속도 {speed}cm/s")
        return self.status()

    # ── 미션 본체 (백그라운드 스레드) ───────────────────────
    def _run(self, speed, hover_sec):
        failed = False
        try:
            self.phase = "takeoff"
            self._ev("이륙")
            if not self.dry_run:
                self._send(self._tello.takeoff)
            else:
                time.sleep(1.0)

            self.phase = "flying"
            for leg in self.mission["legs"]:
                if self._stop_requested():
                    self._ev("중단 요청 - 남은 레그 취소")
                    break

                if leg["skip"]:
                    self._ev(f"leg{leg['seq']} 건너뜀 ({leg['reason']})")
                    continue

                dx = int(round(leg["dx_cm"]))
                dy = int(round(leg["dy_cm"]))
                dest = "원점" if leg["to_rank"] == 0 else f"WP{leg['to_rank']}"
                self._ev(f"leg{leg['seq']} -> {dest}  go({dx}, {dy}, 0, {speed})")

                if not self.dry_run:
                    self._send(self._tello.go_xyz_speed, dx, dy, 0, speed)
                else:
                    time.sleep(max(0.5, leg["dist_cm"] / speed))

                self.cur_x += dx
                self.cur_y += dy
                self._mark_path()
                self.leg_done += 1

                if self._stop_requested():
                    break
                self._hold(hover_sec)      # hover_sec 는 최대 30초 — 그냥 자면 착륙한다

        except Exception as e:
            failed = True
            self.error = f"{type(e).__name__}: {e}"
            self._ev(f"오류 - {self.error}")

        finally:
            # 수동 전환 요청이면 착륙하지 않고 그 자리에 떠 있는 채로 넘긴다.
            # 이게 _abort 와 다른 점이다 - _abort 는 무조건 내린다.
            if self._handover.is_set():
                self._handover.clear()
                self.mode      = "manual"
                self.result    = "handover"
                self.phase     = "hovering"
                self._last_cmd = time.time()   # 유휴 자동착륙 타이머 시작
                self._ev(f"[전환] 수동 조종으로 넘김 - 제자리 유지 "
                         f"(유휴 {IDLE_LAND_SEC:.0f}초 후 자동 착륙)")
                return

            # 무슨 일이 있어도 착륙시킨다
            self.phase = "landing"
            self._ev("착륙")
            try:
                if not self.dry_run:
                    self._send(self._tello.land)
                else:
                    time.sleep(1.0)
            except Exception as e:
                failed = True
                self.error = f"착륙 실패: {type(e).__name__}: {e}"
                self._ev(self.error)

            # 종료 사유를 명확히 남긴다.
            # phase 만으로는 정상완료/중단/실패를 구분할 수 없다.
            if failed:
                self.result = "error"
                self.phase  = "error"
            elif self._abort.is_set():
                self.result = "aborted"
                self.phase  = "ready"
            elif self.leg_done == self.leg_total:
                self.result = "completed"
                self.phase  = "ready"
            else:
                self.result = "incomplete"
                self.phase  = "ready"
            self._ev(f"미션 종료 ({self.result}) - leg {self.leg_done}/{self.leg_total}")

    # ── 수동 모드 ───────────────────────────────────────────
    def _check_ready(self):
        if not self.connected:
            raise RuntimeError("먼저 /drone/connect 로 연결하세요")
        if self._busy():
            raise RuntimeError("이미 명령을 수행 중입니다")

    def _check_battery(self):
        if not self.dry_run:
            try:
                self.battery = self._tello.get_battery()
            except Exception:
                pass
        if self.battery is not None and self.battery < MIN_BATTERY:
            raise RuntimeError(f"배터리 부족 {self.battery}% (최소 {MIN_BATTERY}%)")

    def manual_takeoff(self):
        """이륙만 하고 떠 있는다. 착륙은 명시적으로 해야 한다."""
        self._check_ready()
        if self.airborne:
            raise RuntimeError("이미 비행 중입니다")
        self._check_battery()

        self.mode   = "manual"
        self.result = None
        self.error  = None
        self.leg_done = self.leg_total = 0
        self.cur_x = self.cur_y = 0.0
        self.path  = [(0.0, 0.0)]
        self._abort.clear()
        self._handover.clear()
        self._thread = threading.Thread(target=self._run_takeoff, daemon=True)
        self._thread.start()
        self._ev(f"[수동] 이륙 (유휴 {IDLE_LAND_SEC:.0f}초 후 자동 착륙)")
        return self.status()

    def _run_takeoff(self):
        try:
            self.phase = "takeoff"
            if not self.dry_run:
                self._send(self._tello.takeoff)
            else:
                time.sleep(1.0)
            self.airborne  = True
            self._last_cmd = time.time()
            self.phase     = "hovering"
            self._ev("[수동] 이륙 완료 - 대기 중")
        except Exception as e:
            self.error = f"{type(e).__name__}: {e}"
            self.phase = "error"
            self._ev(f"[수동] 이륙 실패 - {self.error}")
            self._force_land()

    def manual_goto(self, x_cm, y_cm, speed=DEFAULT_SPEED, label=""):
        """지령 위치에서 목표(x_cm, y_cm)로 한 번 이동한다. 착륙하지 않는다."""
        self._check_ready()
        if not self.airborne:
            raise RuntimeError("먼저 이륙하세요")
        self._check_battery()

        dx, dy = x_cm - self.cur_x, y_cm - self.cur_y
        if max(abs(dx), abs(dy)) < GO_MIN_CM:
            raise RuntimeError(
                f"이동량이 {math.hypot(dx, dy):.0f}cm 로 Tello 최소 {GO_MIN_CM}cm 미만입니다")
        if max(abs(dx), abs(dy)) > GO_MAX_CM:
            raise RuntimeError(
                f"이동량이 {math.hypot(dx, dy):.0f}cm 로 최대 {GO_MAX_CM}cm 를 넘습니다")

        self._abort.clear()
        self._handover.clear()
        self._thread = threading.Thread(
            target=self._run_goto, args=(dx, dy, speed, label), daemon=True)
        self._thread.start()
        return self.status()

    def _run_goto(self, dx, dy, speed, label):
        try:
            self.phase = "flying"
            idx, idy = int(round(dx)), int(round(dy))
            self._ev(f"[수동] {label or '이동'}  go({idx}, {idy}, 0, {speed})")
            if not self.dry_run:
                self._send(self._tello.go_xyz_speed, idx, idy, 0, speed)
            else:
                time.sleep(max(0.4, math.hypot(idx, idy) / speed))
            self.cur_x += idx
            self.cur_y += idy
            self._mark_path()
            self.leg_done += 1
            self.leg_total = self.leg_done
            self._last_cmd = time.time()
            self.phase = "hovering"
            self._ev(f"[수동] 도착 - 지령 ({self.cur_x:+.0f}, {self.cur_y:+.0f})cm")
        except Exception as e:
            self.error = f"{type(e).__name__}: {e}"
            self._ev(f"[수동] 이동 실패 - {self.error}")
            self._force_land()

    # ── 추적 모드 ───────────────────────────────────────────
    def start_tracking(self, target_fn, bounds_fn, *, speed=DEFAULT_SPEED,
                       threshold_cm=TRACK_THRESHOLD_CM, max_sec=TRACK_MAX_SEC):
        """확률 1순위 좌표를 계속 따라간다. 이륙부터 착륙까지 자동.

        target_fn()      -> (x_cm, y_cm) 현재 목표의 기체 좌표, 없으면 None
        bounds_fn(x, y)  -> bool 해당 좌표가 지도 안인지
        """
        self._check_ready()
        if self.airborne:
            raise RuntimeError("이미 비행 중입니다")
        self._check_battery()
        # 목표가 아직 없어도 이륙한다. 탐지 확정 직후엔 파티클을 막 뿌린
        # 참이라 waypoint 가 없는데, 그때 기다리면 이륙이 늦어진다.
        # 떠서 제자리 대기하다가 1순위가 생기면 그때부터 따라간다.

        self.mission   = None
        self.mode      = "track"
        self.leg_done  = self.leg_total = 0
        self.cur_x = self.cur_y = 0.0
        self.path   = [(0.0, 0.0)]
        self.error  = None
        self.result = None
        # 회전 누적은 비행 단위다. 이전 비행의 기수 오차를 물려받으면
        # 새 비행이 시작하자마자 회전을 못 하게 된다.
        self.spins = self.yaw_drift = 0
        self._spin_ok = True
        self._abort.clear()
        self._handover.clear()

        self._thread = threading.Thread(
            target=self._run_track,
            args=(target_fn, bounds_fn, speed, threshold_cm, max_sec), daemon=True)
        self._thread.start()
        self._ev(f"[추적] 시작 - 임계 {threshold_cm:.0f}cm, 속도 {speed}cm/s, 최대 {max_sec:.0f}초")
        return self.status()

    def _run_track(self, target_fn, bounds_fn, speed, threshold_cm, max_sec):
        failed = False
        t0 = time.time()
        try:
            self.phase = "takeoff"
            self._ev("[추적] 이륙")
            if not self.dry_run:
                self._send(self._tello.takeoff)
            else:
                time.sleep(1.0)
            self.airborne = True

            self.phase = "flying"
            while not self._stop_requested():
                if time.time() - t0 > max_sec:
                    self._ev(f"[추적] 최대 시간 {max_sec:.0f}초 도달")
                    break
                if self.leg_done >= TRACK_MAX_HOPS:
                    self._ev(f"[추적] 이동 상한 {TRACK_MAX_HOPS}회 도달")
                    break

                # 실기체는 매 홉마다 배터리를 다시 본다.
                # 주의: get_battery() 는 상태 스트림을 읽을 뿐 패킷을 안 보낸다.
                # 15초 타이머는 이걸로 안 풀리므로 keepalive 가 따로 필요하다.
                if not self.dry_run:
                    try:
                        self.battery = self._tello.get_battery()
                    except Exception:
                        pass
                    if self.battery is not None and self.battery < MIN_BATTERY:
                        self._ev(f"[추적] 배터리 {self.battery}% - 중단")
                        break

                tgt = target_fn()
                if tgt is None:
                    if self.phase != "hovering":
                        self.phase = "hovering"
                        self._ev("[추적] 1순위 좌표 대기 중 - 제자리 유지")
                    self._hold(TRACK_POLL_SEC, spin=True)
                    continue
                if self.phase == "hovering":
                    self.phase = "flying"
                    self._ev("[추적] 1순위 좌표 확보 - 추종 시작")

                dx, dy = tgt[0] - self.cur_x, tgt[1] - self.cur_y
                if max(abs(dx), abs(dy)) < threshold_cm:
                    self._hold(TRACK_POLL_SEC, spin=True)
                    continue
                if not bounds_fn(self.cur_x + dx, self.cur_y + dy):
                    self._ev("[추적] 목표가 지도 밖 - 이동 보류")
                    self._hold(TRACK_POLL_SEC, spin=True)
                    continue

                # 한 번에 GO_MAX_CM 을 넘지 않도록 자른다
                scale = min(1.0, GO_MAX_CM / max(abs(dx), abs(dy)))
                idx, idy = int(round(dx * scale)), int(round(dy * scale))
                if max(abs(idx), abs(idy)) < GO_MIN_CM:
                    self._hold(TRACK_POLL_SEC, spin=True)
                    continue

                self.leg_done += 1
                self.leg_total = self.leg_done
                self._ev(f"[추적] 이동 #{self.leg_done}  go({idx}, {idy}, 0, {speed})")
                if not self.dry_run:
                    self._send(self._tello.go_xyz_speed, idx, idy, 0, speed)
                else:
                    time.sleep(max(0.4, math.hypot(idx, idy) / speed))

                self.cur_x += idx
                self.cur_y += idy
                self._mark_path()

        except Exception as e:
            failed = True
            self.error = f"{type(e).__name__}: {e}"
            self._ev(f"[추적] 오류 - {self.error}")
        finally:
            # 수동 전환 요청이면 착륙하지 않고 그 자리에 떠 있는 채로 넘긴다.
            # 이게 _abort 와 다른 점이다 - _abort 는 무조건 내린다.
            if self._handover.is_set():
                self._handover.clear()
                self.mode      = "manual"
                self.result    = "handover"
                self.phase     = "hovering"
                self._last_cmd = time.time()   # 유휴 자동착륙 타이머 시작
                self._ev(f"[전환] 수동 조종으로 넘김 - 제자리 유지 "
                         f"(유휴 {IDLE_LAND_SEC:.0f}초 후 자동 착륙)")
                return

            self.phase = "landing"
            self._ev("[추적] 착륙")
            try:
                if not self.dry_run:
                    self._send(self._tello.land)
                else:
                    time.sleep(1.0)
            except Exception as e:
                failed = True
                self.error = f"착륙 실패: {type(e).__name__}: {e}"
                self._ev(self.error)
            self.airborne = False
            if failed:
                self.result, self.phase = "error", "error"
            elif self._abort.is_set():
                self.result, self.phase = "aborted", "ready"
            else:
                self.result, self.phase = "completed", "ready"
            self._ev(f"[추적] 종료 ({self.result}) - 이동 {self.leg_done}회, "
                     f"{time.time() - t0:.0f}초")

    def _force_land(self):
        """오류 시 즉시 착륙시킨다."""
        try:
            if not self.dry_run and self._tello is not None:
                self._send(self._tello.land)
        except Exception:
            pass
        self.airborne = False
        self.phase    = "error"
        self.result   = "error"

    def _idle_watch(self):
        """수동 모드에서 떠 있는 채로 방치되면 자동 착륙시킨다.
        겸사겸사 keepalive 도 여기서 보낸다.

        유휴 판정은 반드시 mode == "manual" 일 때만 한다. 자동 모드(추적)도
        목표를 기다리는 동안 phase 를 "hovering" 으로 두는데, phase 만 보면
        추적이 이륙하자마자 여기에 걸려 착륙당한다. 게다가 추적은 _last_cmd
        를 쓰지 않아 0.0 인 채라, 유휴 시간이 유닉스 시각 그대로 계산돼
        무조건 상한을 넘긴다 - 실제로 "유휴 1787639559초" 로 겪었다.
        추적에는 자체 상한(TRACK_MAX_SEC)이 따로 있다.
        """
        while True:
            time.sleep(2.0)
            if not self.airborne:
                continue
            # 비행 스레드가 명령 중이면 건드리지 않는다. _tx 락이 있어 안전하지만
            # 굳이 go 응답을 기다리며 줄 서 있을 이유가 없다.
            if not self._busy():
                self._keepalive()
            if self.mode != "manual" or self.phase != "hovering":
                continue
            if not self._last_cmd:
                continue          # 아직 명령이 한 번도 없었으면 판단할 수 없다
            idle = time.time() - self._last_cmd
            if idle >= IDLE_LAND_SEC:
                self._ev(f"[수동] 유휴 {idle:.0f}초 - 자동 착륙")
                self.land_now()

    # ── 자율비행 -> 수동 전환 ───────────────────────────────
    def take_manual(self):
        """자율비행(자동 순회 / 자동 모드) 중에 수동 조종으로 넘겨받는다.

        착륙하지 않는다. 실행 스레드만 빠져나오고 기체는 그 자리에 뜬 채로
        남아, 이후 /drone/goto 로 지점을 하나씩 보낼 수 있다.

        즉시 끊지 못하는 경우가 있다. go 명령은 기체가 이동을 마치고 응답할
        때까지 블로킹하므로(500cm 를 30cm/s 로 가면 17초), 그 이동이 끝난
        직후에 전환된다. 그래서 요청만 걸어두고 바로 반환한다 - 화면은
        status 의 handover 플래그로 "전환 중" 을 보여주면 된다.
        """
        if not self.airborne:
            raise RuntimeError("비행 중이 아닙니다")
        if not self._busy():
            # 이미 스레드가 끝나 떠 있기만 한 상태 (수동 모드 유휴 등)
            self.mode      = "manual"
            self.phase     = "hovering"
            self._last_cmd = time.time()
            return self.status()
        if self._handover.is_set():
            return self.status()        # 이미 요청됨
        self._handover.set()
        self._ev("[전환] 수동 조종 요청 - 진행 중인 이동이 끝나면 넘깁니다")
        return self.status()

    # ── 비상 착륙 ───────────────────────────────────────────
    def land_now(self):
        # 착륙이 인계보다 우선이다. 인계 대기 중에 비상 착륙을 누르면
        # 그대로 내려야 하므로 요청을 지운다.
        self._handover.clear()
        self._abort.set()
        self._ev("착륙 요청 (abort)")
        if self._busy():
            # 자동 모드는 실행 스레드의 finally 가 착륙시킨다.
            # 수동 모드는 이동이 끝나도 떠 있으므로 여기서 직접 내린다.
            if self.mode != "manual":
                return self.status()
        try:
            if not self.dry_run and self._tello is not None:
                self._send(self._tello.land)
            self.airborne = False
            self.phase    = "ready"
            if self.mode == "manual":
                self.result = "completed"
            self._ev("착륙 완료")
        except Exception as e:
            self.airborne = False
            self.error = f"{type(e).__name__}: {e}"
            self.phase = "error"
            self._ev(f"착륙 실패 - {self.error}")
        return self.status()

    # ── 상태 ────────────────────────────────────────────────
    def status(self):
        h = None
        if self.connected and not self.dry_run and not self._busy():
            try:
                h = self._tello.get_height()
            except Exception:
                pass
        return {
            "dry_run":     self.dry_run,
            "connected":   self.connected,
            "phase":       self.phase,
            "battery":     self.battery,
            "height_cm":   h,
            "leg_done":    self.leg_done,
            "leg_total":   self.leg_total,
            "cur_x_cm":    round(self.cur_x, 1),
            "cur_y_cm":    round(self.cur_y, 1),
            "running":     self._busy(),
            "airborne":    self.airborne,    # 수동 모드에서 떠 있는 상태
            "mode":        self.mode,        # auto | manual | track
            # 수동 전환 요청이 걸려 있는가 (진행 중인 이동이 끝나면 넘어감)
            "handover":    self._handover.is_set(),
            "path":        list(self.path),
            # 대기 중 제자리 회전. yaw_drift 가 커지면 spin_ok 가 꺼진다
            "spins":       self.spins,
            "yaw_drift":   self.yaw_drift,
            "spin_ok":     self._spin_ok,
            "idle_left":   (round(max(0.0, IDLE_LAND_SEC - (time.time() - self._last_cmd)))
                            if (self.airborne and self.phase == "hovering") else None),
            "result":      self.result,      # completed|aborted|error|incomplete
            "error":       self.error,
            "events":      list(self.events[-15:]),
        }


driver = TelloDriver()
