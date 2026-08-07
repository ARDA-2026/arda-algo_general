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

import sys
import threading
import time

MIN_BATTERY   = 30     # % - 이보다 낮으면 이륙 거부
DEFAULT_SPEED = 30     # cm/s - go 명령 속도 (10~100)
HOVER_SEC     = 3.0    # 각 waypoint 도착 후 정지 시간
GO_MIN_CM     = 20
GO_MAX_CM     = 500


class TelloDriver:
    def __init__(self):
        self._lock    = threading.Lock()
        self._tello   = None
        self._thread  = None
        self._abort   = threading.Event()

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
            self._tello.connect()                    # "command" 전송 후 ok 대기
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
        self.leg_total = sum(1 for l in mission["legs"] if not l["skip"])
        self.leg_done  = 0
        self.cur_x = self.cur_y = 0.0
        self.error  = None
        self.result = None
        self._abort.clear()

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
                self._tello.takeoff()
            else:
                time.sleep(1.0)

            self.phase = "flying"
            for leg in self.mission["legs"]:
                if self._abort.is_set():
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
                    self._tello.go_xyz_speed(dx, dy, 0, speed)
                else:
                    time.sleep(max(0.5, leg["dist_cm"] / speed))

                self.cur_x += dx
                self.cur_y += dy
                self.leg_done += 1

                if self._abort.is_set():
                    break
                time.sleep(hover_sec)

        except Exception as e:
            failed = True
            self.error = f"{type(e).__name__}: {e}"
            self._ev(f"오류 - {self.error}")

        finally:
            # 무슨 일이 있어도 착륙시킨다
            self.phase = "landing"
            self._ev("착륙")
            try:
                if not self.dry_run:
                    self._tello.land()
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

    # ── 비상 착륙 ───────────────────────────────────────────
    def land_now(self):
        self._abort.set()
        self._ev("착륙 요청 (abort)")
        if self._busy():
            return self.status()          # 실행 스레드의 finally 가 착륙시킨다
        try:
            if not self.dry_run and self._tello is not None:
                self._tello.land()
            self.phase = "ready"
            self._ev("착륙 완료")
        except Exception as e:
            self.error = f"{type(e).__name__}: {e}"
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
            "result":      self.result,      # completed|aborted|error|incomplete
            "error":       self.error,
            "events":      list(self.events[-15:]),
        }


driver = TelloDriver()
