"""
로봇 클라이언트 진입점
====================

**프로세스 1개 = 로봇 1대.** 한 프로세스에 여러 대를 넣지 않는다.
10대를 띄우려면 10번 실행한다.

    python -m robot_client.main --id AMR-001 --x 1.2 --y 6.0
    python -m robot_client.main --id AMR-002 --x 1.2 --y 4.0

FMS 도 뷰어도 이 프로세스를 직접 알지 못한다. 오직 브로커를 통해
state 를 읽고 order 를 보낼 뿐이다. 프로세스를 죽이면(kill/docker stop)
브로커가 LWT 로 CONNECTION_BROKEN 을 대신 발행한다 — 장애 주입 수단이다.

--map 을 주면 맵을 읽어 시작 위치가 주행 가능한 셀인지 확인한다.
FMS 는 목적지 좌표만 오더로 내려주고, 거기까지 장애물을 피해 가는 경로는
로봇이 이 맵으로 스스로 A* 계획을 짠다 (robot_client/planner.py).
--map 을 안 주면 경로계획도 충돌 감지도 꺼지고 노드 사이를 직선으로만 움직인다.

--gui 를 주면 로컬 제어판(robot_client/gui.py)이 뜬다. 정보 표시, 목적지 이동
명령, 강제 재위치까지 하나의 창에서 손으로 찔러볼 수 있다 — MQTT 를 안 타고
같은 프로세스 안에서 커맨드 큐로 바로 꽂아 넣는다 (사람이 조작하는 로컬
콘솔이지 FMS 가 아니다). Tk 의존성은 --gui 없이는 아예 import 되지 않으므로
docker-compose 로 N대를 헤드리스로 띄울 때는 영향이 없다.
"""

from __future__ import annotations

import argparse
import itertools
import logging
import queue
import random
import signal
import sys
import threading
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

from common.schemas import InstantAction, Order, OrderNode, Pose  # noqa: E402
from robot_client.clock import RealtimeClock, Ticker  # noqa: E402
from robot_client.comm import RobotComm  # noqa: E402
from robot_client.executor import OrderExecutor  # noqa: E402
from robot_client.sim.battery import BatterySim  # noqa: E402

log = logging.getLogger("robot")

TICK_PERIOD = 0.05      # 20 Hz 시뮬레이션
STATE_PERIOD = 0.2      # 200ms 주기 상태 보고 (CLAUDE.md 프로토콜)


def build_arg_parser() -> argparse.ArgumentParser:
    p = argparse.ArgumentParser(description="AMR 로봇 클라이언트 (프로세스 1개 = 로봇 1대)")
    p.add_argument("--id", required=True, help="로봇 식별자. 예: AMR-001")
    p.add_argument("--host", default="localhost", help="MQTT 브로커 호스트")
    p.add_argument("--port", type=int, default=1883, help="MQTT 브로커 포트")
    p.add_argument("--x", type=float, default=0.0, help="시작 x (m)")
    p.add_argument("--y", type=float, default=0.0, help="시작 y (m)")
    p.add_argument("--theta", type=float, default=0.0, help="시작 각도 (rad)")
    p.add_argument("--map", help="맵 메타데이터 JSON. 경로계획(A*) + 충돌 감지용 (선택)")
    p.add_argument("--robot-width", type=float, default=0.5, help="로봇 폭 (m). 경로계획 시 이보다 좁은 통로는 피한다")
    p.add_argument("--robot-length", type=float, default=0.7, help="로봇 길이 (m)")
    p.add_argument("--max-velocity", type=float, default=1.2, help="최대 속도 (m/s)")
    p.add_argument("--max-acceleration", type=float, default=0.6, help="최대 가속도 (m/s^2)")
    p.add_argument("--battery", type=float, default=100.0, help="시작 배터리 (%%)")
    p.add_argument("--seed", type=int, help="속도 노이즈용 시드. 실험 재현에 필요")
    p.add_argument(
        "--speed-noise", type=float, default=0.05,
        help="최대 속도에 줄 상대 노이즈 (0.05 = ±5%%). 로봇이 완벽 동기화되면 "
             "충돌 상황이 재현되지 않으므로 기본으로 켜 둔다",
    )
    p.add_argument("--log-level", default="INFO", help="DEBUG / INFO / WARNING")
    p.add_argument("--gui", action="store_true", help="로컬 제어판(Tk) 창을 띄운다")
    return p


_gui_order_seq = itertools.count(1)


def _dispatch_gui_command(executor: OrderExecutor, command: tuple) -> None:
    """GUI 스레드가 큐에 넣은 명령을 tick 루프 스레드에서 실제로 실행한다."""
    kind = command[0]
    if kind == "move":
        _, x, y = command
        order = Order(
            header_id=1, timestamp=0.0, robot_id=executor.robot_id,
            order_id=f"GUI-{next(_gui_order_seq)}", order_update_id=0,
            nodes=[OrderNode(node_id="N0", sequence_id=0, released=True,
                              position=Pose(x=x, y=y, theta=0.0))],
        )
        if not executor.accept_order(order):
            log.warning("[%s] GUI 이동 명령 거절 (다른 오더 실행 중이거나 상태 불일치)",
                        executor.robot_id)
    elif kind == "relocalize":
        _, x, y, theta = command
        executor.force_relocalize(x, y, theta)


def _robot_loop(
    executor: OrderExecutor, comm: RobotComm, clock: RealtimeClock, ticker: Ticker,
    command_queue: "queue.Queue",
) -> None:
    since_state = 0.0
    try:
        for dt in ticker:
            for message in comm.drain():
                if isinstance(message, Order):
                    executor.accept_order(message)
                elif isinstance(message, InstantAction):
                    executor.handle_instant(message)

            while True:
                try:
                    command = command_queue.get_nowait()
                except queue.Empty:
                    break
                _dispatch_gui_command(executor, command)

            executor.tick(dt)

            since_state += dt
            if since_state >= STATE_PERIOD:
                since_state = 0.0
                comm.publish_state(
                    executor.build_state(comm.next_header_id(), clock.now())
                )
    finally:
        log.info("[%s] 종료", executor.robot_id)
        comm.disconnect()


def main() -> int:
    args = build_arg_parser().parse_args()
    logging.basicConfig(
        level=getattr(logging, args.log_level.upper(), logging.INFO),
        format="%(asctime)s %(levelname)-7s %(message)s",
        datefmt="%H:%M:%S",
    )

    # 같은 시드 + 같은 로봇 id 면 항상 같은 노이즈가 나오게 한다 (실험 재현성)
    rng = random.Random(f"{args.seed}-{args.id}" if args.seed is not None else None)
    max_velocity = args.max_velocity * (1.0 + rng.uniform(-args.speed_noise, args.speed_noise))

    map_model = None
    if args.map:
        from common.map_model import MapModel

        map_model = MapModel.load(args.map)
        cell = map_model.cell_at(args.x, args.y)
        if not cell.drivable:
            log.error("[%s] 시작 위치 (%.2f, %.2f) 가 주행 불가 셀이다: %s",
                      args.id, args.x, args.y, cell.value)
            return 1
        log.info("[%s] 맵 확인 완료: 시작 위치 %s (경로계획 + 충돌 감지 켜짐)", args.id, cell.value)

    executor = OrderExecutor(
        robot_id=args.id,
        start_x=args.x, start_y=args.y, start_theta=args.theta,
        max_velocity=max_velocity,
        max_acceleration=args.max_acceleration,
        battery=BatterySim(charge=args.battery, max_velocity=max_velocity),
        map_model=map_model,
        robot_width=args.robot_width,
        robot_length=args.robot_length,
    )

    comm = RobotComm(robot_id=args.id, host=args.host, port=args.port)
    try:
        comm.connect()
    except OSError as e:
        log.error("[%s] 브로커 접속 실패 (%s:%d): %s — mosquitto 가 떠 있는지 확인",
                  args.id, args.host, args.port, e)
        return 1

    if not comm.wait_until_connected(timeout=5.0):
        log.error("[%s] 브로커 응답 없음", args.id)
        comm.disconnect()
        return 1

    clock = RealtimeClock()
    ticker = Ticker(clock, TICK_PERIOD)
    signal.signal(signal.SIGINT, lambda *_: ticker.stop())
    signal.signal(signal.SIGTERM, lambda *_: ticker.stop())

    log.info("[%s] 기동. 시작 pose=(%.2f, %.2f, %.2f) 최대속도=%.3f m/s",
             args.id, args.x, args.y, args.theta, max_velocity)

    command_queue: "queue.Queue" = queue.Queue()

    if args.gui:
        from robot_client.gui import launch_gui

        thread = threading.Thread(
            target=_robot_loop, args=(executor, comm, clock, ticker, command_queue),
            daemon=True,
        )
        thread.start()
        launch_gui(
            executor, command_queue,
            max_velocity=max_velocity, max_acceleration=args.max_acceleration,
            robot_width=args.robot_width, robot_length=args.robot_length,
            on_close=ticker.stop,
        )
        ticker.stop()
        thread.join(timeout=5.0)
    else:
        _robot_loop(executor, comm, clock, ticker, command_queue)
    return 0


if __name__ == "__main__":
    sys.exit(main())
