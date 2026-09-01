"""
플릿 모니터 (MQTT 관측자)
========================

브로커에서 로봇들의 state / connection 을 구독해 뷰어에 그린다.

    python tools/fleet_monitor.py maps/warehouse/warehouse.json

**순수 관측자다.** 로봇에게 아무것도 보내지 않고, 로봇을 시뮬레이션하지도 않는다.
화면에 보이는 모든 값은 로봇 프로세스가 스스로 보고한 것이다. 모니터를 꺼도
로봇들은 그대로 돌아가고, 로봇이 없으면 빈 맵만 보인다.

    [robot_client/main.py --id AMR-001]  --state-->  [broker]  --구독-->  [이 프로그램]
    [robot_client/main.py --id AMR-002]  --state-->     |
                  ...                                  +---------------> [FMS 서버]

FMS 가 떠 있으면 order 토픽도 함께 구독해 로봇이 받은 경로를 겹쳐 그린다.
FMS 가 없어도 로봇 pose 와 궤적은 그대로 보인다.

MQTT 콜백은 paho 네트워크 스레드에서 오고 matplotlib 은 메인 스레드에서만
그릴 수 있다. 그래서 콜백은 큐에 넣기만 하고, 타이머가 메인 스레드에서 꺼내 그린다.
"""

from __future__ import annotations

import argparse
import logging
import queue
import sys
import time
from pathlib import Path
from typing import Callable, Optional

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

import paho.mqtt.client as mqtt  # noqa: E402
from pydantic import ValidationError  # noqa: E402

from common.map_model import MapModel  # noqa: E402
from common.schemas import (  # noqa: E402
    TOPIC_CONNECTION_ALL,
    TOPIC_PREFIX,
    TOPIC_STATE_ALL,
    Connection,
    ConnectionState,
    Order,
    State,
)
from tools.map_viewer import MapViewer  # noqa: E402

log = logging.getLogger("monitor")

TOPIC_ORDER_ALL = f"{TOPIC_PREFIX}/+/order"
REFRESH_PERIOD_MS = 100


class FleetMonitor:
    """MQTT 를 구독해 MapViewer 에 밀어넣는다. 발행은 하지 않는다."""

    def __init__(self, viewer: MapViewer, host: str = "localhost", port: int = 1883,
                 watch_orders: bool = True, on_event: Optional[Callable[[str], None]] = None):
        self.viewer = viewer
        self.host = host
        self.port = port
        self.watch_orders = watch_orders
        # 화면 이벤트 로그용 훅 (선택). 접속 변화/오더 관측/새 오류 발생 시 한 줄씩 호출된다.
        # matplotlib 뷰어는 안 쓰고, tools/fleet_monitor_qt.py 가 이걸로 온스크린 로그를 채운다
        self.on_event = on_event
        self._inbox: "queue.Queue[object]" = queue.Queue()
        self._seen_orders: dict[str, str] = {}
        self._last_errors: dict[str, tuple] = {}

        self._client = mqtt.Client(
            mqtt.CallbackAPIVersion.VERSION2,
            client_id=f"fleet-monitor-{int(time.time())}",
            clean_session=True,
        )
        self._client.on_connect = self._on_connect
        self._client.on_message = self._on_message

    def start(self) -> None:
        self._client.connect(self.host, self.port, keepalive=30)
        self._client.loop_start()

    def stop(self) -> None:
        self._client.loop_stop()
        self._client.disconnect()

    # -- paho 콜백 (네트워크 스레드) -----------------------------------------

    def _on_connect(self, client, userdata, flags, reason_code, properties=None) -> None:
        if reason_code != 0:
            log.error("브로커 접속 실패: %s", reason_code)
            return
        topics = [(TOPIC_STATE_ALL, 0), (TOPIC_CONNECTION_ALL, 1)]
        if self.watch_orders:
            topics.append((TOPIC_ORDER_ALL, 1))
        client.subscribe(topics)
        log.info("구독 시작: %s", ", ".join(t for t, _ in topics))

    def _on_message(self, client, userdata, msg) -> None:
        try:
            payload = msg.payload.decode("utf-8")
        except UnicodeDecodeError:
            return
        try:
            if msg.topic.endswith("/state"):
                self._inbox.put(State.model_validate_json(payload))
            elif msg.topic.endswith("/connection"):
                self._inbox.put(Connection.model_validate_json(payload))
            elif msg.topic.endswith("/order"):
                self._inbox.put(Order.model_validate_json(payload))
        except ValidationError as e:
            log.warning("파싱 실패 (%s): %d건", msg.topic, e.error_count())

    # -- 메인 스레드에서 호출 ------------------------------------------------

    def pump(self) -> bool:
        """큐를 비우고 뷰어에 반영한다. 바뀐 게 있으면 True."""
        changed = False
        now = time.time()
        while True:
            try:
                message = self._inbox.get_nowait()
            except queue.Empty:
                break
            changed = True
            if isinstance(message, State):
                self.viewer.update_robot(
                    message.robot_id,
                    message.pose.x, message.pose.y, message.pose.theta,
                    velocity=message.velocity,
                    width=message.width,
                    length=message.length,
                    state=message.display_state,
                    battery=message.battery.charge,
                    order_id=message.order_id,
                    remaining_nodes=len(message.node_states),
                    errors=[e.error_type for e in message.errors],
                    planned_path=[(p.x, p.y) for p in message.local_path],
                    now=now,
                )
                errors = tuple(e.error_type for e in message.errors)
                if errors and errors != self._last_errors.get(message.robot_id):
                    self._emit_event(f"{message.robot_id} | ERROR {', '.join(errors)}")
                self._last_errors[message.robot_id] = errors
            elif isinstance(message, Connection):
                self.viewer.set_connection(
                    message.robot_id, message.connection_state.value, now=now
                )
                if message.connection_state != ConnectionState.ONLINE:
                    log.info("[%s] %s", message.robot_id, message.connection_state.value)
                    self._emit_event(f"{message.robot_id} | {message.connection_state.value}")
            elif isinstance(message, Order):
                self._seen_orders[message.robot_id] = message.order_id
                log.info("[%s] 오더 관측: %s update=%d, 노드 %d개",
                         message.robot_id, message.order_id,
                         message.order_update_id, len(message.nodes))
                self._emit_event(f"{message.robot_id} | order {message.order_id}")
        return changed

    def _emit_event(self, text: str) -> None:
        if self.on_event is not None:
            self.on_event(text)


def main() -> int:
    parser = argparse.ArgumentParser(description="MQTT 로 로봇들을 관측해 맵 위에 표시")
    parser.add_argument("metadata", help="맵 메타데이터 JSON 경로")
    parser.add_argument("--host", default="localhost", help="MQTT 브로커 호스트")
    parser.add_argument("--port", type=int, default=1883, help="MQTT 브로커 포트")
    parser.add_argument("--grid-step", type=float, default=1.0, help="격자 간격 (m)")
    parser.add_argument("--no-orders", action="store_true",
                        help="order 토픽은 구독하지 않는다")
    parser.add_argument("--no-panel", action="store_true", help="우측 정보 패널 숨김")
    parser.add_argument("--log-level", default="INFO")
    args = parser.parse_args()

    logging.basicConfig(
        level=getattr(logging, args.log_level.upper(), logging.INFO),
        format="%(asctime)s %(levelname)-7s %(message)s",
        datefmt="%H:%M:%S",
    )

    import matplotlib.pyplot as plt

    viewer = MapViewer(
        MapModel.load(args.metadata),
        grid_step=args.grid_step,
        show_panel=not args.no_panel,
    )
    viewer.build()

    monitor = FleetMonitor(viewer, args.host, args.port, watch_orders=not args.no_orders)
    try:
        monitor.start()
    except OSError as e:
        log.error("브로커 접속 실패 (%s:%d): %s — mosquitto 가 떠 있는지 확인",
                  args.host, args.port, e)
        return 1

    # 큐를 비우고 화면을 갱신하는 타이머. 메인 스레드에서만 그린다
    timer = viewer._fig.canvas.new_timer(interval=REFRESH_PERIOD_MS)
    timer.add_callback(lambda: (monitor.pump(), viewer.refresh(time.time())))
    timer.start()

    print(MapViewer._usage_text())
    print(f"브로커 {args.host}:{args.port} 관측 중. 로봇 프로세스를 띄우면 여기 나타난다.\n")
    try:
        plt.show()
    finally:
        timer.stop()
        monitor.stop()
    return 0


if __name__ == "__main__":
    sys.exit(main())
