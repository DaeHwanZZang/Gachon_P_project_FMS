"""
로봇 MQTT 통신
=============

로봇 프로세스 하나가 브로커에 붙어 자기 토픽만 쓰고 읽는다.

    fms/v1/{robot_id}/connection   발행 (retain, LWT)
    fms/v1/{robot_id}/state        발행 (주기 보고)
    fms/v1/{robot_id}/order        구독
    fms/v1/{robot_id}/instant      구독

수신 메시지는 paho 의 네트워크 스레드에서 콜백으로 들어온다. 그 스레드에서
로봇 상태를 직접 건드리면 경합이 생기므로, 파싱만 해서 큐에 넣고 실제 처리는
메인 루프가 drain 해서 한다. 락 없이 단일 스레드 상태머신을 유지하기 위한 구조다.

검증에 실패한 메시지는 버리고 로그만 남긴다. 로봇은 이상한 오더를 받느니
하던 일을 계속하거나 멈춰 있는 쪽이 안전하다.
"""

from __future__ import annotations

import logging
import queue
import time
from typing import Optional, Union

import paho.mqtt.client as mqtt
from pydantic import ValidationError

from common.schemas import (
    Connection,
    ConnectionState,
    InstantAction,
    Order,
    State,
    topic_connection,
    topic_instant,
    topic_order,
    topic_state,
)

log = logging.getLogger(__name__)

Incoming = Union[Order, InstantAction]

QOS_STATE = 0       # 200ms 주기 텔레메트리. 한 개쯤 유실돼도 다음 게 온다
QOS_CONTROL = 1     # 오더/즉시명령/접속상태는 반드시 도착해야 한다


class RobotComm:
    """로봇 한 대의 MQTT 입출력."""

    def __init__(
        self,
        robot_id: str,
        host: str = "localhost",
        port: int = 1883,
        keepalive: int = 15,
    ):
        self.robot_id = robot_id
        self.host = host
        self.port = port
        self.keepalive = keepalive

        self.inbox: "queue.Queue[Incoming]" = queue.Queue()
        self._header_id = 0
        self._connected = False

        self._client = self._build_client()

    def _build_client(self) -> mqtt.Client:
        client = mqtt.Client(
            mqtt.CallbackAPIVersion.VERSION2,
            client_id=f"robot-{self.robot_id}",
            clean_session=True,
        )
        client.on_connect = self._on_connect
        client.on_disconnect = self._on_disconnect
        client.on_message = self._on_message

        # 프로세스가 강제 종료되면 브로커가 대신 이걸 발행한다 (docker stop 장애 주입)
        client.will_set(
            topic_connection(self.robot_id),
            self._connection_payload(ConnectionState.CONNECTION_BROKEN),
            qos=QOS_CONTROL,
            retain=True,
        )
        return client

    # -- 접속 ------------------------------------------------------------

    def connect(self) -> None:
        """브로커에 접속하고 네트워크 스레드를 띄운다."""
        self._client.connect(self.host, self.port, self.keepalive)
        self._client.loop_start()

    def rebind_id(self, new_robot_id: str) -> None:
        """로봇 ID 를 바꾸고 새 토픽으로 재접속한다 (GUI 설정 변경 전용, IDLE 일 때만 호출할 것).

        LWT 는 최초 CONNECT 패킷에 실려야 브로커에 등록되므로, 이미 접속된
        세션의 will 을 그냥 덮어쓸 수 없다 — 옛 세션을 OFFLINE 으로 정리하고
        새 client_id/LWT 로 통째로 재접속한다.
        """
        if new_robot_id == self.robot_id:
            return
        old_id = self.robot_id
        try:
            info = self._client.publish(
                topic_connection(old_id),
                self._connection_payload(ConnectionState.OFFLINE),
                qos=QOS_CONTROL,
                retain=True,
            )
            info.wait_for_publish(timeout=2.0)
        except Exception as e:
            log.warning("[%s] ID 변경 중 OFFLINE 발행 실패: %s", old_id, e)
        self._client.loop_stop()
        self._client.disconnect()

        self.robot_id = new_robot_id
        self._client = self._build_client()
        self.connect()
        log.info("[%s] ID 변경 완료 (이전: %s)", new_robot_id, old_id)

    def disconnect(self) -> None:
        """정상 종료. OFFLINE 을 남기고 끊는다 (LWT 는 발행되지 않는다)."""
        try:
            info = self._client.publish(
                topic_connection(self.robot_id),
                self._connection_payload(ConnectionState.OFFLINE),
                qos=QOS_CONTROL,
                retain=True,
            )
            info.wait_for_publish(timeout=2.0)
        except Exception as e:  # 종료 경로에서 예외로 죽지 않게
            log.warning("OFFLINE 발행 실패: %s", e)
        self._client.loop_stop()
        self._client.disconnect()

    @property
    def is_connected(self) -> bool:
        return self._connected

    def wait_until_connected(self, timeout: float = 5.0) -> bool:
        deadline = time.time() + timeout
        while time.time() < deadline:
            if self._connected:
                return True
            time.sleep(0.05)
        return False

    # -- 발행 ------------------------------------------------------------

    def publish_state(self, state: State) -> None:
        self._client.publish(
            topic_state(self.robot_id), state.model_dump_json(), qos=QOS_STATE, retain=False
        )

    def publish_online(self) -> None:
        self._client.publish(
            topic_connection(self.robot_id),
            self._connection_payload(ConnectionState.ONLINE),
            qos=QOS_CONTROL,
            retain=True,
        )

    def next_header_id(self) -> int:
        """송신 메시지에 붙일 단조 증가 카운터."""
        self._header_id += 1
        return self._header_id

    # -- 수신 ------------------------------------------------------------

    def drain(self) -> list[Incoming]:
        """큐에 쌓인 수신 메시지를 전부 꺼낸다. 메인 루프에서만 호출할 것."""
        items: list[Incoming] = []
        while True:
            try:
                items.append(self.inbox.get_nowait())
            except queue.Empty:
                return items

    # -- paho 콜백 (네트워크 스레드에서 실행됨) ------------------------------

    def _on_connect(self, client, userdata, flags, reason_code, properties=None) -> None:
        if reason_code != 0:
            log.error("[%s] 접속 실패: %s", self.robot_id, reason_code)
            return
        self._connected = True
        client.subscribe([
            (topic_order(self.robot_id), QOS_CONTROL),
            (topic_instant(self.robot_id), QOS_CONTROL),
        ])
        self.publish_online()
        log.info("[%s] 브로커 접속 %s:%d", self.robot_id, self.host, self.port)

    def _on_disconnect(self, client, userdata, flags, reason_code, properties=None) -> None:
        self._connected = False
        if reason_code:
            log.warning("[%s] 접속 끊김: %s (재접속 시도)", self.robot_id, reason_code)

    def _on_message(self, client, userdata, msg) -> None:
        try:
            payload = msg.payload.decode("utf-8")
        except UnicodeDecodeError:
            log.warning("[%s] 디코딩 불가 메시지 무시: %s", self.robot_id, msg.topic)
            return

        try:
            if msg.topic == topic_order(self.robot_id):
                self.inbox.put(Order.model_validate_json(payload))
            elif msg.topic == topic_instant(self.robot_id):
                self.inbox.put(InstantAction.model_validate_json(payload))
            else:
                log.debug("[%s] 구독하지 않은 토픽: %s", self.robot_id, msg.topic)
        except ValidationError as e:
            # 스키마에 맞지 않으면 버린다. 반쯤 해석한 오더를 실행하는 것보다 안전하다
            log.error("[%s] 잘못된 메시지 무시 (%s): %s", self.robot_id, msg.topic, e.error_count())

    # -- 내부 ------------------------------------------------------------

    def _connection_payload(self, state: ConnectionState) -> str:
        # LWT 는 접속 시점에 미리 등록해야 해서 header_id/timestamp 를 그때 확정한다.
        # 브로커가 나중에 대신 발행하므로 timestamp 는 "죽은 시각" 이 아니라 "등록 시각" 이다.
        return Connection(
            header_id=self.next_header_id(),
            timestamp=time.time(),
            robot_id=self.robot_id,
            connection_state=state,
        ).model_dump_json()
