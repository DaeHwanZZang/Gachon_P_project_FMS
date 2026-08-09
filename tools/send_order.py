"""
오더 발행 도구 (임시 FMS 대역)
============================

FMS 서버가 아직 없으니, 로봇에게 오더를 던져보기 위한 개발용 도구다.
FMS 가 생기면 이 역할은 fms_server/allocator.py 로 넘어간다.

    # 경유점 3개짜리 오더
    python tools/send_order.py AMR-001 --path 1.2,6 1.2,2 7.4,2 7.4,9

    # 마지막 노드는 통행권 없이 (로봇이 그 직전에서 WAITING)
    python tools/send_order.py AMR-001 --path 1.2,6 1.2,2 7.4,2 --released 2

    # 통행권을 열어주는 갱신 (같은 order_id, update_id 를 올려서 재발행)
    python tools/send_order.py AMR-001 --path 1.2,6 1.2,2 7.4,2 \\
        --order-id ORD-1 --update-id 1

    # 마지막 노드에서 PICK 5초
    python tools/send_order.py AMR-001 --path 1.2,6 7.4,2 --action PICK:5

    # 즉시 명령
    python tools/send_order.py AMR-001 --instant EMERGENCY_STOP
    python tools/send_order.py AMR-001 --instant RESUME
    python tools/send_order.py AMR-001 --instant INJECT_FAULT --fault MOTOR_FAULT
"""

from __future__ import annotations

import argparse
import math
import sys
import time
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

import paho.mqtt.client as mqtt  # noqa: E402

from common.schemas import (  # noqa: E402
    Action,
    ActionType,
    InstantAction,
    InstantActionType,
    Order,
    OrderNode,
    Pose,
    topic_instant,
    topic_order,
)
from tools.map_viewer import parse_xy  # noqa: E402


def build_order(robot_id: str, waypoints: list[tuple[float, float]], order_id: str,
                update_id: int, released_count: int | None,
                action_spec: str | None) -> Order:
    if released_count is None:
        released_count = len(waypoints)

    action = None
    if action_spec:
        kind, _, duration = action_spec.partition(":")
        action = Action(
            action_id=f"{order_id}-A1",
            action_type=ActionType(kind.upper()),
            duration=float(duration) if duration else 5.0,
        )

    nodes = []
    for i, (x, y) in enumerate(waypoints):
        # 다음 경유점을 바라보는 각도. 마지막 노드는 직전 방향을 유지한다
        if i + 1 < len(waypoints):
            theta = math.atan2(waypoints[i + 1][1] - y, waypoints[i + 1][0] - x)
        elif i > 0:
            theta = math.atan2(y - waypoints[i - 1][1], x - waypoints[i - 1][0])
        else:
            theta = 0.0
        nodes.append(OrderNode(
            node_id=f"N{i}",
            sequence_id=i,
            released=i < released_count,
            position=Pose(x=x, y=y, theta=round(theta, 4)),
            action=action if (action and i == len(waypoints) - 1) else None,
        ))

    return Order(
        header_id=1,
        timestamp=time.time(),
        robot_id=robot_id,
        order_id=order_id,
        order_update_id=update_id,
        nodes=nodes,
    )


def main() -> int:
    parser = argparse.ArgumentParser(description="로봇에 오더/즉시명령 발행 (임시 FMS 대역)")
    parser.add_argument("robot_id", help="대상 로봇. 예: AMR-001")
    parser.add_argument("--host", default="localhost")
    parser.add_argument("--port", type=int, default=1883)
    parser.add_argument("--path", type=parse_xy, nargs="+", metavar="X,Y",
                        help="경유점 목록")
    parser.add_argument("--order-id", default=None, help="오더 식별자. 생략하면 자동 생성")
    parser.add_argument("--update-id", type=int, default=0,
                        help="order_update_id. 같은 오더를 갱신할 때 올린다")
    parser.add_argument("--released", type=int, default=None, metavar="N",
                        help="앞에서부터 N개 노드만 통행권 부여 (생략하면 전부)")
    parser.add_argument("--action", metavar="TYPE[:SEC]",
                        help="마지막 노드에 붙일 액션. 예: PICK:5, DROP:3, CHARGE:20")
    parser.add_argument("--instant", metavar="TYPE",
                        help="즉시 명령. CANCEL_ORDER / EMERGENCY_STOP / RESUME / "
                             "RESET_ERROR / INJECT_FAULT")
    parser.add_argument("--fault", default="MOTOR_FAULT",
                        help="INJECT_FAULT 일 때 주입할 장애 이름")
    args = parser.parse_args()

    if not args.instant and not args.path:
        parser.error("--path 또는 --instant 중 하나는 필요하다")

    client = mqtt.Client(mqtt.CallbackAPIVersion.VERSION2, client_id=f"sender-{int(time.time())}")
    try:
        client.connect(args.host, args.port, 10)
    except OSError as e:
        print(f"브로커 접속 실패 ({args.host}:{args.port}): {e} — mosquitto 가 떠 있는지 확인")
        return 1
    client.loop_start()

    if args.instant:
        message = InstantAction(
            header_id=1,
            timestamp=time.time(),
            robot_id=args.robot_id,
            action_id=f"IA-{int(time.time())}",
            action_type=InstantActionType(args.instant.upper()),
            params={"fault": args.fault} if args.instant.upper() == "INJECT_FAULT" else {},
        )
        topic = topic_instant(args.robot_id)
        info = client.publish(topic, message.model_dump_json(), qos=1)
        info.wait_for_publish(timeout=5.0)
        print(f"발행 -> {topic}\n  {message.action_type.value}")
    else:
        order_id = args.order_id or f"ORD-{int(time.time()) % 100000}"
        order = build_order(
            args.robot_id, args.path, order_id, args.update_id, args.released, args.action
        )
        topic = topic_order(args.robot_id)
        info = client.publish(topic, order.model_dump_json(), qos=1)
        info.wait_for_publish(timeout=5.0)
        released = sum(1 for n in order.nodes if n.released)
        print(f"발행 -> {topic}")
        print(f"  order_id={order.order_id} update_id={order.order_update_id}")
        print(f"  노드 {len(order.nodes)}개 (통행권 {released}개)")
        for n in order.nodes:
            act = f"  {n.action.action_type.value}({n.action.duration}s)" if n.action else ""
            print(f"    {n.sequence_id} {n.node_id} ({n.position.x:.2f}, {n.position.y:.2f}) "
                  f"released={n.released}{act}")

    client.loop_stop()
    client.disconnect()
    return 0


if __name__ == "__main__":
    sys.exit(main())
