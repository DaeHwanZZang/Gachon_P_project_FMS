"""
schemas.py 사용 예제.
python3 example_usage.py 로 실행하면 동작을 확인할 수 있다.
"""

from pydantic import ValidationError

from common.schemas import (
    Action, ActionType, BatteryInfo, Connection, ConnectionState,
    InstantAction, InstantActionType, Order, OrderNode, Pose,
    RobotState, State, TimeWindow, topic_order, topic_state,
)


def demo_fms_sends_order():
    """[FMS] 작업 지시서 생성 -> JSON 발행"""
    order = Order(
        header_id=1,
        timestamp=1200.0,
        robot_id="AMR-001",
        order_id="ORD-00123",
        task_id="TASK-777",
        nodes=[
            OrderNode(
                node_id="N12", sequence_id=0, released=True,
                position=Pose(x=12.0, y=8.0, theta=0.0),
                reserved_window=TimeWindow(enter=1200.0, exit=1206.0),
            ),
            OrderNode(
                node_id="N18", sequence_id=1, released=True,
                position=Pose(x=18.0, y=8.0, theta=0.0),
                action=Action(action_id="A-1", action_type=ActionType.PICK,
                              duration=5.0, payload_id="PLT-042"),
                reserved_window=TimeWindow(enter=1206.0, exit=1215.0),
            ),
            OrderNode(
                # 아직 통행권 없음 -> 로봇은 N18 에서 대기
                node_id="N25", sequence_id=2, released=False,
                position=Pose(x=25.0, y=8.0, theta=0.0),
                action=Action(action_id="A-2", action_type=ActionType.DROP,
                              duration=5.0, payload_id="PLT-042"),
            ),
        ],
    )
    payload = order.model_dump_json()
    print(f"publish -> {topic_order('AMR-001')}")
    print(payload[:160] + " ...\n")
    return payload


def demo_robot_receives(payload: str):
    """[로봇] 수신 -> 파싱 -> 진행 가능한 노드까지만 실행"""
    order = Order.model_validate_json(payload)
    runnable = [n for n in order.nodes if n.released]
    print(f"수신 오더 {order.order_id} / 전체 {len(order.nodes)}개 노드")
    print(f"진행 가능: {[n.node_id for n in runnable]}")
    print(f"대기 지점: {[n.node_id for n in order.nodes if not n.released]}\n")


def demo_robot_reports():
    """[로봇] 상태 보고 -> JSON 발행"""
    state = State(
        header_id=42,
        timestamp=1207.4,
        robot_id="AMR-001",
        state=RobotState.ACTING,
        pose=Pose(x=18.0, y=8.0, theta=0.0),
        velocity=0.0,
        last_node_id="N18",
        last_node_sequence_id=1,
        order_id="ORD-00123",
        battery=BatteryInfo(charge=78.3, estimated_runtime=4200.0),
    )
    print(f"publish -> {topic_state('AMR-001')}")
    print(f"available? {state.is_available}\n")
    return state.model_dump_json()


def demo_validation_catches_bugs():
    """검증이 실제로 버그를 잡는 사례들"""

    print("--- 검증 실패 사례 ---")

    # 1) 필수 필드 누락
    try:
        State.model_validate({"robot_id": "AMR-001"})
    except ValidationError as e:
        print(f"1. 필드 누락 -> {e.error_count()}건 검출")

    # 2) 오타난 필드명 (extra=forbid 덕분에 잡힘)
    try:
        Pose(x=1.0, y=2.0, thetaa=0.0)
    except ValidationError as e:
        print(f"2. 오타 필드 -> {e.errors()[0]['type']}")

    # 3) 배터리 범위 초과
    try:
        BatteryInfo(charge=150.0)
    except ValidationError as e:
        print(f"3. 범위 초과 -> {e.errors()[0]['msg']}")

    # 4) 없는 상태값
    try:
        State.model_validate({
            "header_id": 1, "timestamp": 0.0, "robot_id": "AMR-001",
            "state": "MOVINNG",  # 오타
            "pose": {"x": 0, "y": 0, "theta": 0},
            "last_node_id": "N0",
            "battery": {"charge": 50.0},
        })
    except ValidationError as e:
        print(f"4. 잘못된 상태값 -> {e.errors()[0]['type']}")

    # 5) released 순서 규칙 위반
    try:
        Order(
            header_id=1, timestamp=0.0, robot_id="AMR-001", order_id="O-1",
            nodes=[
                OrderNode(node_id="N1", sequence_id=0, released=False,
                          position=Pose(x=0, y=0, theta=0)),
                OrderNode(node_id="N2", sequence_id=1, released=True,
                          position=Pose(x=1, y=0, theta=0)),
            ],
        )
    except ValidationError as e:
        print(f"5. released 규칙 위반 -> 검출됨")

    # 6) 문자열 -> float 자동 변환 (이건 통과)
    b = BatteryInfo(charge="78.3")
    print(f"6. 자동 변환 -> {b.charge!r} ({type(b.charge).__name__})")
    print()


def demo_instant_action():
    """[FMS] 긴급 명령 / 장애 주입"""
    stop = InstantAction(
        header_id=99, timestamp=1300.0, robot_id="AMR-005",
        action_id="IA-1", action_type=InstantActionType.EMERGENCY_STOP,
    )
    fault = InstantAction(
        header_id=100, timestamp=1300.0, robot_id="AMR-005",
        action_id="IA-2", action_type=InstantActionType.INJECT_FAULT,
        params={"fault": "MOTOR_FAULT", "duration": 30.0},
    )
    print(f"긴급정지: {stop.action_type.value}")
    print(f"장애주입: {fault.params}\n")


def demo_connection():
    """[로봇] 접속 알림 / LWT 등록용 메시지"""
    online = Connection(header_id=0, timestamp=1000.0, robot_id="AMR-001",
                        connection_state=ConnectionState.ONLINE)
    lwt = Connection(header_id=0, timestamp=0.0, robot_id="AMR-001",
                     connection_state=ConnectionState.CONNECTION_BROKEN)
    print(f"접속 시 발행: {online.connection_state.value}")
    print(f"LWT 로 등록:  {lwt.connection_state.value}")


if __name__ == "__main__":
    print("=" * 60)
    payload = demo_fms_sends_order()
    demo_robot_receives(payload)
    demo_robot_reports()
    demo_validation_catches_bugs()
    demo_instant_action()
    demo_connection()
    print("=" * 60)
