"""
오더 실행기 (로봇 상태머신)
=========================

오더를 받아 노드를 순서대로 처리하고, 매 tick 마다 State 스냅샷을 만든다.
운동학·배터리 시뮬레이션을 소유하지만, 밖에서 보면 그냥 "로봇이 하는 일" 이다.

    IDLE ─(order)→ MOVING ─(도착)→ ACTING ─(완료)→ MOVING ...
                     │                            └→ IDLE
                     └─(released:false)→ WAITING
    ERROR (어느 상태에서든 진입)

released 처리 — 트래픽 제어의 핵심
---------------------------------
FMS 는 안전이 확보된 노드만 released=True 로 내려보낸다. 로봇은 released=False
노드 **직전에서 멈추고** WAITING 이 된다. 스키마가 "released 는 앞에서부터 연속"
을 보장하므로, 처음 나오는 False 이후는 전부 False 다.

주행 구간(segment)은 current_index 부터
  - released=False 노드를 만나거나
  - action 이 있는 노드에 도달하면 (거기서 멈춰야 하므로)
끊어서 잡는다. 그 사이 노드들은 감속 없이 통과한다.

order_update_id
--------------
같은 order_id 로 재발행될 때 자신이 가진 값보다 큰 것만 수용한다. 재전송이나
순서 뒤바뀜으로 오래된 오더가 도착해도 통행권을 되돌리지 않기 위한 규칙이다.
"""

from __future__ import annotations

import logging
from typing import TYPE_CHECKING, Optional

from common.schemas import (
    Action,
    ActionProgress,
    ActionStatus,
    ErrorInfo,
    ErrorLevel,
    InstantAction,
    InstantActionType,
    NodeProgress,
    Order,
    OrderNode,
    Point,
    Pose,
    RobotState,
    State,
)
from robot_client.planner import PlanningError, plan_path
from robot_client.sim.battery import BatterySim
from robot_client.sim.kinematics import TrapezoidalKinematics

if TYPE_CHECKING:
    from common.map_model import MapModel

log = logging.getLogger(__name__)


class OrderExecutor:
    """오더 하나를 끝까지 실행하는 상태머신."""

    def __init__(
        self,
        robot_id: str,
        start_x: float,
        start_y: float,
        start_theta: float = 0.0,
        max_velocity: float = 1.2,
        max_acceleration: float = 0.6,
        battery: Optional[BatterySim] = None,
        map_model: Optional["MapModel"] = None,
        robot_width: float = 0.5,
        robot_length: float = 0.7,
    ):
        self.robot_id = robot_id
        self.robot_width = robot_width
        self.robot_length = robot_length
        # 맵은 이제 두 가지 용도다: (1) 경로계획 — FMS 는 목적지만 주고, 거기까지
        # 장애물을 피해 가는 경로는 여기서 A* 로 스스로 짠다 (robot_client/planner.py).
        # (2) 안전장치 — 실제 AMR 이 범퍼/라이다로 스스로 멈추듯, 계획한 경로를 밟다가도
        # 실제로 장애물 셀에 들어가면 ERROR 로 잡는다. 맵이 없으면 둘 다 꺼지고
        # (경로계획 불가) 노드 사이 직선 이동만 한다
        self.map = map_model
        self.kinematics = TrapezoidalKinematics(
            x=start_x, y=start_y, theta=start_theta,
            max_velocity=max_velocity,
            max_acceleration=max_acceleration,
            max_deceleration=max_acceleration,
        )
        self.battery = battery or BatterySim(max_velocity=max_velocity)

        self.state = RobotState.IDLE
        self.errors: list[ErrorInfo] = []

        self.order_id: Optional[str] = None
        self.order_update_id: int = 0
        self.nodes: list[OrderNode] = []
        self.current_index: int = 0          # 아직 도달하지 않은 첫 노드
        self.last_node_id: str = ""
        self.last_node_sequence_id: int = 0

        # 마지막으로 완료한 (order_id, order_update_id). 완료 뒤 order_id 를 비우기 때문에
        # 이걸 따로 기억하지 않으면, QoS 1 재전송이나 FMS 재시작으로 같은 오더가 다시
        # 오면 처음부터 또 실행한다
        self._last_completed: Optional[tuple[str, int]] = None

        self._segment: list[int] = []        # 이번 주행 구간의 노드 인덱스들
        self._acting: Optional[Action] = None
        self._acting_elapsed: float = 0.0
        self._action_states: dict[str, ActionStatus] = {}
        self._paused = False

    # -- 오더 수신 --------------------------------------------------------

    def accept_order(self, order: Order) -> bool:
        """오더를 수용하면 True. 오래된 update 나 다른 오더 실행 중이면 False."""
        if (
            self._last_completed is not None
            and order.order_id == self._last_completed[0]
            and order.order_update_id <= self._last_completed[1]
        ):
            log.info("[%s] 이미 완료한 오더 무시: %s update %d",
                     self.robot_id, order.order_id, order.order_update_id)
            return False

        if self.order_id == order.order_id:
            if order.order_update_id <= self.order_update_id:
                log.info(
                    "[%s] 오래된 오더 무시: %s update %d <= %d",
                    self.robot_id, order.order_id, order.order_update_id, self.order_update_id,
                )
                return False
            return self._apply_order_update(order)

        if self.order_id is not None and self.state != RobotState.IDLE:
            log.warning(
                "[%s] 다른 오더 실행 중이라 %s 거절 (진행 중: %s)",
                self.robot_id, order.order_id, self.order_id,
            )
            return False

        self.order_id = order.order_id
        self.order_update_id = order.order_update_id
        self.nodes = list(order.nodes)
        self.current_index = 0
        self._action_states = {
            n.action.action_id: ActionStatus.WAITING for n in self.nodes if n.action
        }
        self._acting = None
        self._replan()
        log.info(
            "[%s] 오더 수신 %s update=%d, 노드 %d개 (released %d개)",
            self.robot_id, order.order_id, order.order_update_id,
            len(self.nodes), sum(1 for n in self.nodes if n.released),
        )
        return True

    def _apply_order_update(self, order: Order) -> bool:
        """
        같은 오더의 갱신. 노드가 추가되거나 released 가 열린 경우다.
        이미 지나온 노드는 그대로 두고 앞쪽만 교체한다.
        """
        old_len = len(self.nodes)
        self.order_update_id = order.order_update_id
        self.nodes = list(order.nodes)
        for n in self.nodes:
            if n.action and n.action.action_id not in self._action_states:
                self._action_states[n.action.action_id] = ActionStatus.WAITING

        # 진행 중인 액션은 건드리지 않는다. 아니면 통행권이 열린 만큼 다시 계획
        if self._acting is None:
            self._replan()
        log.info(
            "[%s] 오더 갱신 %s update=%d, 노드 %d -> %d개 (released %d개)",
            self.robot_id, order.order_id, order.order_update_id,
            old_len, len(self.nodes), sum(1 for n in self.nodes if n.released),
        )
        return True

    # -- 즉시 명령 --------------------------------------------------------

    def handle_instant(self, action: InstantAction) -> None:
        kind = action.action_type
        if kind == InstantActionType.CANCEL_ORDER:
            log.info("[%s] 오더 취소: %s", self.robot_id, self.order_id)
            self._clear_order()
        elif kind == InstantActionType.EMERGENCY_STOP:
            log.warning("[%s] 긴급정지", self.robot_id)
            self._paused = True
            self.kinematics.emergency_stop()
            self.state = RobotState.PAUSED
        elif kind == InstantActionType.RESUME:
            log.info("[%s] 재개", self.robot_id)
            self._paused = False
            self._replan()
        elif kind == InstantActionType.RESET_ERROR:
            log.info("[%s] 오류 초기화", self.robot_id)
            self.errors.clear()
            if self.state == RobotState.ERROR:
                # FATAL 이 나면 오더는 이미 버려졌다. 새 오더를 받을 수 있는 상태로 돌아간다
                self.state = RobotState.IDLE if self.order_id is None else RobotState.MOVING
        elif kind == InstantActionType.INJECT_FAULT:
            fault = str(action.params.get("fault", "UNKNOWN_FAULT"))
            log.warning("[%s] 장애 주입: %s (오더 %s 중단)", self.robot_id, fault, self.order_id)
            self.raise_fatal(
                ErrorInfo(error_type=fault, level=ErrorLevel.FATAL, description="주입된 장애")
            )

    # -- 매 tick ----------------------------------------------------------

    def tick(self, dt: float) -> None:
        if self.state == RobotState.ERROR or self._paused:
            self.battery.step(dt, velocity=0.0)
            return

        if self._acting is not None:
            self._tick_action(dt)
            self.battery.step(dt, velocity=0.0)
            return

        if self.order_id is None:
            self.state = RobotState.IDLE
            self.battery.step(dt, velocity=0.0)
            return

        if self.current_index >= len(self.nodes):
            log.info("[%s] 오더 완료: %s", self.robot_id, self.order_id)
            self._last_completed = (self.order_id, self.order_update_id)
            self._clear_order()
            self.battery.step(dt, velocity=0.0)
            return

        # 다음 노드에 통행권이 없으면 여기서 대기
        if not self.nodes[self.current_index].released:
            self.state = RobotState.WAITING
            self.battery.step(dt, velocity=0.0)
            return

        if not self._segment:
            self._replan()
            if not self._segment:
                self.state = RobotState.WAITING
                self.battery.step(dt, velocity=0.0)
                return

        prev_x, prev_y = self.kinematics.x, self.kinematics.y
        kin_state = self.kinematics.step(dt)
        self._consume_reached_nodes()
        self.battery.step(dt, velocity=kin_state.velocity)

        if self._check_collision(kin_state.x, kin_state.y, prev_x, prev_y):
            return

        if self.kinematics.is_path_complete:
            self._on_segment_arrived()
        else:
            self.state = RobotState.MOVING

    def _check_collision(self, x: float, y: float, prev_x: float, prev_y: float) -> bool:
        """
        주행 불가 셀에 들어갔으면 멈추고 ERROR 로 간다. 로봇 자신의 안전장치이며
        (실물이면 범퍼/라이다가 하는 일), 잘못된 경로를 조용히 지나치지 않게 한다.
        맵을 주지 않았으면 검사하지 않는다.

        멈추는 위치는 직전 스텝의 위치다. 장애물 안에 서 있으면 어떤 오더를 새로 받아도
        첫 tick 에 다시 충돌해서 빠져나올 수 없다. 실물 로봇도 부딪히기 전에 멈춘다.
        """
        if self.map is None or self.map.is_free(x, y):
            return False

        cell = self.map.cell_at(x, y)
        log.error("[%s] 충돌: (%.2f, %.2f) 는 %s 셀이다. (%.2f, %.2f) 에 정지, 오더 %s 중단",
                  self.robot_id, x, y, cell.value, prev_x, prev_y, self.order_id)
        self.kinematics.set_pose(prev_x, prev_y)
        self.raise_fatal(ErrorInfo(
            error_type="COLLISION",
            level=ErrorLevel.FATAL,
            description=f"({x:.2f}, {y:.2f}) 가 {cell.value} 셀",
        ))
        return True

    def raise_fatal(self, error: ErrorInfo) -> None:
        """
        FATAL 오류. 즉시 멈추고 **오더를 버린다**.

        오더를 들고 있으면 오류를 초기화하는 순간 같은 경로로 다시 달려 같은 사고를
        반복한다. CLAUDE.md 의 ErrorLevel 규칙대로 FATAL 은 작업 중단이고,
        재배정은 FMS 몫이다. 로봇은 ERROR 로 서서 사람/FMS 의 개입을 기다린다.
        """
        self.errors.append(error)
        self.kinematics.emergency_stop()
        self._clear_order()
        self.state = RobotState.ERROR

    def _tick_action(self, dt: float) -> None:
        self.state = RobotState.ACTING
        self._acting_elapsed += dt
        if self._acting_elapsed < self._acting.duration:
            return
        self._action_states[self._acting.action_id] = ActionStatus.FINISHED
        log.info("[%s] 액션 완료: %s (%s)",
                 self.robot_id, self._acting.action_id, self._acting.action_type.value)
        self._acting = None
        self._acting_elapsed = 0.0
        self._replan()

    # -- 내부: 경로 계획 ---------------------------------------------------

    def _replan(self) -> None:
        """current_index 부터 멈춰야 하는 지점까지를 주행 구간으로 잡는다."""
        self._segment = []
        if self._paused or self.order_id is None:
            self.kinematics.set_path([])
            return

        for i in range(self.current_index, len(self.nodes)):
            node = self.nodes[i]
            if not node.released:
                break               # 통행권 없음 -> 직전에서 멈춘다
            self._segment.append(i)
            if node.action is not None:
                break               # 액션 노드에서는 멈춰야 한다

        try:
            path = self._plan_segment_path()
        except PlanningError as e:
            log.error("[%s] 경로계획 실패: %s", self.robot_id, e)
            self._segment = []
            self.raise_fatal(ErrorInfo(
                error_type="NO_PATH", level=ErrorLevel.FATAL, description=str(e),
            ))
            return

        self.kinematics.set_path(path)

    def _plan_segment_path(self) -> list[tuple[float, float]]:
        """
        이번 주행 구간의 목표 노드들까지 실제로 밟을 점렬을 만든다.

        맵이 있으면 현재 위치 -> 첫 목표, 목표 -> 다음 목표 순으로 각각 A* 를 돌려
        장애물을 피하는 경로를 잇는다 (FMS 는 이제 목적지 좌표만 준다 — 그 사이를
        어떻게 갈지는 로봇 책임). 맵이 없으면 예전처럼 노드 사이를 직선으로 잇는다.
        """
        targets = [(self.nodes[i].position.x, self.nodes[i].position.y) for i in self._segment]
        if self.map is None:
            return targets

        full_path: list[tuple[float, float]] = []
        start = (self.kinematics.x, self.kinematics.y)
        for target in targets:
            full_path.extend(plan_path(
                self.map, start, target,
                robot_width=self.robot_width, robot_length=self.robot_length,
            ))
            start = target
        return full_path

    def _consume_reached_nodes(self) -> None:
        """운동학이 지나친 경유점만큼 current_index 와 last_node 를 밀어준다."""
        remaining = len(self.kinematics.get_state().local_path)
        reached = len(self._segment) - remaining
        if reached <= 0:
            return
        last_reached = self._segment[reached - 1]
        node = self.nodes[last_reached]
        self.last_node_id = node.node_id
        self.last_node_sequence_id = node.sequence_id
        self.current_index = max(self.current_index, last_reached + 1)

    def _on_segment_arrived(self) -> None:
        """구간 끝 노드에 도착했다. 액션이 있으면 수행하고, 없으면 다음 구간으로."""
        if not self._segment:
            self._replan()
            return
        node_index = self._segment[-1]
        node = self.nodes[node_index]
        self._segment = []

        # 여기서 직접 밀어준다. _consume_reached_nodes 는 운동학이 경유점을 버린 만큼만
        # 세는데, 이동 거리가 0인 노드(이미 그 자리에 서 있는 경우)는 버려지지 않아
        # current_index 가 영영 멈춰 있게 된다
        self.last_node_id = node.node_id
        self.last_node_sequence_id = node.sequence_id
        self.current_index = max(self.current_index, node_index + 1)

        if node.action is not None and self._action_states.get(node.action.action_id) != ActionStatus.FINISHED:
            self._acting = node.action
            self._acting_elapsed = 0.0
            self._action_states[node.action.action_id] = ActionStatus.RUNNING
            self.state = RobotState.ACTING
            log.info("[%s] 액션 시작: %s (%s, %.1fs)", self.robot_id,
                     node.action.action_id, node.action.action_type.value, node.action.duration)
            return

        self._replan()
        if not self._segment:
            # 더 갈 곳이 없다. 오더가 끝났거나 통행권 대기
            self.state = (
                RobotState.WAITING
                if self.current_index < len(self.nodes)
                else RobotState.IDLE
            )

    def _clear_order(self) -> None:
        self.order_id = None
        self.order_update_id = 0
        self.nodes = []
        self.current_index = 0
        self._segment = []
        self._acting = None
        self._acting_elapsed = 0.0
        self._action_states = {}
        self.kinematics.set_path([])
        self.state = RobotState.IDLE

    def force_relocalize(self, x: float, y: float, theta: float = 0.0) -> bool:
        """
        로컬라이징을 강제로 다시 잡는다. **경로를 그려서 이동하는 게 아니라, 좌표/각도
        값 자체를 그 자리에서 그냥 덮어쓴다** — 실물 로봇의 위치추정기가 잘못 잡았을 때
        사람이 "여기가 진짜 니 위치야" 하고 값을 정정해주는 것과 같다. 디버그/테스트
        전용이며 로봇 제어판(robot_client/gui.py)에서만 호출한다. FMS 프로토콜
        (order/instant)에는 이런 명령이 없다 (실물이 순간이동할 수 없으니 당연하다).

        경로계획을 안 타므로 맵의 주행 가능 여부도 검사하지 않는다 — 사람이 정정하는
        값이니 시스템이 막을 이유가 없다 (맵 자체가 틀렸을 수도 있는 상황이다).

        위치가 갑자기 바뀌므로 진행 중이던 오더의 경로는 더 이상 의미가 없다 — 오더를 버린다.
        """
        self._clear_order()
        self.kinematics.emergency_stop()
        self.kinematics.set_pose(x, y, theta)
        log.warning("[%s] 강제 재위치: (%.2f, %.2f, %.2f)", self.robot_id, x, y, theta)
        return True

    # -- 상태 보고 --------------------------------------------------------

    def build_state(self, header_id: int, timestamp: float) -> State:
        """FMS 로 보낼 State 메시지. 로봇에 대해 FMS 가 아는 유일한 진실이다."""
        k = self.kinematics.get_state()
        return State(
            header_id=header_id,
            timestamp=timestamp,
            robot_id=self.robot_id,
            state=self.state,
            pose=Pose(x=round(k.x, 4), y=round(k.y, 4), theta=round(k.theta, 4)),
            velocity=round(k.velocity, 4),
            width=self.robot_width,
            length=self.robot_length,
            local_path=[Point(x=round(x, 4), y=round(y, 4)) for x, y in k.local_path],
            last_node_id=self.last_node_id,
            last_node_sequence_id=self.last_node_sequence_id,
            order_id=self.order_id,
            order_update_id=self.order_update_id,
            node_states=[
                NodeProgress(node_id=n.node_id, sequence_id=n.sequence_id, released=n.released)
                for n in self.nodes[self.current_index:]
            ],
            action_states=[
                ActionProgress(action_id=aid, status=st)
                for aid, st in self._action_states.items()
            ],
            battery=self.battery.info(),
            errors=list(self.errors),
        )
