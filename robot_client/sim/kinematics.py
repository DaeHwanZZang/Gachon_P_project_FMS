"""
로봇 물리 시뮬레이션 (운동학)
============================

실물 센서/모터가 없으므로 로컬라이징을 "구현"하지 않는다 — 시뮬레이션 자체가
곧 ground truth 이고, 로봇은 그 값을 그대로 자기 위치로 보고한다.

이 모듈은 FMS 에서 절대 import 하지 않는다 (CLAUDE.md 원칙 3).
FMS 는 로봇이 MQTT `state` 로 보고하는 값만 안다.

운동 모델: 등가속/등감속 사다리꼴 속도 프로파일.
    - 남은 전체 경로 길이 기준으로 가속/등속/감속 단계를 계산한다
      (직전 관유점마다 멈추지 않고, 경로 끝에서만 정지).
    - 코너링(관유점 통과)은 순간 처리 — theta 는 현재 구간 방향으로 즉시 갱신된다.
    - 경로계획(A*)은 FMS 담당. 여기서는 주어진 관유점 사이를 선형 보간만 한다.
"""

from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Iterable, Optional


@dataclass
class RobotKinematicState:
    """운동학 시뮬레이션 한 스텝의 결과. executor 가 이 값으로 State 메시지를 채운다."""

    x: float
    y: float
    theta: float                              # rad, -pi ~ pi
    velocity: float                            # m/s, >= 0
    acceleration: float                        # m/s^2, 부호 있음 (감속 시 음수)
    local_path: list[tuple[float, float]]       # 아직 도달하지 않은 관유점들 (world 좌표)
    path_complete: bool


class TrapezoidalKinematics:
    """관유점 목록을 따라가는 단일 로봇의 등가속/등감속 운동 시뮬레이션."""

    def __init__(
        self,
        x: float,
        y: float,
        theta: float = 0.0,
        max_velocity: float = 1.0,
        max_acceleration: float = 0.5,
        max_deceleration: float = 0.5,
    ):
        self.x = x
        self.y = y
        self.theta = theta
        self.max_velocity = max_velocity
        self.max_acceleration = max_acceleration
        self.max_deceleration = max_deceleration

        self._velocity = 0.0
        self._acceleration = 0.0
        self._remaining: list[tuple[float, float]] = []

    def set_path(self, waypoints: Iterable[tuple[float, float]]) -> None:
        """새 경로를 지정한다. 현재 속도는 유지한 채 이어서 따라간다."""
        self._remaining = list(waypoints)

    def set_pose(self, x: float, y: float, theta: Optional[float] = None) -> None:
        """위치를 강제로 지정한다. 충돌 직전 위치로 되돌리거나 리셋할 때 쓴다."""
        self.x = x
        self.y = y
        if theta is not None:
            self.theta = theta

    def emergency_stop(self) -> None:
        """
        즉시 정지. 감속 프로파일을 무시하고 속도를 0으로 만들고 경로를 버린다.
        긴급정지·장애 주입처럼 "지금 당장 멈춰야 하는" 경우에만 쓴다.
        """
        self._velocity = 0.0
        self._acceleration = 0.0
        self._remaining = []

    @property
    def is_path_complete(self) -> bool:
        return self._remaining_length() < 1e-9

    def step(self, dt: float) -> RobotKinematicState:
        remaining_dist = self._remaining_length()

        if remaining_dist < 1e-9:
            accel = -self.max_deceleration if self._velocity > 1e-9 else 0.0
        else:
            stop_dist = (
                self._velocity ** 2 / (2 * self.max_deceleration)
                if self.max_deceleration > 0
                else 0.0
            )
            if remaining_dist <= stop_dist:
                accel = -self.max_deceleration
            elif self._velocity < self.max_velocity:
                accel = self.max_acceleration
            else:
                accel = 0.0

        new_velocity = max(0.0, min(self.max_velocity, self._velocity + accel * dt))
        move_dist = min((self._velocity + new_velocity) / 2 * dt, remaining_dist)
        self._advance(move_dist)

        self._velocity = new_velocity if self._remaining_length() > 1e-9 else 0.0
        self._acceleration = accel
        return self.get_state()

    def get_state(self) -> RobotKinematicState:
        return RobotKinematicState(
            x=self.x,
            y=self.y,
            theta=self.theta,
            velocity=self._velocity,
            acceleration=self._acceleration,
            local_path=list(self._remaining),
            path_complete=self.is_path_complete,
        )

    # -- 내부 -------------------------------------------------------------

    def _remaining_length(self) -> float:
        if not self._remaining:
            return 0.0
        total = math.hypot(self._remaining[0][0] - self.x, self._remaining[0][1] - self.y)
        for (x1, y1), (x2, y2) in zip(self._remaining, self._remaining[1:]):
            total += math.hypot(x2 - x1, y2 - y1)
        return total

    def _advance(self, dist: float) -> None:
        while dist > 1e-9 and self._remaining:
            tx, ty = self._remaining[0]
            dx, dy = tx - self.x, ty - self.y
            seg_len = math.hypot(dx, dy)
            if seg_len < 1e-9:
                self._remaining.pop(0)
                continue
            self.theta = math.atan2(dy, dx)
            if dist >= seg_len:
                self.x, self.y = tx, ty
                dist -= seg_len
                self._remaining.pop(0)
            else:
                frac = dist / seg_len
                self.x += dx * frac
                self.y += dy * frac
                dist = 0.0
