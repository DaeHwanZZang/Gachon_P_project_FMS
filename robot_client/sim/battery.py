"""
배터리 시뮬레이션
================

로봇 클라이언트 내부에만 존재한다. FMS 는 이 모듈을 import 하지 않고,
로봇이 State 메시지로 보고하는 BatteryInfo 만 본다.

소모 모델은 단순하다:
    소모율(%/s) = idle_drain + move_drain * (속도 / 최대속도)

정밀한 모델링이 목적이 아니라, 충전 스케줄링 알고리즘을 검증할 수 있을 만큼의
"시간에 따라 줄어들고 충전하면 늘어나는" 동작을 제공하는 게 목적이다.
"""

from __future__ import annotations

from common.schemas import BatteryInfo


class BatterySim:
    """단순 선형 방전/충전 모델."""

    def __init__(
        self,
        charge: float = 100.0,
        idle_drain: float = 0.01,      # %/s, 정지 상태에서도 나가는 기본 소모
        move_drain: float = 0.05,      # %/s, 최대 속도로 달릴 때 추가되는 소모
        charge_rate: float = 1.0,      # %/s, 충전 속도
        max_velocity: float = 1.2,     # move_drain 을 정규화할 기준 속도
    ):
        self.charge = max(0.0, min(100.0, charge))
        self.idle_drain = idle_drain
        self.move_drain = move_drain
        self.charge_rate = charge_rate
        self.max_velocity = max_velocity
        self.charging = False
        self._last_drain = idle_drain

    def step(self, dt: float, velocity: float = 0.0) -> None:
        if self.charging:
            self.charge = min(100.0, self.charge + self.charge_rate * dt)
            self._last_drain = 0.0
            return

        speed_ratio = min(1.0, abs(velocity) / self.max_velocity) if self.max_velocity > 0 else 0.0
        drain = self.idle_drain + self.move_drain * speed_ratio
        self._last_drain = drain
        self.charge = max(0.0, self.charge - drain * dt)

    @property
    def estimated_runtime(self) -> float | None:
        """현재 소모율 기준 잔여 가동 시간(초). 충전 중이면 None."""
        if self.charging or self._last_drain <= 0:
            return None
        return self.charge / self._last_drain

    def info(self) -> BatteryInfo:
        """State 메시지에 실을 배터리 정보."""
        return BatteryInfo(
            charge=round(self.charge, 2),
            charging=self.charging,
            estimated_runtime=self.estimated_runtime,
        )
