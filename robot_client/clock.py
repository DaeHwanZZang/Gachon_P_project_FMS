"""
시계 추상화
==========

로봇 코드가 time.sleep() 을 직접 부르지 않도록 시계를 한 겹 감싼다.
지금은 실시간 모드만 쓰지만, 나중에 가속 시뮬레이션이나 FMS 의 clock tick
브로드캐스트를 도입할 때 이 인터페이스만 갈아끼우면 되게 하려는 것이다.

프로세스 모델이 "로봇 1대 = 프로세스 1개" 라서 전역 시계를 공유할 수 없다.
그래서 현재는 RealtimeClock 만 존재한다 (CLAUDE.md 아키텍처 결정 4번).
"""

from __future__ import annotations

import time
from typing import Iterator, Protocol


class Clock(Protocol):
    """로봇이 보는 시간. 구현체를 바꾸면 시간 흐름을 바꿀 수 있다."""

    def now(self) -> float:
        """현재 시각(초)."""
        ...

    def sleep(self, seconds: float) -> None:
        """지정 시간만큼 대기."""
        ...


class RealtimeClock:
    """벽시계 그대로 쓰는 실시간 모드."""

    def now(self) -> float:
        return time.time()

    def sleep(self, seconds: float) -> None:
        if seconds > 0:
            time.sleep(seconds)


class Ticker:
    """
    일정 주기로 도는 루프. 처리 시간이 들쭉날쭉해도 누적 오차가 생기지 않도록
    다음 깨어날 시각을 기준으로 대기한다.

        ticker = Ticker(clock, period=0.05)
        for dt in ticker:
            executor.tick(dt)
    """

    def __init__(self, clock: Clock, period: float):
        if period <= 0:
            raise ValueError("period 는 0보다 커야 한다")
        self.clock = clock
        self.period = period
        self._running = True

    def stop(self) -> None:
        self._running = False

    def __iter__(self) -> Iterator[float]:
        next_at = self.clock.now() + self.period
        last = self.clock.now()
        while self._running:
            self.clock.sleep(next_at - self.clock.now())
            now = self.clock.now()
            dt = now - last
            last = now
            next_at += self.period
            # 한참 밀렸으면 따라잡기를 포기하고 기준을 다시 잡는다 (spiral of death 방지)
            if next_at < now:
                next_at = now + self.period
            yield dt
