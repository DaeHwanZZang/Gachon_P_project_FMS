"""
로컬 경로계획 (A*)
==================

FMS 는 이제 목적지 좌표만 보낸다. 장애물을 피해 거기까지 가는 경로는
로봇이 자기 맵으로 스스로 짠다 (실물 로봇의 로컬 플래너와 같은 위치).

FMS 쪽 경로계획(다중 로봇 트래픽 조정)과는 별개다. 여기서는 딱 "이 지도에서
A 에서 B 까지 안 부딪히고 가는 점렬" 만 만든다. 그래서 map_model 만 있으면
동작하고, 다른 로봇 상태는 전혀 모른다.

로봇 자신의 맵 밖에서 FMS 가 절대 이 모듈을 import 하지 않는다
(CLAUDE.md 원칙 3 — FMS 는 로봇이 시뮬레이터인지 실물인지 몰라야 한다.
경로계획도 로봇마다 알아서 하는 로컬 동작이다).
"""

from __future__ import annotations

import heapq
import math
from typing import TYPE_CHECKING, Optional

if TYPE_CHECKING:
    import numpy as np

    from common.map_model import MapModel


class PlanningError(Exception):
    """시작점/목적지가 막혀 있거나 경로를 찾지 못했을 때."""


# 8방향 이동 (dx, dy, cost)
_NEIGHBORS = [
    (1, 0, 1.0), (-1, 0, 1.0), (0, 1, 1.0), (0, -1, 1.0),
    (1, 1, math.sqrt(2)), (1, -1, math.sqrt(2)),
    (-1, 1, math.sqrt(2)), (-1, -1, math.sqrt(2)),
]

# 맵당 팽창(inflation) 격자 캐시. 정적인 맵을 매 replan 마다 다시 계산하지 않기 위함
# (id(map_model), radius_px) -> 팽창된 grid. 맵은 런타임에 안 바뀐다고 가정한다
_inflation_cache: dict[tuple[int, int], "np.ndarray"] = {}


def plan_path(
    map_model: "MapModel",
    start: tuple[float, float],
    goal: tuple[float, float],
    robot_width: float = 0.5,
    robot_length: float = 0.7,
) -> list[tuple[float, float]]:
    """
    start -> goal 로 가는, 장애물을 피하는 world 좌표 점렬을 돌려준다.
    start 자체는 포함하지 않는다 (거기서부터 출발하니까).

    로봇 몸체(robot_width x robot_length)가 지나갈 수 있도록 장애물을
    robot_width/2 만큼 부풀린 뒤 그 위에서 A* 를 돌린다 (config-space 접근).
    이동 방향이 통로를 따라 정렬된다고 가정하고 폭(짧은 변)만 기준으로 삼는다
    — robot_length 는 지금은 코너링 여유를 더 두고 싶을 때 쓸 여지로만 남겨둔다.
    그 결과 로봇 폭보다 좁은 통로는 애초에 격자에서 막혀서 경로가 생성되지 않는다.

    격자 A* 로 찾은 뒤, 꺾이는 점만 남기고 나머지는 지운다
    (line-of-sight 로 직선으로 가도 안전하면 중간 점 불필요 — 격자 계단현상 방지).
    """
    if map_model is None:
        raise PlanningError("맵이 없으면 경로계획을 할 수 없다")

    if not map_model.is_free(*start):
        raise PlanningError(f"시작점 {start} 이 주행 불가 셀이다")
    if not map_model.is_free(*goal):
        raise PlanningError(f"목적지 {goal} 이 주행 불가 셀이다")

    start_px = map_model.world_to_pixel(*start)
    goal_px = map_model.world_to_pixel(*goal)

    if start_px == goal_px:
        return [goal]

    radius_px = math.ceil((robot_width / 2) / map_model.metadata.resolution)
    inflated = _get_inflated_grid(map_model, radius_px)
    height_px, width_px = inflated.shape

    def drivable(px: int, py: int) -> bool:
        return 0 <= px < width_px and 0 <= py < height_px and inflated[py, px] == 2

    # 시작/목적지 칸이 벽에 바짝 붙어 있어 팽창 때문에 "막힘" 으로 잡히더라도
    # 그 두 칸만은 예외로 통행 가능 취급한다 (원본 맵에서는 FREE 였던 칸이다 —
    # 안 그러면 로봇은 한 발짝도 못 떼거나, 도킹 지점처럼 벽에 붙은 목적지에 못 간다).
    # 팽창은 "지나가는 길" 폭을 보장하는 것이지 정지 지점까지 막을 이유는 없다
    if not drivable(*start_px) or not drivable(*goal_px):
        inflated = inflated.copy()
        inflated[start_px[1], start_px[0]] = 2
        inflated[goal_px[1], goal_px[0]] = 2

    path_px = _astar(start_px, goal_px, drivable, width_px, height_px)
    if path_px is None:
        raise PlanningError(
            f"{start} -> {goal} 경로를 찾지 못함 (막혀 있거나 로봇 폭 {robot_width}m 보다 좁은 통로뿐)"
        )

    waypoints_px = _simplify(path_px, drivable)
    waypoints = [map_model.pixel_to_world(px, py) for px, py in waypoints_px]
    # 목적지는 격자 중심이 아니라 요청받은 정확한 좌표로 맞춘다
    if waypoints:
        waypoints[-1] = goal
    else:
        waypoints = [goal]
    return waypoints


def _get_inflated_grid(map_model: "MapModel", radius_px: int) -> "np.ndarray":
    key = (id(map_model), radius_px)
    cached = _inflation_cache.get(key)
    if cached is not None:
        return cached
    inflated = _inflate(map_model.classify_grid(), radius_px)
    _inflation_cache[key] = inflated
    return inflated


def _inflate(grid: "np.ndarray", radius_px: int) -> "np.ndarray":
    """
    FREE(2) 가 아닌 칸(OCCUPIED/UNKNOWN) 주변 radius_px 이내를 전부 막힌 것으로 만든다.
    로봇 몸체 절반 크기만큼 장애물을 부풀리는 것과 같다 (Minkowski 팽창).
    """
    import numpy as np

    if radius_px <= 0:
        return grid.copy()

    blocked = grid != 2
    result = blocked.copy()
    h, w = blocked.shape
    for dy in range(-radius_px, radius_px + 1):
        for dx in range(-radius_px, radius_px + 1):
            if dx == 0 and dy == 0 or dx * dx + dy * dy > radius_px * radius_px:
                continue
            shifted = np.zeros_like(blocked)
            src_y0, src_y1 = max(0, dy), h + min(0, dy)
            src_x0, src_x1 = max(0, dx), w + min(0, dx)
            dst_y0, dst_y1 = max(0, -dy), h + min(0, -dy)
            dst_x0, dst_x1 = max(0, -dx), w + min(0, -dx)
            shifted[dst_y0:dst_y1, dst_x0:dst_x1] = blocked[src_y0:src_y1, src_x0:src_x1]
            result |= shifted

    inflated_grid = grid.copy()
    inflated_grid[result] = 0
    return inflated_grid


def _astar(
    start: tuple[int, int],
    goal: tuple[int, int],
    drivable,
    width_px: int,
    height_px: int,
) -> Optional[list[tuple[int, int]]]:
    def heuristic(a: tuple[int, int], b: tuple[int, int]) -> float:
        return math.hypot(a[0] - b[0], a[1] - b[1])

    open_heap: list[tuple[float, tuple[int, int]]] = [(0.0, start)]
    came_from: dict[tuple[int, int], tuple[int, int]] = {}
    g_score: dict[tuple[int, int], float] = {start: 0.0}
    visited: set[tuple[int, int]] = set()

    while open_heap:
        _, current = heapq.heappop(open_heap)
        if current in visited:
            continue
        visited.add(current)

        if current == goal:
            path = [current]
            while current in came_from:
                current = came_from[current]
                path.append(current)
            path.reverse()
            return path

        cx, cy = current
        for dx, dy, cost in _NEIGHBORS:
            nx, ny = cx + dx, cy + dy
            neighbor = (nx, ny)
            if neighbor in visited or not drivable(nx, ny):
                continue
            # 대각 이동이 양쪽 벽 모서리를 스치고 지나가는 것(코너 컷팅)을 막는다
            if dx != 0 and dy != 0 and not (drivable(cx + dx, cy) and drivable(cx, cy + dy)):
                continue
            tentative = g_score[current] + cost
            if tentative < g_score.get(neighbor, math.inf):
                came_from[neighbor] = current
                g_score[neighbor] = tentative
                heapq.heappush(open_heap, (tentative + heuristic(neighbor, goal), neighbor))

    return None


def _line_of_sight(a: tuple[int, int], b: tuple[int, int], drivable) -> bool:
    """a, b 픽셀 사이를 직선으로 가도 전부 주행 가능 셀인지 (Bresenham)."""
    x0, y0 = a
    x1, y1 = b
    dx, dy = abs(x1 - x0), abs(y1 - y0)
    sx = 1 if x0 < x1 else -1
    sy = 1 if y0 < y1 else -1
    err = dx - dy
    x, y = x0, y0
    while (x, y) != (x1, y1):
        if not drivable(x, y):
            return False
        e2 = 2 * err
        if e2 > -dy:
            err -= dy
            x += sx
        if e2 < dx:
            err += dx
            y += sy
    return drivable(x1, y1)


def _simplify(path_px: list[tuple[int, int]], drivable) -> list[tuple[int, int]]:
    """직선으로 가도 안전한 중간 점들을 지운다 (격자 계단현상 -> 실제 꺾이는 점만)."""
    if len(path_px) <= 2:
        return path_px
    result = [path_px[0]]
    anchor = 0
    i = 1
    while i < len(path_px) - 1:
        if not _line_of_sight(path_px[anchor], path_px[i + 1], drivable):
            result.append(path_px[i])
            anchor = i
        i += 1
    result.append(path_px[-1])
    return result
