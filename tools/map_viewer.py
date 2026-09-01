"""
맵 뷰어 (표시 전용)
==================

occupancy grid 맵과 로봇들을 world 좌표계(미터) 위에 그린다.

**이 모듈은 로봇을 시뮬레이션하지 않는다.** 운동학도 배터리도 여기 없다.
누군가 넘겨준 pose 를 그리기만 한다. 로봇의 실제 움직임은 별도 프로세스
(`robot_client/main.py`) 안에 있고, 뷰어는 그 프로세스가 보고한 값만 본다.

라이브로 보려면 `tools/fleet_monitor.py` 를 쓴다 (MQTT 구독 -> 이 뷰어).
이 파일을 직접 실행하면 맵 확인용 정적 뷰어가 된다.

    python tools/map_viewer.py maps/warehouse/warehouse.json
    python tools/map_viewer.py maps/warehouse/warehouse.json --robot 1.2,6,0 --path 1.2,6 1.2,2
    python tools/map_viewer.py maps/warehouse/warehouse.json --save out.png     # 창 없이 파일로

조작
    마우스 이동   좌표 / 셀 종류가 툴바에 표시됨
    드래그        화면 이동 (pan)
    스크롤        확대/축소 (커서 위치 기준)
    g             격자 토글
    c             색상 모드 토글 (분류색 <-> 원본 그레이스케일)
    t             계획 경로 토글
    r             확대/이동 초기화 (맵 전체 보기)
"""

from __future__ import annotations

import argparse
import math
import sys
from dataclasses import dataclass, field
from pathlib import Path
from typing import Iterable, Optional, Sequence

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

import matplotlib  # noqa: E402
import numpy as np  # noqa: E402

from common.map_model import CellType, MapModel  # noqa: E402

# classify_grid() 의 코드 순서와 맞춘다: 0 = OCCUPIED, 1 = UNKNOWN, 2 = FREE
CELL_COLORS = ["#1c1c1e", "#9aa0a6", "#ffffff"]
CELL_LABELS_KO = ["OCCUPIED (장애물)", "UNKNOWN (미탐사·주행불가)", "FREE (주행가능)"]
CELL_LABELS_EN = ["OCCUPIED (obstacle)", "UNKNOWN (unexplored/blocked)", "FREE (drivable)"]

PATH_COLOR = "#0090ff"

# 로봇 20대까지 서로 구분되는 색
ROBOT_PALETTE = [
    "#e5484d", "#0090ff", "#30a46c", "#f5a623", "#8e4ec6",
    "#e93d82", "#12a594", "#d6409f", "#f76b15", "#3e63dd",
    "#46a758", "#ab4aba", "#e54666", "#0d9488", "#ca8a04",
    "#7c3aed", "#dc2626", "#0284c7", "#65a30d", "#c026d3",
]

# 이 시간 동안 상태 보고가 없으면 끊긴 것으로 보고 흐리게 그린다
STALE_AFTER = 2.0

# 한글 라벨용 폰트 후보. 개발 환경(macOS)과 배포 환경(Ubuntu) 모두 커버한다.
# Ubuntu 에 없으면: sudo apt install fonts-nanum   또는   fonts-noto-cjk
KOREAN_FONT_CANDIDATES = [
    "AppleGothic", "Apple SD Gothic Neo",      # macOS
    "NanumGothic", "Nanum Gothic",             # fonts-nanum
    "Noto Sans CJK KR", "Noto Sans KR",        # fonts-noto-cjk
    "Malgun Gothic",                           # Windows
]


def setup_korean_font() -> Optional[str]:
    """
    한글이 깨지지 않게 matplotlib 기본 폰트를 잡는다.
    쓸 수 있는 폰트를 찾으면 이름을, 없으면 None 을 반환한다 (호출부가 영문 라벨로 대체).
    """
    import matplotlib.pyplot as plt
    from matplotlib import font_manager

    available = {f.name for f in font_manager.fontManager.ttflist}
    for name in KOREAN_FONT_CANDIDATES:
        if name in available:
            plt.rcParams["font.family"] = name
            plt.rcParams["axes.unicode_minus"] = False  # 한글 폰트는 유니코드 마이너스가 깨진다
            return name
    return None


@dataclass
class RobotView:
    """
    뷰어가 아는 로봇 하나. 전부 로봇이 보고한 값이며, 뷰어가 추측한 값은 없다.
    (last_seen 만 뷰어가 수신 시각으로 기록한다 — 끊김 판정용)
    """

    robot_id: str
    x: float = 0.0
    y: float = 0.0
    theta: float = 0.0
    velocity: float = 0.0
    width: float = 0.5      # 로봇 실제 폭 (m). 로봇이 State 로 보고한 값
    length: float = 0.7     # 로봇 실제 길이 (m)
    state: str = "?"
    battery: Optional[float] = None
    connection: str = "?"
    order_id: Optional[str] = None
    remaining_nodes: int = 0
    errors: list[str] = field(default_factory=list)
    last_seen: float = 0.0
    # 목적지까지 로봇이 스스로 계획한 남은 경로 (장애물 회피 포함). 지나온 길이 아니라
    # 앞으로 갈 길이다 — 로봇이 State.local_path 로 보고한 값 그대로
    planned_path: list[tuple[float, float]] = field(default_factory=list)

    # retain 된 connection 만 받고 state 는 아직 못 받은 로봇이 있다 (예: 죽은 뒤 남은
    # OFFLINE). 그런 로봇의 (0, 0) 을 진짜 위치처럼 그리면 안 된다
    has_pose: bool = False

    def is_stale(self, now: float) -> bool:
        return self.has_pose and (now - self.last_seen) > STALE_AFTER


class MapViewer:
    """맵 + 로봇들을 그리는 뷰어. 표시 전용이며 시뮬레이션을 하지 않는다."""

    def __init__(
        self,
        map_model: MapModel,
        grid_step: float = 1.0,
        classified: bool = True,
        show_panel: bool = True,
    ):
        self.map = map_model
        self.grid_step = grid_step
        self.classified = classified
        self.show_panel = show_panel

        self.robots: dict[str, RobotView] = {}
        self._path: list[tuple[float, float]] = []

        self._fig = None
        self._ax = None
        self._panel_text = None
        self._image = None
        self._robot_artists: dict[str, list] = {}
        self._planned_path_artists: dict[str, object] = {}
        self._path_artist = None
        self._show_grid = True
        self._show_planned_path = True
        self._korean_font = setup_korean_font()
        self._coord_text = None
        self._full_xlim: tuple[float, float] = (0.0, 1.0)
        self._full_ylim: tuple[float, float] = (0.0, 1.0)

        # 드래그 이동(pan) 상태
        self._panning = False
        self._pan_start_display: Optional[tuple[float, float]] = None
        self._pan_start_xlim: Optional[tuple[float, float]] = None
        self._pan_start_ylim: Optional[tuple[float, float]] = None

    # -- 로봇 데이터 갱신 (표시용 입력) --------------------------------------

    def update_robot(
        self,
        robot_id: str,
        x: float,
        y: float,
        theta: float = 0.0,
        *,
        velocity: float = 0.0,
        width: float = 0.5,
        length: float = 0.7,
        state: str = "?",
        battery: Optional[float] = None,
        order_id: Optional[str] = None,
        remaining_nodes: int = 0,
        errors: Optional[list[str]] = None,
        planned_path: Optional[list[tuple[float, float]]] = None,
        now: float = 0.0,
    ) -> None:
        """로봇이 보고한 값을 반영한다. 없던 로봇이면 새로 만든다."""
        view = self.robots.get(robot_id)
        if view is None:
            view = RobotView(robot_id=robot_id)
            self.robots[robot_id] = view
        view.x, view.y, view.theta = x, y, theta
        view.velocity = velocity
        view.width = width
        view.length = length
        view.state = state
        view.battery = battery
        view.order_id = order_id
        view.remaining_nodes = remaining_nodes
        view.errors = errors or []
        view.planned_path = list(planned_path) if planned_path else []
        view.last_seen = now
        view.has_pose = True

    def set_connection(self, robot_id: str, connection: str, now: float = 0.0) -> None:
        view = self.robots.get(robot_id)
        if view is None:
            view = RobotView(robot_id=robot_id, last_seen=now)
            self.robots[robot_id] = view
        view.connection = connection

    def remove_robot(self, robot_id: str) -> None:
        self.robots.pop(robot_id, None)
        for artist in self._robot_artists.pop(robot_id, []):
            artist.remove()
        planned = self._planned_path_artists.pop(robot_id, None)
        if planned is not None:
            planned.remove()

    def robot_color(self, robot_id: str) -> str:
        ordered = sorted(self.robots)
        idx = ordered.index(robot_id) if robot_id in ordered else 0
        return ROBOT_PALETTE[idx % len(ROBOT_PALETTE)]

    def set_path(self, waypoints: Iterable[Sequence[float]]) -> None:
        """참고용 경로(경유점) 오버레이. 로봇 주행과는 무관한 표시선이다."""
        self._path = [(float(p[0]), float(p[1])) for p in waypoints]
        if self._ax is not None:
            self._draw_path()

    # -- 렌더링 -----------------------------------------------------------

    def build(self):
        """figure 를 만들고 맵을 그린다. (fig, ax) 반환."""
        import matplotlib
        import matplotlib.pyplot as plt
        from matplotlib.colors import ListedColormap

        # 기본 네비게이션 툴바 제거 — 드래그/스크롤로 직접 이동·확대하므로 불필요
        matplotlib.rcParams["toolbar"] = "None"

        m = self.map.metadata
        xmin, xmax, ymin, ymax = self.map.world_extent
        # world y 는 이제 아래로 증가한다 (origin=좌상단, 뒤집지 않음). matplotlib 의
        # extent/ylim 세번째·네번째 값은 각각 "축 하단/상단"에 그대로 박히므로,
        # y 를 (ymax, ymin) 역순으로 줘야 화면 위쪽이 작은 y(원점 쪽)로 나온다
        extent = (xmin, xmax, ymax, ymin)
        self._full_xlim = (xmin, xmax)
        self._full_ylim = (ymax, ymin)

        aspect = m.width_m / m.height_m
        fig_w = min(16.0, max(8.0, 7.0 * aspect))
        fig_h = fig_w / aspect

        fig = plt.figure(figsize=(fig_w, fig_h))
        # 맵이 창 전체를 채운다. 정보는 그 위에 오버레이로만 얹는다 (별도 서브플롯 없음)
        ax = fig.add_axes([0.0, 0.0, 1.0, 1.0])
        # set_axis_off() 는 격자(grid)까지 통째로 숨긴다 — 눈금/라벨/테두리만
        # 개별로 지우고 축 자체는 켜 둔다 (격자를 그대로 쓰기 위해)
        ax.set_xticklabels([])
        ax.set_yticklabels([])
        ax.tick_params(length=0)
        for spine in ax.spines.values():
            spine.set_visible(False)
        self._fig, self._ax = fig, ax

        if self.classified:
            data = self.map.classify_grid()
            self._image = ax.imshow(
                data, cmap=ListedColormap(CELL_COLORS), vmin=0, vmax=2, extent=extent,
                origin="upper", interpolation="nearest",
            )
        else:
            self._image = ax.imshow(
                self.map.grid, cmap="gray", vmin=0, vmax=255, extent=extent,
                origin="upper", interpolation="nearest",
            )

        # adjustable="datalim" (기본값 "box" 아님) — "box" 는 종횡비를 맞추려고
        # 축(axes) 박스 자체를 줄여서 [0,0,1,1] 안에 다시 센터링해버린다. 창을
        # 최대화해서 종횡비가 바뀌면 그 결과가 딱 "가운데 작은 4:3 박스"로 보이는
        # 원인이다. "datalim" 은 박스 위치(꽉 채운 상태)는 그대로 두고 보이는
        # 데이터 범위 쪽을 늘려서 맞추므로 맵이 항상 창 전체를 채운다
        ax.set_aspect("equal", adjustable="datalim")
        ax.set_xlim(*self._full_xlim)
        ax.set_ylim(*self._full_ylim)
        self._apply_grid()
        self._add_title_overlay()
        self._add_legend()
        ax.plot(0, 0, "+", color="#ff3b30", markersize=12, markeredgewidth=2.0, zorder=9)
        if self.show_panel:
            self._panel_text = ax.text(
                0.995, 0.995, "", transform=ax.transAxes, va="top", ha="right",
                fontsize=8.5, family="monospace", color="white", zorder=20,
                bbox=dict(boxstyle="round,pad=0.4", facecolor="black", alpha=0.55, edgecolor="none"),
            )

        # 기본 툴바 상태바를 없앴으니 마우스 좌표는 직접 오버레이로 띄운다 (우측 하단)
        self._coord_text = ax.text(
            0.995, 0.005, "", transform=ax.transAxes, va="bottom", ha="right",
            fontsize=8.5, family="monospace", color="white", zorder=20,
            bbox=dict(boxstyle="round,pad=0.35", facecolor="black", alpha=0.55, edgecolor="none"),
        )
        fig.canvas.mpl_connect("scroll_event", self._on_scroll)
        fig.canvas.mpl_connect("button_press_event", self._on_press)
        fig.canvas.mpl_connect("button_release_event", self._on_release)
        fig.canvas.mpl_connect("motion_notify_event", self._on_motion)
        fig.canvas.mpl_connect("key_press_event", self._on_key)

        self._draw_path()
        self.refresh()
        return fig, ax

    def refresh(self, now: float = 0.0) -> None:
        """로봇 표시를 현재 데이터로 다시 그린다."""
        if self._ax is None:
            return
        self._draw_robots(now)
        self._draw_panel(now)
        self._fig.canvas.draw_idle()

    def show(self) -> None:
        import matplotlib.pyplot as plt

        if self._fig is None:
            self.build()
        print(self._usage_text())
        plt.show()

    def save(self, path: str | Path, dpi: int = 150) -> Path:
        if self._fig is None:
            self.build()
        path = Path(path)
        self._fig.savefig(path, dpi=dpi, bbox_inches="tight")
        return path

    # -- 내부: 그리기 ------------------------------------------------------

    def _apply_grid(self) -> None:
        """격자 on/off 만 다룬다 — 확대/이동 상태(xlim/ylim)는 건드리지 않는다."""
        ax = self._ax
        if self._show_grid and self.grid_step > 0:
            x0, x1, y0, y1 = self.map.world_extent
            ax.set_xticks(np.arange(math.floor(x0), x1 + self.grid_step, self.grid_step))
            ax.set_yticks(np.arange(math.floor(y0), y1 + self.grid_step, self.grid_step))
            ax.grid(True, color="#0090ff", alpha=0.25, linewidth=0.6)
        else:
            ax.grid(False)

    def _reset_view(self) -> None:
        self._ax.set_xlim(*self._full_xlim)
        self._ax.set_ylim(*self._full_ylim)
        self._fig.canvas.draw_idle()

    def _add_title_overlay(self) -> None:
        m = self.map.metadata
        self._ax.text(
            0.005, 0.995,
            f"{m.name}  —  {m.width_m:g} x {m.height_m:g} m  @ {m.resolution:g} m/px",
            transform=self._ax.transAxes, va="top", ha="left", fontsize=9,
            color="white", zorder=20,
            bbox=dict(boxstyle="round,pad=0.35", facecolor="black", alpha=0.55, edgecolor="none"),
        )

    def _add_legend(self) -> None:
        from matplotlib.patches import Patch

        if not self.classified:
            return
        labels = CELL_LABELS_KO if self._korean_font else CELL_LABELS_EN
        handles = [
            Patch(facecolor=c, edgecolor="#888", label=l)
            for c, l in zip(CELL_COLORS, labels)
        ]
        self._ax.legend(
            handles=handles, loc="lower left", framealpha=0.55,
            facecolor="black", labelcolor="white", fontsize=8, borderpad=0.6,
        )

    def _draw_robots(self, now: float) -> None:
        from matplotlib.patches import Rectangle
        from matplotlib.transforms import Affine2D

        for artists in self._robot_artists.values():
            for a in artists:
                a.remove()
        self._robot_artists = {}

        for robot_id, view in sorted(self.robots.items()):
            if not view.has_pose:
                continue  # 접속만 알고 pose 를 아직 못 받은 로봇
            color = self.robot_color(robot_id)
            stale = view.is_stale(now)
            alpha = 0.3 if stale else 0.92
            size = max(view.width, view.length)

            # 로봇 실제 크기(width x length)로 그린다. length 축이 진행방향(theta) 이다
            body = Rectangle(
                (-view.length / 2, -view.width / 2), view.length, view.width,
                facecolor=color, edgecolor="white", linewidth=1.5, alpha=alpha, zorder=5,
            )
            body.set_transform(
                Affine2D().rotate(view.theta).translate(view.x, view.y) + self._ax.transData
            )
            self._ax.add_patch(body)
            heading = self._ax.arrow(
                view.x, view.y,
                math.cos(view.theta) * view.length * 0.9,
                math.sin(view.theta) * view.length * 0.9,
                head_width=size * 0.5, head_length=size * 0.4,
                fc=color, ec="white", linewidth=1.0, alpha=alpha,
                zorder=6, length_includes_head=True,
            )
            label = self._ax.annotate(
                robot_id.replace("AMR-", ""),
                (view.x, view.y), xytext=(0, size * 34),
                textcoords="offset points", ha="center", fontsize=7.5,
                color="white", alpha=alpha, zorder=7,
                bbox=dict(boxstyle="round,pad=0.18", facecolor=color, edgecolor="none", alpha=alpha),
            )
            self._robot_artists[robot_id] = [body, heading, label]

            self._update_planned_path(robot_id, view, color, stale)

        # 사라진 로봇의 경로선 정리
        for robot_id in [r for r in self._planned_path_artists if r not in self.robots]:
            self._planned_path_artists.pop(robot_id).remove()

    def _update_planned_path(self, robot_id: str, view: RobotView, color: str, stale: bool) -> None:
        """목적지까지 남은 계획 경로. 지나온 궤적이 아니라 앞으로 갈 경로다."""
        line = self._planned_path_artists.get(robot_id)
        if not self._show_planned_path or len(view.planned_path) < 1:
            if line is not None:
                line.set_data([], [])
            return
        xs = [view.x] + [p[0] for p in view.planned_path]
        ys = [view.y] + [p[1] for p in view.planned_path]
        if line is None:
            (line,) = self._ax.plot([], [], linewidth=1.5, linestyle="--", zorder=3)
            self._planned_path_artists[robot_id] = line
        line.set_data(xs, ys)
        line.set_color(color)
        line.set_alpha(0.25 if stale else 0.7)

    def _draw_panel(self, now: float) -> None:
        if self._panel_text is None:
            return
        # 정렬을 위해 monospace 로 그리는데 한글 글리프가 없는 폰트가 많다.
        # 이 패널만 ASCII 로 유지한다 (맵 라벨·범례는 한글 그대로)
        if not self.robots:
            self._panel_text.set_text("no robots\n\nstart a robot process\nand it shows up here.")
            return

        live = sum(1 for v in self.robots.values() if v.has_pose)
        lines = [f"ROBOTS: {live} reporting / {len(self.robots)} known", ""]
        for robot_id, view in sorted(self.robots.items()):
            if not view.has_pose:
                # state 를 한 번도 못 받았다. 아는 건 브로커에 남은 connection 뿐이다
                lines.append(f"- {robot_id}  (no state)")
                lines.append(f"    conn {view.connection}")
                lines.append("")
                continue
            stale = view.is_stale(now)
            mark = "x" if stale else "*"
            battery = f"{view.battery:5.1f}%" if view.battery is not None else "    ?"
            lines.append(f"{mark} {robot_id}  {view.state}")
            lines.append(f"    xy   {view.x:7.2f}, {view.y:6.2f}")
            lines.append(f"    th   {view.theta:7.3f} rad")
            lines.append(f"    v    {view.velocity:7.2f} m/s")
            lines.append(f"    bat  {battery}")
            if view.order_id:
                lines.append(f"    ord  {view.order_id} ({view.remaining_nodes} left)")
            if view.connection not in ("ONLINE", "?"):
                lines.append(f"    conn {view.connection}")
            if view.errors:
                lines.append(f"    ERR  {', '.join(view.errors)}")
            if stale:
                lines.append(f"    (no report for {now - view.last_seen:.0f}s)")
            lines.append("")
        self._panel_text.set_text("\n".join(lines))

    def _draw_path(self) -> None:
        if self._path_artist is not None:
            self._path_artist.remove()
            self._path_artist = None
        if not self._path:
            return
        xs = [p[0] for p in self._path]
        ys = [p[1] for p in self._path]
        (self._path_artist,) = self._ax.plot(
            xs, ys, "-o", color=PATH_COLOR, linewidth=2.0, markersize=5,
            markerfacecolor="white", zorder=4,
        )

    # -- 내부: 이벤트 ------------------------------------------------------

    def _format_coord(self, x: float, y: float) -> str:
        # coord_text 는 monospace 폰트라 한글 글리프가 없을 수 있다 (패널과 같은 이유로
        # ASCII 로 대체). 맵 범례처럼 한글 폰트 지원 여부에 따라 갈린다
        if not self.map.in_bounds(x, y):
            return f"x={x:7.2f} m  y={y:7.2f} m   [out of bounds]"
        px, py = self.map.world_to_pixel(x, y)
        cell = self.map.cell_at(x, y)
        grey = int(self.map.grid[py, px])
        mark = "drivable" if cell.drivable else "blocked"
        return (
            f"x={x:7.2f} m  y={y:7.2f} m   px=({px},{py})  "
            f"grey={grey:3d}  {cell.value} ({mark})"
        )

    def _on_scroll(self, event) -> None:
        """마우스 커서 위치를 기준으로 확대/축소한다."""
        if event.inaxes is not self._ax or event.xdata is None:
            return
        scale = 1 / 1.15 if event.button == "up" else 1.15
        ax = self._ax
        x0, x1 = ax.get_xlim()
        y0, y1 = ax.get_ylim()
        xdata, ydata = event.xdata, event.ydata
        new_w = (x1 - x0) * scale
        new_h = (y1 - y0) * scale
        relx = (x1 - xdata) / (x1 - x0)
        rely = (y1 - ydata) / (y1 - y0)
        ax.set_xlim(xdata - new_w * (1 - relx), xdata + new_w * relx)
        ax.set_ylim(ydata - new_h * (1 - rely), ydata + new_h * rely)
        self._fig.canvas.draw_idle()

    def _on_press(self, event) -> None:
        """왼쪽 버튼 드래그로 화면 이동(pan)을 시작한다."""
        if event.inaxes is not self._ax or event.button != 1:
            return
        toolbar = getattr(self._fig.canvas, "toolbar", None)
        if toolbar is not None and getattr(toolbar, "mode", ""):
            return  # 툴바 확대/이동 도구가 켜져 있으면 겹쳐서 처리하지 않는다
        self._panning = True
        self._pan_start_display = (event.x, event.y)
        self._pan_start_xlim = self._ax.get_xlim()
        self._pan_start_ylim = self._ax.get_ylim()

    def _on_motion(self, event) -> None:
        self._update_coord_text(event)

        if not self._panning or event.x is None or event.y is None:
            return
        ax = self._ax
        bbox = ax.get_window_extent()
        x0, x1 = self._pan_start_xlim
        y0, y1 = self._pan_start_ylim
        dx = (event.x - self._pan_start_display[0]) * (x1 - x0) / bbox.width
        dy = (event.y - self._pan_start_display[1]) * (y1 - y0) / bbox.height
        ax.set_xlim(x0 - dx, x1 - dx)
        ax.set_ylim(y0 - dy, y1 - dy)
        self._fig.canvas.draw_idle()

    def _update_coord_text(self, event) -> None:
        """기본 툴바 상태바가 없으니 마우스 아래 좌표를 우측 하단 오버레이에 직접 띄운다."""
        if self._coord_text is None:
            return
        if event.inaxes is not self._ax or event.xdata is None:
            text = ""
        else:
            text = self._format_coord(event.xdata, event.ydata)
        if text != self._coord_text.get_text():
            self._coord_text.set_text(text)
            self._fig.canvas.draw_idle()

    def _on_release(self, event) -> None:
        self._panning = False
        self._pan_start_display = None

    def _on_key(self, event) -> None:
        if event.key == "g":
            self._show_grid = not self._show_grid
            self._apply_grid()
            self._fig.canvas.draw_idle()
        elif event.key == "c":
            self.classified = not self.classified
            self._rebuild_image()
        elif event.key == "t":
            self._show_planned_path = not self._show_planned_path
            self.refresh()
        elif event.key == "r":
            self._reset_view()

    def _rebuild_image(self) -> None:
        from matplotlib.colors import ListedColormap

        if self.classified:
            self._image.set_data(self.map.classify_grid())
            self._image.set_cmap(ListedColormap(CELL_COLORS))
            self._image.set_clim(0, 2)
        else:
            self._image.set_data(self.map.grid)
            self._image.set_cmap("gray")
            self._image.set_clim(0, 255)
        self._fig.canvas.draw_idle()

    @staticmethod
    def _usage_text() -> str:
        return (
            "조작: 드래그=이동 / 스크롤=확대·축소 / g=격자 / c=색상모드 / t=계획경로 / r=보기 초기화\n"
            "      마우스를 움직이면 창 하단에 좌표·셀 종류가 표시된다.\n"
        )


# =============================================================================
# CLI — 맵 확인용 정적 뷰어. 라이브 관측은 tools/fleet_monitor.py 를 쓸 것.
# =============================================================================

def parse_xy(text: str) -> tuple[float, float]:
    parts = text.replace(",", " ").split()
    if len(parts) != 2:
        raise argparse.ArgumentTypeError(f"'{text}' 는 x,y 형식이 아니다")
    return float(parts[0]), float(parts[1])


def parse_pose(text: str) -> tuple[float, float, float]:
    parts = text.replace(",", " ").split()
    if len(parts) not in (2, 3):
        raise argparse.ArgumentTypeError(f"'{text}' 는 x,y[,theta] 형식이 아니다")
    theta = float(parts[2]) if len(parts) == 3 else 0.0
    return float(parts[0]), float(parts[1]), theta


def main() -> None:
    parser = argparse.ArgumentParser(
        description="occupancy grid 맵 뷰어 (정적). 라이브 관측은 tools/fleet_monitor.py"
    )
    parser.add_argument("metadata", help="맵 메타데이터 JSON 경로 (예: maps/warehouse/warehouse.json)")
    parser.add_argument("--robot", type=parse_pose, metavar="X,Y[,THETA]",
                        help="로봇 위치 표시 (정적). theta 는 라디안")
    parser.add_argument("--path", type=parse_xy, nargs="+", metavar="X,Y",
                        help="참고용 경로 표시")
    parser.add_argument("--grid-step", type=float, default=1.0,
                        help="격자 간격 (m). 0 이면 격자 없음")
    parser.add_argument("--raw", action="store_true",
                        help="분류색 대신 원본 그레이스케일로 표시")
    parser.add_argument("--save", metavar="PNG", help="창을 띄우지 않고 파일로 저장")
    args = parser.parse_args()

    if args.save:
        matplotlib.use("Agg")

    viewer = MapViewer(
        MapModel.load(args.metadata),
        grid_step=args.grid_step,
        classified=not args.raw,
        show_panel=False,
    )
    viewer.build()

    if args.robot:
        x, y, theta = args.robot
        viewer.update_robot("STATIC", x, y, theta, state="(정적 표시)")
        viewer.refresh()
    if args.path:
        viewer.set_path(args.path)

    if args.save:
        print(f"저장: {viewer.save(args.save)}")
    else:
        viewer.show()


if __name__ == "__main__":
    main()
