"""
로봇 제어판 (디버그/테스트용 GUI)
================================

`robot_client.main --gui` 로 띄우는 로컬 조작 창. 표준 Tk 위젯만 쓴다
(추가 의존성 없음 — 헤드리스 컨테이너에서 --gui 없이 돌릴 땐 이 모듈을
아예 import 하지 않는다, `main.py` 참고).

**이건 FMS 가 아니다.** 실물 로봇이 아직 없으니 그 자리를 대신하는 개발 툴이다.
MQTT 프로토콜을 타지 않고 같은 프로세스 안에서 커맨드 큐를 통해 tick 루프에
바로 꽂는다 (main.py 의 `_dispatch_gui_command` 가 실제 실행부).

창은 두 부분으로 나뉜다.

    물리 패널   실물 로봇 몸체에 붙어 있는 버튼들의 재현.
                start / stop(더미) / e-stop 래치 / auto-manual 토글
    디버그      실물에 없는 기능. 강제 재위치, 설정 변경, 좌표 이동 명령

스레드 모델
-----------
Tk 는 메인 스레드에서 돌아야 안전하다(특히 macOS). 그래서 로봇의 tick/MQTT
루프를 백그라운드 스레드로 보내고, 이 GUI 가 메인 스레드에서 `mainloop()`
를 돈다. 정보 패널은 `executor` 를 락 없이 직접 읽는다 — 표시 전용이고
0.2초마다 갱신되니 아주 드물게 값이 살짝 어긋나 보여도 다음 틱에 바로
정정된다 (디버그 패널이라 감수할 수준).

설정 변경(ID/크기/속도/가속도)은 IDLE 상태에서만 허용한다 — 이동/대기/충전
중에 로봇 몸체 크기나 속도가 바뀌면 이미 계획된 경로·트래픽 통행권 전제가
깨진다. GUI 쪽에서 IDLE 아니면 입력칸을 비활성화하고, tick 루프 쪽
(`main.py`)에서도 다시 한번 상태를 확인한다 (레이스 방지용 이중 방어).

창 내용 전체가 Canvas + Scrollbar 로 감싸인 스크롤 영역 안에 들어간다 — 창을
줄이거나 섹션이 늘어나도 아래 내용이 잘리지 않고 스크롤로 접근할 수 있다.
"""

from __future__ import annotations

import queue
import sys
import tkinter as tk
from pathlib import Path
from tkinter import ttk
from typing import TYPE_CHECKING, Callable, Optional

from common.schemas import OperatingMode, RobotState, State

if TYPE_CHECKING:
    from robot_client.executor import OrderExecutor

REFRESH_MS = 200

_ICON_PATH = Path(__file__).resolve().parent.parent / "assets" / "icons" / "robot_gui_icon.png"

# -- 색 팔레트 (라이트 테마) -------------------------------------------------
BG = "#f2f3f6"
PANEL_BG = "#ffffff"
BORDER = "#d7dae1"
FG = "#20222b"
FG_DIM = "#6b7080"
ACCENT = "#2f6fed"
DANGER = "#d92d43"
ESTOP_IDLE = "#c23b52"
ESTOP_ON = "#e0263f"

STATE_COLORS = {
    RobotState.IDLE: "#5b6070",
    RobotState.BUSY: "#2f6fed",
    RobotState.CHARGING: "#0f9d84",
    RobotState.ERROR: "#d92d43",
    RobotState.EMERGENCY: "#e0263f",
}


def _set_macos_menu_bar_name(name: str) -> None:
    """macOS 메뉴바(애플 로고 옆)에 뜨는 앱 이름을 바꾼다. pyobjc 없으면 조용히 넘어감."""
    if sys.platform != "darwin":
        return
    try:
        from Foundation import NSBundle
        info = NSBundle.mainBundle().localizedInfoDictionary() or NSBundle.mainBundle().infoDictionary()
        if info is not None:
            info["CFBundleName"] = name
    except Exception:
        pass


def _setup_style(root: tk.Tk) -> None:
    root.configure(bg=BG)
    style = ttk.Style(root)
    try:
        style.theme_use("clam")
    except tk.TclError:
        pass

    style.configure(".", background=BG, foreground=FG, fieldbackground=PANEL_BG,
                     bordercolor=BORDER, font=("Helvetica", 10))
    style.configure("TFrame", background=BG)
    style.configure("Panel.TFrame", background=PANEL_BG)
    style.configure("TLabel", background=BG, foreground=FG)
    style.configure("Panel.TLabel", background=PANEL_BG, foreground=FG)
    style.configure("Dim.TLabel", background=PANEL_BG, foreground=FG_DIM)
    style.configure("Value.TLabel", background=PANEL_BG, foreground=FG,
                     font=("Menlo", 11))
    style.configure("Header.TLabel", background=BG, foreground=FG,
                     font=("Helvetica", 15, "bold"))
    style.configure("Section.TLabelframe", background=PANEL_BG, bordercolor=BORDER,
                     relief="solid", borderwidth=1)
    style.configure("Section.TLabelframe.Label", background=BG, foreground=FG_DIM,
                     font=("Helvetica", 9, "bold"))
    style.configure("Hint.TLabel", background=PANEL_BG, foreground=FG_DIM,
                     font=("Helvetica", 8))
    style.configure("Warn.TLabel", background=PANEL_BG, foreground=DANGER,
                     font=("Helvetica", 8))
    style.configure("TEntry", fieldbackground=PANEL_BG, foreground=FG,
                     insertcolor=FG, bordercolor=BORDER)
    style.map("TEntry", fieldbackground=[("disabled", "#eceef2")],
              foreground=[("disabled", FG_DIM)])
    style.configure("Accent.TButton", background=ACCENT, foreground="#ffffff",
                     font=("Helvetica", 10, "bold"), padding=6)
    style.map("Accent.TButton", background=[("disabled", "#c3cad6"), ("active", "#5089f5")],
              foreground=[("disabled", "#ffffff")])
    style.configure("Panel.TButton", background=BORDER, foreground=FG,
                     font=("Helvetica", 10), padding=6)
    style.map("Panel.TButton", background=[("active", "#c7cbd4")])
    # e-stop 은 래치형이라 눌린 상태가 색으로 유지돼야 한다
    style.configure("Estop.TButton", background=ESTOP_IDLE, foreground="#ffffff",
                     font=("Helvetica", 11, "bold"), padding=10)
    style.map("Estop.TButton", background=[("active", "#a82c44")])
    style.configure("EstopOn.TButton", background=ESTOP_ON, foreground="#ffffff",
                     font=("Helvetica", 11, "bold"), padding=10)
    style.map("EstopOn.TButton", background=[("active", "#f04a63")])
    style.configure("Mode.TRadiobutton", background=PANEL_BG, foreground=FG,
                     font=("Helvetica", 10))
    style.map("Mode.TRadiobutton", background=[("active", PANEL_BG)],
              foreground=[("disabled", FG_DIM)])
    style.configure("TSeparator", background=BORDER)


def _section(root: tk.Widget, text: str) -> ttk.Labelframe:
    frame = ttk.Labelframe(root, text=f"  {text}  ", style="Section.TLabelframe")
    frame.pack(fill="x", padx=10, pady=6)
    inner = ttk.Frame(frame, style="Panel.TFrame")
    inner.pack(fill="x", padx=10, pady=8)
    return inner


def _build_scroll_area(root: tk.Tk) -> ttk.Frame:
    """
    창 내용 전체를 감싸는 스크롤 영역. 섹션이 늘어나거나 창을 줄여도 아래
    내용이 잘리지 않고 스크롤로 접근 가능하다.

    Canvas 위에 내용 Frame 을 얹는 표준 Tk 패턴이다 (ttk 는 스크롤 가능한
    컨테이너를 기본 제공하지 않는다) — Canvas 만 스크롤이 가능하므로, 실제
    위젯들은 그 안에 올린 Frame(content) 에 담고 Canvas 는 뷰포트 역할만 한다.
    """
    outer = ttk.Frame(root)
    outer.pack(fill="both", expand=True)

    canvas = tk.Canvas(outer, bg=BG, highlightthickness=0, bd=0)
    scrollbar = ttk.Scrollbar(outer, orient="vertical", command=canvas.yview)
    canvas.configure(yscrollcommand=scrollbar.set)
    canvas.pack(side="left", fill="both", expand=True)
    scrollbar.pack(side="right", fill="y")

    content = ttk.Frame(canvas)
    window_id = canvas.create_window((0, 0), window=content, anchor="nw")

    def _on_content_resize(_event: tk.Event) -> None:
        canvas.configure(scrollregion=canvas.bbox("all"))

    def _on_canvas_resize(event: tk.Event) -> None:
        # content 폭을 canvas 폭에 맞춰야 안의 위젯들(fill="x")이 창 너비를 따라간다
        canvas.itemconfigure(window_id, width=event.width)

    content.bind("<Configure>", _on_content_resize)
    canvas.bind("<Configure>", _on_canvas_resize)

    def _on_mousewheel(event: tk.Event) -> None:
        if event.num == 4:          # Linux 스크롤 업
            canvas.yview_scroll(-1, "units")
        elif event.num == 5:        # Linux 스크롤 다운
            canvas.yview_scroll(1, "units")
        else:                       # macOS/Windows
            canvas.yview_scroll(-1 if event.delta > 0 else 1, "units")

    # 포인터가 창 위에 있을 때만 바인딩 — 다른 창의 스크롤을 가로채지 않는다
    def _bind_wheel(_event: tk.Event) -> None:
        canvas.bind_all("<MouseWheel>", _on_mousewheel)
        canvas.bind_all("<Button-4>", _on_mousewheel)
        canvas.bind_all("<Button-5>", _on_mousewheel)

    def _unbind_wheel(_event: tk.Event) -> None:
        canvas.unbind_all("<MouseWheel>")
        canvas.unbind_all("<Button-4>")
        canvas.unbind_all("<Button-5>")

    canvas.bind("<Enter>", _bind_wheel)
    canvas.bind("<Leave>", _unbind_wheel)

    return content


def launch_gui(
    executor: "OrderExecutor",
    command_queue: "queue.Queue",
    *,
    max_velocity: float,
    max_acceleration: float,
    robot_width: float,
    robot_length: float,
    on_close: Callable[[], None],
) -> None:
    """창을 만들고 mainloop 를 돈다. 창이 닫히면 on_close() 를 부르고 반환한다."""
    _set_macos_menu_bar_name(f"{executor.robot_id} 제어판")
    root = tk.Tk()
    root.title(f"{executor.robot_id} 제어판")
    root.geometry("420x720")
    root.minsize(380, 360)
    _setup_style(root)
    if _ICON_PATH.exists():
        icon = tk.PhotoImage(file=str(_ICON_PATH))
        root.iconphoto(True, icon)
        root._icon_ref = icon  # GC 방지

    content = _build_scroll_area(root)

    _, id_label, status_var, status_label = _build_header(content, executor)
    panel = _build_panel_section(content, executor, command_queue)
    info_vars = _build_info_section(content, executor)
    settings = _build_settings_section(
        content, executor, command_queue,
        robot_width=robot_width, robot_length=robot_length,
        max_velocity=max_velocity, max_acceleration=max_acceleration,
    )
    _build_move_section(content, command_queue)
    _build_relocalize_section(content, command_queue)

    def _refresh() -> None:
        snapshot = _update_info(info_vars, executor)
        status_var.set(snapshot.display_state)
        status_label.configure(foreground=STATE_COLORS.get(snapshot.state, FG))
        panel.sync(executor)
        # 설정 변경은 IDLE 에서만. e-stop 이 눌려 있으면 state 는 EMERGENCY 라
        # 자동으로 잠긴다
        settings.set_enabled(snapshot.state == RobotState.IDLE)
        if id_label.cget("text") != executor.robot_id:
            id_label.configure(text=executor.robot_id)
            root.title(f"{executor.robot_id} 제어판")
        root.after(REFRESH_MS, _refresh)

    def _on_window_close() -> None:
        on_close()
        root.destroy()

    root.protocol("WM_DELETE_WINDOW", _on_window_close)
    root.after(REFRESH_MS, _refresh)
    root.mainloop()


# -- 헤더 ------------------------------------------------------------------

def _build_header(
    root: tk.Tk, executor: "OrderExecutor",
) -> tuple[ttk.Frame, ttk.Label, tk.StringVar, ttk.Label]:
    frame = ttk.Frame(root)
    frame.pack(fill="x", padx=14, pady=(14, 4))
    id_label = ttk.Label(frame, text=executor.robot_id, style="Header.TLabel")
    id_label.pack(side="left")
    status_var = tk.StringVar(value="-")
    status_label = ttk.Label(frame, textvariable=status_var, style="Header.TLabel")
    status_label.pack(side="right")
    return frame, id_label, status_var, status_label


# -- 물리 패널 (실물 로봇 몸체 버튼의 재현) -----------------------------------

class _PanelSection:
    """e-stop 래치와 auto/manual 토글을 executor 의 실제 값과 맞춰준다."""

    def __init__(self, estop_button: ttk.Button, mode_var: tk.StringVar, hint: tk.StringVar):
        self._estop = estop_button
        self._mode_var = mode_var
        self._hint = hint

    def sync(self, executor: "OrderExecutor") -> None:
        # 버튼 상태의 진실은 executor 다. GUI 는 명령만 큐에 넣고 결과를 읽어 반영한다
        if executor.estop_latched:
            self._estop.configure(text="■ E-STOP 눌림 — 다시 눌러 해제", style="EstopOn.TButton")
        else:
            self._estop.configure(text="■ E-STOP", style="Estop.TButton")
        self._mode_var.set(executor.mode.value)
        if executor.paused and executor.pause_source is not None:
            self._hint.set(f"일시정지 중 ({executor.pause_source.value}) — START 로 해제")
        elif executor.mode == OperatingMode.MANUAL:
            self._hint.set("MANUAL — job 을 받지 않고 주행도 멈춘다")
        elif executor.estop_latched:
            self._hint.set("래치를 풀면 하던 작업을 그 자리에서 이어서 한다")
        else:
            self._hint.set("")


def _build_panel_section(
    root: tk.Tk, executor: "OrderExecutor", command_queue: "queue.Queue",
) -> _PanelSection:
    frame = _section(root, "물리 패널")

    hint = tk.StringVar(value="")

    button_row = ttk.Frame(frame, style="Panel.TFrame")
    button_row.pack(fill="x")

    ttk.Button(
        button_row, text="START", style="Accent.TButton",
        command=lambda: command_queue.put(("press_start",)),
    ).pack(side="left", expand=True, fill="x", padx=(0, 4))
    ttk.Button(
        button_row, text="STOP", style="Panel.TButton",
        command=lambda: command_queue.put(("press_stop",)),
    ).pack(side="left", expand=True, fill="x", padx=(4, 0))

    estop = ttk.Button(frame, text="■ E-STOP", style="Estop.TButton")
    estop.configure(command=lambda: command_queue.put(("estop", not executor.estop_latched)))
    estop.pack(fill="x", pady=(10, 2))

    mode_row = ttk.Frame(frame, style="Panel.TFrame")
    mode_row.pack(fill="x", pady=(10, 0))
    ttk.Label(mode_row, text="모드", style="Dim.TLabel", width=12).pack(side="left")
    mode_var = tk.StringVar(value=executor.mode.value)

    def on_mode() -> None:
        command_queue.put(("mode", mode_var.get()))

    for mode in (OperatingMode.AUTO, OperatingMode.MANUAL):
        ttk.Radiobutton(
            mode_row, text=mode.value, value=mode.value, variable=mode_var,
            command=on_mode, style="Mode.TRadiobutton",
        ).pack(side="left", padx=(0, 12))

    ttk.Label(frame, textvariable=hint, style="Hint.TLabel",
              wraplength=320, justify="left").pack(fill="x", pady=(6, 0))

    return _PanelSection(estop, mode_var, hint)


# -- 정보 패널 -----------------------------------------------------------

def _build_info_section(root: tk.Tk, executor: "OrderExecutor") -> dict[str, tk.StringVar]:
    frame = _section(root, "실시간 상태")

    dynamic_fields = [
        "sub_state", "x", "y", "theta", "velocity", "battery", "order_id", "errors",
    ]
    labels_ko = {
        "sub_state": "세부 단계", "x": "x (m)", "y": "y (m)", "theta": "각도 (rad)",
        "velocity": "속도 (m/s)", "battery": "배터리", "order_id": "오더", "errors": "오류",
    }
    info_vars: dict[str, tk.StringVar] = {}
    for field in dynamic_fields:
        row = ttk.Frame(frame, style="Panel.TFrame")
        row.pack(fill="x", pady=2)
        ttk.Label(row, text=labels_ko[field], style="Dim.TLabel", width=12).pack(side="left")
        var = tk.StringVar(value="-")
        ttk.Label(row, textvariable=var, style="Value.TLabel").pack(side="left")
        info_vars[field] = var

    battery_row = ttk.Frame(frame, style="Panel.TFrame")
    battery_row.pack(fill="x", pady=(6, 0))
    battery_bar = ttk.Progressbar(battery_row, orient="horizontal", length=100,
                                   mode="determinate", maximum=100)
    battery_bar.pack(fill="x")
    info_vars["_battery_bar"] = battery_bar  # type: ignore[assignment]
    return info_vars


def _update_info(info_vars: dict[str, tk.StringVar], executor: "OrderExecutor") -> "State":
    state = executor.build_state(header_id=0, timestamp=0.0)
    info_vars["sub_state"].set(state.sub_state.value if state.sub_state else "-")
    info_vars["x"].set(f"{state.pose.x:.3f}")
    info_vars["y"].set(f"{state.pose.y:.3f}")
    info_vars["theta"].set(f"{state.pose.theta:.3f}")
    info_vars["velocity"].set(f"{state.velocity:.3f}")
    info_vars["battery"].set(f"{state.battery.charge:.1f} %")
    info_vars["order_id"].set(state.order_id or "(없음)")
    info_vars["errors"].set(", ".join(e.error_type for e in state.errors) or "(없음)")
    bar = info_vars["_battery_bar"]
    bar["value"] = state.battery.charge
    return state


# -- 설정 변경 (ID / 크기 / 속도 / 가속도, IDLE 전용) -------------------------

class _SettingsSection:
    """엔트리 위젯 묶음 + IDLE 여부에 따른 활성화 토글."""

    def __init__(self, entries: dict[str, tk.Entry], button: ttk.Button, hint: tk.StringVar):
        self._entries = entries
        self._button = button
        self._hint = hint

    def set_enabled(self, enabled: bool) -> None:
        state = "normal" if enabled else "disabled"
        for entry in self._entries.values():
            entry.configure(state=state)
        self._button.configure(state=state)
        self._hint.set("" if enabled else "IDLE 상태에서만 수정 가능")


def _build_settings_section(
    root: tk.Tk, executor: "OrderExecutor", command_queue: "queue.Queue",
    *, robot_width: float, robot_length: float,
    max_velocity: float, max_acceleration: float,
) -> _SettingsSection:
    frame = _section(root, "로봇 설정 (IDLE 에서만 변경)")

    id_var = tk.StringVar(value=executor.robot_id)
    width_var = tk.StringVar(value=f"{robot_width:.2f}")
    length_var = tk.StringVar(value=f"{robot_length:.2f}")
    velocity_var = tk.StringVar(value=f"{max_velocity:.3f}")
    accel_var = tk.StringVar(value=f"{max_acceleration:.3f}")

    entries: dict[str, tk.Entry] = {}

    def _row(label: str, var: tk.StringVar, key: str) -> None:
        r = ttk.Frame(frame, style="Panel.TFrame")
        r.pack(fill="x", pady=1)
        ttk.Label(r, text=label, style="Dim.TLabel", width=12).pack(side="left")
        entry = ttk.Entry(r, textvariable=var, width=16)
        entry.pack(side="left")
        entries[key] = entry

    _row("로봇 ID", id_var, "robot_id")
    _row("폭 (m)", width_var, "robot_width")
    _row("길이 (m)", length_var, "robot_length")
    _row("최대 속도 (m/s)", velocity_var, "max_velocity")
    _row("최대 가속도 (m/s²)", accel_var, "max_acceleration")

    status = tk.StringVar(value="")

    def on_apply() -> None:
        robot_id = id_var.get().strip()
        parsed = {
            "robot_width": _parse_positive(width_var.get()),
            "robot_length": _parse_positive(length_var.get()),
            "max_velocity": _parse_positive(velocity_var.get()),
            "max_acceleration": _parse_positive(accel_var.get()),
        }
        if not robot_id:
            status.set("로봇 ID 는 비울 수 없다")
            return
        if any(v is None for v in parsed.values()):
            status.set("폭/길이/속도/가속도는 0보다 큰 숫자여야 한다")
            return
        parsed["robot_id"] = robot_id
        command_queue.put(("configure", parsed))
        status.set("설정 변경 명령 전송")

    button = ttk.Button(frame, text="적용", style="Accent.TButton", command=on_apply)
    button.pack(pady=(8, 2), anchor="e")
    ttk.Label(frame, textvariable=status, style="Hint.TLabel").pack(fill="x")

    return _SettingsSection(entries, button, status)


# -- 이동 명령 -------------------------------------------------------------

def _build_move_section(root: tk.Tk, command_queue: "queue.Queue") -> None:
    frame = _section(root, "이동 명령 (목적지 좌표)")

    x_var = tk.StringVar()
    y_var = tk.StringVar()
    _xy_entry_row(frame, "x (m)", x_var)
    _xy_entry_row(frame, "y (m)", y_var)

    status = tk.StringVar(value="")

    def on_submit() -> None:
        parsed = _parse_xy(x_var.get(), y_var.get())
        if parsed is None:
            status.set("x, y 는 숫자여야 한다")
            return
        command_queue.put(("move", parsed[0], parsed[1]))
        status.set(f"이동 명령 전송: ({parsed[0]:.2f}, {parsed[1]:.2f})")

    ttk.Button(frame, text="이동", style="Accent.TButton", command=on_submit).pack(pady=(8, 2), anchor="e")
    ttk.Label(frame, textvariable=status, style="Hint.TLabel").pack(fill="x")


# -- 강제 로컬라이징 --------------------------------------------------------

def _build_relocalize_section(root: tk.Tk, command_queue: "queue.Queue") -> None:
    frame = _section(root, "강제 로컬라이징 (위치 강제 지정)")

    ttk.Label(
        frame, text="실물 로봇엔 없는 디버그 기능. 진행 중인 오더는 버려진다.",
        style="Warn.TLabel", wraplength=320, justify="left",
    ).pack(fill="x", pady=(0, 6))

    x_var = tk.StringVar()
    y_var = tk.StringVar()
    theta_var = tk.StringVar(value="0.0")
    _xy_entry_row(frame, "x (m)", x_var)
    _xy_entry_row(frame, "y (m)", y_var)
    _xy_entry_row(frame, "theta (rad)", theta_var)

    status = tk.StringVar(value="")

    def on_submit() -> None:
        parsed = _parse_xy(x_var.get(), y_var.get())
        theta = _parse_float(theta_var.get())
        if parsed is None or theta is None:
            status.set("x, y, theta 는 숫자여야 한다")
            return
        command_queue.put(("relocalize", parsed[0], parsed[1], theta))
        status.set(f"재위치 명령 전송: ({parsed[0]:.2f}, {parsed[1]:.2f}, {theta:.2f})")

    ttk.Button(frame, text="강제 적용", style="Accent.TButton", command=on_submit).pack(pady=(8, 2), anchor="e")
    ttk.Label(frame, textvariable=status, style="Hint.TLabel").pack(fill="x")


# -- 공용 위젯/파싱 헬퍼 -----------------------------------------------------

def _xy_entry_row(parent: tk.Widget, label: str, var: tk.StringVar) -> None:
    row = ttk.Frame(parent, style="Panel.TFrame")
    row.pack(fill="x", pady=1)
    ttk.Label(row, text=label, style="Dim.TLabel", width=10).pack(side="left")
    ttk.Entry(row, textvariable=var, width=14).pack(side="left")


def _parse_float(text: str) -> Optional[float]:
    try:
        return float(text.strip())
    except (TypeError, ValueError):
        return None


def _parse_positive(text: str) -> Optional[float]:
    value = _parse_float(text)
    if value is None or value <= 0:
        return None
    return value


def _parse_xy(x_text: str, y_text: str) -> Optional[tuple[float, float]]:
    x = _parse_float(x_text)
    y = _parse_float(y_text)
    if x is None or y is None:
        return None
    return x, y
