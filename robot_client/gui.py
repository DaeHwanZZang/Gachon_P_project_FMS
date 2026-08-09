"""
로봇 제어판 (디버그/테스트용 GUI)
================================

`robot_client.main --gui` 로 띄우는 로컬 조작 창. 표준 Tk 위젯만 쓴다
(추가 의존성 없음 — 헤드리스 컨테이너에서 --gui 없이 돌릴 땐 이 모듈을
아예 import 하지 않는다, `main.py` 참고).

**이건 FMS 가 아니다.** 실물 로봇에 없는 기능(강제 재위치)까지 있는,
사람이 로봇 하나를 손으로 찔러보기 위한 판넬이다. MQTT 프로토콜을 타지
않고 같은 프로세스 안에서 커맨드 큐를 통해 tick 루프에 바로 꽂는다
(main.py 의 `_dispatch_gui_command` 가 실제 실행부).

스레드 모델
-----------
Tk 는 메인 스레드에서 돌아야 안전하다(특히 macOS). 그래서 로봇의 tick/MQTT
루프를 백그라운드 스레드로 보내고, 이 GUI 가 메인 스레드에서 `mainloop()`
를 돈다. 정보 패널은 `executor` 를 락 없이 직접 읽는다 — 표시 전용이고
0.2초마다 갱신되니 아주 드물게 값이 살짝 어긋나 보여도 다음 틱에 바로
정정된다 (디버그 패널이라 감수할 수준).
"""

from __future__ import annotations

import queue
import tkinter as tk
from tkinter import ttk
from typing import TYPE_CHECKING, Callable, Optional

if TYPE_CHECKING:
    from robot_client.executor import OrderExecutor

REFRESH_MS = 200


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
    root = tk.Tk()
    root.title(f"{executor.robot_id} 제어판")
    root.geometry("340x520")

    info_vars = _build_info_section(root, executor, max_velocity, max_acceleration,
                                     robot_width, robot_length)
    _build_move_section(root, command_queue)
    _build_relocalize_section(root, command_queue)

    def _refresh() -> None:
        _update_info(info_vars, executor)
        root.after(REFRESH_MS, _refresh)

    def _on_window_close() -> None:
        on_close()
        root.destroy()

    root.protocol("WM_DELETE_WINDOW", _on_window_close)
    root.after(REFRESH_MS, _refresh)
    root.mainloop()


# -- 정보 패널 -----------------------------------------------------------

def _build_info_section(
    root: tk.Tk, executor: "OrderExecutor",
    max_velocity: float, max_acceleration: float,
    robot_width: float, robot_length: float,
) -> dict[str, tk.StringVar]:
    frame = ttk.LabelFrame(root, text="로봇 정보")
    frame.pack(fill="x", padx=8, pady=6)

    static_rows = [
        ("로봇 ID", executor.robot_id),
        ("크기 (W x L)", f"{robot_width:.2f} x {robot_length:.2f} m"),
        ("최대 속도", f"{max_velocity:.3f} m/s"),
        ("최대 가속도", f"{max_acceleration:.3f} m/s²"),
    ]
    for label, value in static_rows:
        row = ttk.Frame(frame)
        row.pack(fill="x", padx=4, pady=1)
        ttk.Label(row, text=label, width=14).pack(side="left")
        ttk.Label(row, text=value).pack(side="left")

    ttk.Separator(frame).pack(fill="x", pady=4)

    dynamic_fields = ["state", "x", "y", "theta", "velocity", "battery", "order_id", "errors"]
    labels_ko = {
        "state": "상태", "x": "x", "y": "y", "theta": "각도(rad)",
        "velocity": "속도", "battery": "배터리", "order_id": "오더", "errors": "오류",
    }
    info_vars: dict[str, tk.StringVar] = {}
    for field in dynamic_fields:
        row = ttk.Frame(frame)
        row.pack(fill="x", padx=4, pady=1)
        ttk.Label(row, text=labels_ko[field], width=14).pack(side="left")
        var = tk.StringVar(value="-")
        ttk.Label(row, textvariable=var).pack(side="left")
        info_vars[field] = var
    return info_vars


def _update_info(info_vars: dict[str, tk.StringVar], executor: "OrderExecutor") -> None:
    state = executor.build_state(header_id=0, timestamp=0.0)
    info_vars["state"].set(state.state.value)
    info_vars["x"].set(f"{state.pose.x:.3f} m")
    info_vars["y"].set(f"{state.pose.y:.3f} m")
    info_vars["theta"].set(f"{state.pose.theta:.3f}")
    info_vars["velocity"].set(f"{state.velocity:.3f} m/s")
    info_vars["battery"].set(f"{state.battery.charge:.1f} %")
    info_vars["order_id"].set(state.order_id or "(없음)")
    info_vars["errors"].set(", ".join(e.error_type for e in state.errors) or "(없음)")


# -- 이동 명령 -------------------------------------------------------------

def _build_move_section(root: tk.Tk, command_queue: "queue.Queue") -> None:
    frame = ttk.LabelFrame(root, text="이동 명령 (목적지 좌표)")
    frame.pack(fill="x", padx=8, pady=6)

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

    ttk.Button(frame, text="이동", command=on_submit).pack(padx=4, pady=4, anchor="e")
    ttk.Label(frame, textvariable=status, foreground="#555").pack(fill="x", padx=4)


# -- 강제 로컬라이징 --------------------------------------------------------

def _build_relocalize_section(root: tk.Tk, command_queue: "queue.Queue") -> None:
    frame = ttk.LabelFrame(root, text="강제 로컬라이징 (위치 강제 지정)")
    frame.pack(fill="x", padx=8, pady=6)

    ttk.Label(
        frame, text="실물 로봇엔 없는 디버그 기능. 진행 중인 오더는 버려진다.",
        foreground="#a00", wraplength=300, justify="left",
    ).pack(fill="x", padx=4, pady=(2, 4))

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

    ttk.Button(frame, text="강제 적용", command=on_submit).pack(padx=4, pady=4, anchor="e")
    ttk.Label(frame, textvariable=status, foreground="#555").pack(fill="x", padx=4)


# -- 공용 위젯/파싱 헬퍼 -----------------------------------------------------

def _xy_entry_row(parent: tk.Widget, label: str, var: tk.StringVar) -> None:
    row = ttk.Frame(parent)
    row.pack(fill="x", padx=4, pady=1)
    ttk.Label(row, text=label, width=10).pack(side="left")
    ttk.Entry(row, textvariable=var, width=12).pack(side="left")


def _parse_float(text: str) -> Optional[float]:
    try:
        return float(text.strip())
    except (TypeError, ValueError):
        return None


def _parse_xy(x_text: str, y_text: str) -> Optional[tuple[float, float]]:
    x = _parse_float(x_text)
    y = _parse_float(y_text)
    if x is None or y is None:
        return None
    return x, y
