"""
플릿 모니터 (Qt 뷰어 버전)
=========================

`tools/fleet_monitor.py` 와 MQTT 구독 로직은 완전히 동일하다 (그 파일의
`FleetMonitor` 클래스를 그대로 재사용한다 — 렌더러에 의존하지 않게 짜여
있었기 때문에 뷰어만 matplotlib -> Qt 로 갈아끼우면 됐다).

    python tools/fleet_monitor_qt.py maps/warehouse/warehouse.json

조작
    드래그        화면 이동 (Qt 기본 ScrollHandDrag)
    스크롤        확대/축소 (커서 위치 기준, Qt 기본 AnchorUnderMouse)
    g             격자 토글
    c             색상 모드 토글 (분류색 <-> 원본 그레이스케일)
    t             계획 경로 토글
    r             보기 초기화 (맵 전체)
"""

from __future__ import annotations

import argparse
import logging
import sys
import time
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

from PySide6.QtCore import QTimer  # noqa: E402
from PySide6.QtGui import QIcon  # noqa: E402
from PySide6.QtWidgets import QApplication  # noqa: E402

from common.map_model import MapModel  # noqa: E402
from tools.fleet_monitor import FleetMonitor  # noqa: E402
from tools.qt_viewer import ICON_PATH, FleetViewWindow  # noqa: E402

log = logging.getLogger("monitor-qt")

REFRESH_PERIOD_MS = 100
APP_NAME = "Fleet Viewer"


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


def main() -> int:
    parser = argparse.ArgumentParser(description="MQTT 로 로봇들을 관측해 맵 위에 표시 (Qt 뷰어)")
    parser.add_argument("metadata", help="맵 메타데이터 JSON 경로")
    parser.add_argument("--host", default="localhost", help="MQTT 브로커 호스트")
    parser.add_argument("--port", type=int, default=1883, help="MQTT 브로커 포트")
    parser.add_argument("--no-orders", action="store_true", help="order 토픽은 구독하지 않는다")
    parser.add_argument("--log-level", default="INFO")
    args = parser.parse_args()

    logging.basicConfig(
        level=getattr(logging, args.log_level.upper(), logging.INFO),
        format="%(asctime)s %(levelname)-7s %(message)s",
        datefmt="%H:%M:%S",
    )

    _set_macos_menu_bar_name(APP_NAME)
    app = QApplication(sys.argv)
    app.setApplicationName(APP_NAME)
    if ICON_PATH.exists():
        app.setWindowIcon(QIcon(str(ICON_PATH)))

    map_model = MapModel.load(args.metadata)
    window = FleetViewWindow(map_model, title=f"Fleet Viz — {map_model.metadata.name}")
    window.show()

    monitor = FleetMonitor(
        window, args.host, args.port,
        watch_orders=not args.no_orders, on_event=window.append_event,
    )
    try:
        monitor.start()
    except OSError as e:
        log.error("브로커 접속 실패 (%s:%d): %s — mosquitto 가 떠 있는지 확인",
                  args.host, args.port, e)
        return 1

    timer = QTimer()
    timer.setInterval(REFRESH_PERIOD_MS)
    timer.timeout.connect(lambda: (monitor.pump(), window.refresh(time.time())))
    timer.start()

    print(f"브로커 {args.host}:{args.port} 관측 중. 로봇 프로세스를 띄우면 여기 나타난다.\n")
    try:
        return app.exec()
    finally:
        timer.stop()
        monitor.stop()


if __name__ == "__main__":
    sys.exit(main())
