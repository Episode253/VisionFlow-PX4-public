#!/usr/bin/env python3
"""
Embedded Gamma Arm GUI.

This opens the existing local web UI in a QtWebEngine window, so the control
panel no longer depends on an external browser application.
"""
import os
import sys


def main() -> int:
    if len(sys.argv) < 2:
        print("Usage: gamma_arm_gui.py http://127.0.0.1:9000/index.html", file=sys.stderr)
        return 2

    url = sys.argv[1]

    from PySide6.QtCore import QUrl
    from PySide6.QtWidgets import QApplication
    from PySide6.QtWebEngineWidgets import QWebEngineView

    os.environ.setdefault("QTWEBENGINE_DISABLE_SANDBOX", "1")

    app = QApplication(sys.argv)
    view = QWebEngineView()
    view.setWindowTitle("Gamma Arm Web Control")
    view.resize(1360, 900)
    view.load(QUrl(url))
    view.show()
    return app.exec()


if __name__ == "__main__":
    raise SystemExit(main())
