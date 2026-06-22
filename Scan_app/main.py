from __future__ import annotations

import sys
from PyQt5.QtWidgets import QApplication

from core.project_io import load_config
from gui.main_window import ScannerMainWindow


def main() -> int:
    cfg = load_config()
    app = QApplication(sys.argv)
    win = ScannerMainWindow(cfg)
    win.show()
    return app.exec_()


if __name__ == "__main__":
    raise SystemExit(main())
