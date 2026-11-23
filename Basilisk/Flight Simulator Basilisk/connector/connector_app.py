"""AOCS Connector -- graphical editor for the software/simulator binding.

Double-click ``Run_Connector.exe`` (or run ``pythonw connector_app.py``).
An optional argument opens a specific ``.aocslink`` file.
"""

import subprocess
import sys
import traceback
from pathlib import Path

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))

# pythonw has no console: keep tracebacks and simulator output in a log file.
if sys.stderr is None or sys.stdout is None:
    _log = open(HERE / "connector.log", "w", buffering=1, encoding="utf-8")
    sys.stdout = sys.stdout or _log
    sys.stderr = sys.stderr or _log


def _ensure_pyside():
    """Offer to install PySide6 when it is missing (no console needed)."""
    try:
        import PySide6  # noqa: F401
        return True
    except ImportError:
        pass
    import tkinter
    from tkinter import messagebox
    root = tkinter.Tk()
    root.withdraw()
    if not messagebox.askyesno(
            "AOCS Connector",
            "The connector needs the PySide6 (Qt) package, which is not "
            "installed.\n\nInstall it now with pip? (about 80 MB)"):
        return False
    result = subprocess.run(
        [sys.executable.replace("pythonw", "python"), "-m", "pip", "install",
         "PySide6-Essentials"], capture_output=True, text=True)
    if result.returncode != 0:
        messagebox.showerror("AOCS Connector",
                             "pip install failed:\n\n" + result.stderr[-1500:])
        return False
    return True


def main():
    if not _ensure_pyside():
        return 1

    from PySide6.QtWidgets import QApplication, QMessageBox

    app = QApplication(sys.argv)
    app.setApplicationName("AOCS Connector")
    app.setStyle("Fusion")

    def show_error(kind, value, tb):
        text = "".join(traceback.format_exception(kind, value, tb))
        sys.stderr.write(text)
        box = QMessageBox(QMessageBox.Critical, "AOCS Connector — error",
                          f"{kind.__name__}: {value}")
        box.setDetailedText(text)
        box.exec()

    sys.excepthook = show_error

    from aocs_connector.gui.window import ConnectorWindow

    path = sys.argv[1] if len(sys.argv) > 1 else None
    window = ConnectorWindow(path)
    window.show()
    return app.exec()


if __name__ == "__main__":
    sys.exit(main())
