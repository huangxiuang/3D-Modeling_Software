#!/usr/bin/env python3
"""
3DSceneSoftware — entry point
==============================
Usage::

    python 3DSceneSoftware_test2.py

Launches the 3D scene visualisation and modelling application.
"""

import sys
import os

# Ensure the project root is on sys.path so ``src`` is importable
_proj_root = os.path.dirname(os.path.abspath(__file__))
if _proj_root not in sys.path:
    sys.path.insert(0, _proj_root)

# ── Qt High-DPI attributes must be set BEFORE QCoreApplication is created ──
from PyQt5.QtCore import Qt

if hasattr(Qt, "AA_EnableHighDpiScaling"):
    from PyQt5 import QtCore

    QtCore.QCoreApplication.setAttribute(Qt.AA_EnableHighDpiScaling, True)
if hasattr(Qt, "AA_UseHighDpiPixmaps"):
    from PyQt5 import QtCore

    QtCore.QCoreApplication.setAttribute(Qt.AA_UseHighDpiPixmaps, True)

import pyvista as pv
from pyvista import themes
from PyQt5 import QtWidgets

from src.main_window import MainWindow


def main():
    # Use a clean, document-style theme
    pv.set_plot_theme(themes.DocumentTheme())

    app = QtWidgets.QApplication(sys.argv)
    app.setApplicationName("3DSceneSoftware")
    app.setOrganizationName("3DSceneSoft")

    _ = MainWindow()
    sys.exit(app.exec_())


if __name__ == "__main__":
    main()
