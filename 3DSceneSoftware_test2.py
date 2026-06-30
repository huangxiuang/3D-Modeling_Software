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

    # Ensure high-DPI support
    app.setAttribute(QtCore.Qt.AA_UseHighDpiPixmaps, True)
    if hasattr(QtCore.Qt, "AA_EnableHighDpiScaling"):
        app.setAttribute(QtCore.Qt.AA_EnableHighDpiScaling, True)

    _ = MainWindow()
    sys.exit(app.exec_())


if __name__ == "__main__":
    # Late import for Qt attributes
    from PyQt5 import QtCore
    main()
