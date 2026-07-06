"""
MainWindow — top-level QMainWindow integrating all UI, scene, and tool subsystems.

Key design decisions
--------------------
* Mouse events are captured at the **Qt** level (``ClickablePlotter``), *not* via
  VTK observers.  PyVista's ``RenderWindowInteractor`` wrapper does not
  propagate VTK observers when used inside ``QVTKRenderWindowInteractor``.
* Scene objects are initialised *before* the UI so docks and trees populate
  correctly on first render.
* Object transforms (position, scale) use VTK's ``UserTransform`` so mesh data
  is never mutated by the UI.
"""

import os
import json
import math
import time
import numpy as np
import pyvista as pv
import vtk
from PyQt5 import QtWidgets, QtCore, QtGui
from PyQt5.QtCore import Qt, QTimer
import pyvistaqt as pvqt

# Optional: matplotlib for plotting and embedded dialogs
try:
    import matplotlib
    matplotlib.use("Qt5Agg")
    import matplotlib.pyplot as plt
    from mpl_toolkits.mplot3d import Axes3D
    from matplotlib.backends.backend_qt5agg import FigureCanvasQTAgg
    from matplotlib.figure import Figure

    _HAS_MPL = True
except ImportError:
    _HAS_MPL = False

from src.config import (
    DEFAULT_CONFIG,
    COORD_SYSTEMS,
    world_to_coord_str,
    load_config,
    save_config,
)
from src.interaction import InteractionMode
from src.scene_builder import build_default_scene
from src.measurement import MeasurementTool
from src.collision import find_collisions


# ═══════════════════════════════════════════════════════════════
#  ClickablePlotter — Qt-level mouse handler
# ═══════════════════════════════════════════════════════════════

class ClickablePlotter(pvqt.QtInteractor):
    """``QtInteractor`` subclass that captures mouse events at the Qt level.

    PyVista's ``RenderWindowInteractor`` wrapper does **not** deliver
    VTK observer callbacks for mouse events (``add_observer`` registers
    successfully but callbacks never fire).  We work around this by
    intercepting events in Qt's event system before they reach VTK.

    Click detection
    ---------------
    A "click" is defined as a left-button press followed by a release
    within 5 screen pixels and 0.5 seconds (i.e. not a drag intended
    for camera orbit).  The click callback receives screen coordinates,
    the world position (via ``vtkPropPicker``), and the hit
    ``vtkActor`` (or ``None``).

    Camera controls are preserved by calling ``super().*Event()`` so
    VTK still processes drags normally.
    """

    def __init__(self, parent=None):
        super().__init__(parent)
        self.setMouseTracking(True)
        self._press_pos = None          # (x, y) at press
        self._press_time = 0.0
        self.click_callback = None      # f(x, y, world_pos, vtkActor)
        self.move_callback = None       # f(x, y, world_pos)

    def _to_vtk_display(self, qt_x, qt_y):
        """Convert Qt widget coords → VTK display coords (pixels, bottom-left origin).

        Qt origin is top-left, VTK display origin is bottom-left.
        Must also scale by device pixel ratio for Retina/HiDPI displays.
        The parent ``QVTKRenderWindowInteractor._setEventInformation`` does the
        same conversion internally — this must match it exactly.
        """
        scale = QtWidgets.QApplication.instance().devicePixelRatio()
        win_size = self.renderer.GetRenderWindow().GetSize()
        vtk_x = int(round(qt_x * scale))
        vtk_y = win_size[1] - int(round(qt_y * scale)) - 1
        return vtk_x, vtk_y

    def mousePressEvent(self, event):
        if event.button() == QtCore.Qt.LeftButton:
            self._press_pos = (event.pos().x(), event.pos().y())
            self._press_time = time.time()
        super().mousePressEvent(event)

    def mouseReleaseEvent(self, event):
        if event.button() == QtCore.Qt.LeftButton and self._press_pos is not None:
            rx, ry = event.pos().x(), event.pos().y()
            dx = rx - self._press_pos[0]
            dy = ry - self._press_pos[1]
            dt = time.time() - self._press_time
            if dx * dx + dy * dy < 25 and dt < 0.5:
                self._process_click(rx, ry)
        self._press_pos = None
        super().mouseReleaseEvent(event)

    def mouseMoveEvent(self, event):
        super().mouseMoveEvent(event)
        if self.move_callback is not None:
            x, y = event.pos().x(), event.pos().y()
            self.move_callback(x, y)

    # ── Internal click processing ───────────────────

    def _process_click(self, x, y):
        if self.click_callback is None:
            return
        vtk_x, vtk_y = self._to_vtk_display(x, y)

        # Use vtkCellPicker for accurate surface intersection (more precise
        # than PropPicker's prop-level pick, especially with overlaid layers).
        picker = vtk.vtkCellPicker()
        picker.SetTolerance(0.001)
        if picker.Pick(vtk_x, vtk_y, 0, self.renderer) and picker.GetCellId() >= 0:
            world = np.array(picker.GetPickPosition())
            actor = picker.GetActor()
        else:
            # Fallback: ray-terrain intersection via PropPicker on visible props
            pp = vtk.vtkPropPicker()
            if pp.Pick(vtk_x, vtk_y, 0, self.renderer):
                world = np.array(pp.GetPickPosition())
                actor = pp.GetActor()
            else:
                # Last resort: focal-plane pick (Z will be wrong, but XY roughly match)
                wp = vtk.vtkWorldPointPicker()
                wp.Pick(vtk_x, vtk_y, 0, self.renderer)
                world = np.array(wp.GetPickPosition())
                actor = None
        self.click_callback(x, y, world, actor)


# ═══════════════════════════════════════════════════════════════
#  MainWindow
# ═══════════════════════════════════════════════════════════════

class MainWindow(QtWidgets.QMainWindow):
    """Top-level application window."""

    # Save/load data directories (ID-10)
    SAVE_DIR_AIRCRAFT = "data/aircraft"
    SAVE_DIR_TERRAIN = "data/terrain"

    # ── lifecycle ─────────────────────────────────────

    def __init__(self):
        super().__init__()
        self.setWindowTitle("3DSceneSoftware — 3D 大场景可视化与目标建模")
        self.resize(1296, 816)

        # ── Config ──
        self.config = load_config()
        self.coord_system = self.config.get("coordinate_system", "ENU")

        # Scene object registry:  name → dict (see scene_builder.py)
        self.scene_objects = {}
        # Actor registry:  name → pyvista.Actor
        self.plotter_actors = {}
        # Reverse lookup:  vtkActor pointer string → name
        self._actor_to_name = {}
        # Custom / imported objects:  name → mesh
        self.custom_objects = {}

        # Selection & highlight
        self.selected_name = None
        self._highlight_props = {}
        self._highlighted_vtk_actor = None

        # Interaction mode
        self._current_mode = InteractionMode.NORMAL
        self._mode_buttons = []                # list of (widget, InteractionMode)

        # Object transform state (separate from scene_objects to avoid
        # mesh mutation)
        self._obj_transforms = {}              # name → {"offset": [3], "scale": float}

        # Waypoints
        self.waypoints = []                    # list of np.array (3,)
        self._wp_actors = []                   # graphics for waypoint dots + nums
        self._path_actor = None

        # Recording
        self._recording = False
        self._frame_count = 0
        self._rec_dir = ""

        # River animation
        self._flowing = True

        # Terrain layer management (ID-18)
        self._terrain_layer_names = {
            "layer_sand": "沙地",
            "layer_grass": "草地",
            "layer_earth": "土地",
            "river": "河流",
            "vegetation": "植被",
        }
        self._terrain_chks = {}        # name → QCheckBox
        self._terrain_opacity_sliders = {}  # name → slider widget
        self._terrain_opacity_setters = {}  # name → setter function

        # Flight animation (ID-11)
        self._flight_timer = QTimer(self)
        self._flight_timer.timeout.connect(self._flight_tick)
        self._flight_active = False
        self._flight_path = []
        self._flight_segments = []
        self._flight_segment_idx = 0
        self._flight_step = 0
        self._flight_steps_per_segment = 0
        self._flight_aircraft = ""
        self._flight_aircraft_list = []  # all aircraft in current flight (ID-27)
        self._flight_data_cache = None  # saved for later export

        # Per-aircraft waypoints (ID-20)
        self._aircraft_waypoints = {}   # name → list of waypoints (shared path attribute)
        # Saved flight states (ID-20)

        self._flight_window = None

        # Formation flight (ID-27)
        self._formation_mode = False
        self._formation_aircraft = []   # [name, ...]  first = leader, rest = followers
        self._formation_offsets = {}    # name → float trail distance

        # Timeline (ID-20)
        self._total_flight_steps = 0
        self._total_flight_time_ms = 0

        # ── Central 3D viewport ──
        self.plotter = ClickablePlotter(self)
        self.setCentralWidget(self.plotter)

        # ── Measurement sub-system ──
        self.meas_tool = MeasurementTool(self.plotter)

        # ── Scene FIRST (populates registries) ──
        self._init_scene()

        # ── UI SECOND (reads scene_objects) ──
        self._setup_menus()
        self._setup_toolbar()
        self._setup_docks()
        self._set_coord_system(self.coord_system)  # sync spinbox prefixes at startup
        self._refresh_ui()

        # ── Wire interaction callbacks on the plotter ──
        self.plotter.click_callback = self._on_3d_click
        self.plotter.move_callback = self._on_3d_move

        # ── Timers ──
        self._setup_timers()

        # ── Status bar ──
        self._status_label = QtWidgets.QLabel(
            "就绪  |  左键旋转 · 滚轮缩放 · 中键平移  |  单击选取物体"
        )
        self.statusBar().addWidget(self._status_label, 1)

        # ── Cleanup on quit ──
        QtWidgets.QApplication.instance().aboutToQuit.connect(self._cleanup)

        self.show()


    # ═══════════════════════════════════════════════════════════════
    #  Scene management
    # ═══════════════════════════════════════════════════════════════

    def _init_scene(self):
        """Build and display the default scene."""
        p = self.plotter
        p.background_color = self.config["bg_color"]

        light1 = pv.Light(position=(10, -10, 15), intensity=0.8)
        light2 = pv.Light(position=(-5, 5, 8), intensity=0.4)
        p.add_light(light1)
        p.add_light(light2)

        if self.config["show_axes"]:
            p.show_axes()
        if self.config["show_grid"]:
            p.show_grid()

        self.scene_objects = build_default_scene()
        for name, obj in self.scene_objects.items():
            if obj.get("visible", True):
                self._add_actor(name, obj)

        p.camera_position = [(18, -16, 8), (0, 0, 2), (0, 0, 1)]
        p.camera.focal_point = (0, 0, 1.5)
        p.render()

    # ── Actor CRUD ──────────────────────────────────

    def _add_actor(self, name, obj):
        """Add a scene-object's mesh to the 3D viewport and register it."""
        if name in self.plotter_actors:
            return
        mesh = obj["mesh"]
        params = dict(obj["params"])
        if obj["type"] == "points":
            actor = self.plotter.add_points(mesh, **params)
        else:
            actor = self.plotter.add_mesh(mesh, **params)
        self.plotter_actors[name] = actor
        self._register_actor_reverse_lookup(actor, name)
        self._tag_mesh(mesh, name)

    def _remove_actor(self, name):
        """Remove a registered actor from the viewport."""
        if name not in self.plotter_actors:
            return
        actor = self.plotter_actors.pop(name)
        self._unregister_actor_reverse_lookup(actor)
        try:
            self.plotter.remove_actor(actor)
        except Exception:
            pass

    def _rebuild_actor(self, name):
        """Remove and re-add the named actor (for visibility or data changes)."""
        obj = self.scene_objects.get(name)
        if obj is None:
            return
        self._remove_actor(name)
        if obj["visible"]:
            self._add_actor(name, obj)

    # ── Reverse actor → name lookup ────────────────

    def _tag_mesh(self, mesh, name):
        """Embed *name* in the mesh's FieldData for pick-time retrieval."""
        try:
            fd = mesh.GetFieldData()
            arr = fd.GetAbstractArray("__scene_name__")
            if arr is None:
                arr = vtk.vtkStringArray()
                arr.SetName("__scene_name__")
                fd.AddArray(arr)
            if arr.GetNumberOfValues() > 0:
                arr.SetValue(0, name)
            else:
                arr.InsertNextValue(name)
        except Exception:
            pass

    def _register_actor_reverse_lookup(self, actor, name):
        """Store *name* by the actor's underlying VTK address."""
        try:
            vtk_actor = self._resolve_vtk_actor(actor)
            key = vtk_actor.GetAddressAsString("")
            self._actor_to_name[key] = name
        except Exception:
            pass

    def _unregister_actor_reverse_lookup(self, actor):
        """Remove the reverse-lookup entry for *actor*."""
        try:
            vtk_actor = self._resolve_vtk_actor(actor)
            key = vtk_actor.GetAddressAsString("")
            self._actor_to_name.pop(key, None)
        except Exception:
            pass

    @staticmethod
    def _resolve_vtk_actor(actor):
        """Get the underlying ``vtkActor`` from a PyVista or VTK actor."""
        if hasattr(actor, "actor"):
            return actor.actor
        return actor

    def _get_name_from_vtk_actor(self, vtk_actor):
        """Look up the scene-object *name* by a ``vtkActor``."""
        try:
            key = vtk_actor.GetAddressAsString("")
            return self._actor_to_name.get(key)
        except Exception:
            return None

    def _get_name_from_pick(self, x, y):
        """Return the scene-object *name* at screen coordinates, or ``None``."""
        picker = vtk.vtkPropPicker()
        vtk_x, vtk_y = self.plotter._to_vtk_display(x, y)
        if not picker.Pick(vtk_x, vtk_y, 0, self.plotter.renderer):
            return None
        vtk_actor = picker.GetActor()
        if vtk_actor is None:
            return None
        return self._get_name_from_vtk_actor(vtk_actor)


    # ═══════════════════════════════════════════════════════════════
    #  UI — menus / toolbar / docks
    # ═══════════════════════════════════════════════════════════════

    # ── Menus ───────────────────────────────────────

    def _setup_menus(self):
        mb = self.menuBar()

        # ── File ──
        fm = mb.addMenu("文件 (&F)")
        fm.addAction("保存数据...", self._save_data)
        fm.addAction("载入数据...", self._load_data)
        fm.addSeparator()
        fm.addAction("截图...", self._take_screenshot)
        fm.addAction("连续截图 (录制) 开/关", self._toggle_recording)
        fm.addSeparator()
        fm.addAction("导入模型 (STL/OBJ)...", self._import_model)
        fm.addAction("导出选中模型...", self._export_selected)
        fm.addSeparator()
        fm.addAction("退出", self.close)

        # ── View ──
        vm = mb.addMenu("视角 (&V)")
        vm.addAction("俯视", lambda: self._set_view("top"))
        vm.addAction("仰视", lambda: self._set_view("bottom"))
        vm.addAction("正视 (前/X 轴)", lambda: self._set_view("front"))
        vm.addAction("侧视 (右/Y 轴)", lambda: self._set_view("side"))
        vm.addSeparator()
        vm.addAction("相机复位", self._reset_camera)
        vm.addSeparator()
        vm.addAction("全局复位 (所有对象)", self._reset_all)

        # ── Tools ──
        tm = mb.addMenu("工具 (&T)")

        self._action_meas_dist = tm.addAction("测距")
        self._action_meas_dist.setCheckable(True)
        self._action_meas_dist.triggered.connect(
            lambda checked: self._set_interaction_mode(
                InteractionMode.MEASURE_DISTANCE if checked else InteractionMode.NORMAL
            )
        )

        self._action_meas_angle = tm.addAction("测角")
        self._action_meas_angle.setCheckable(True)
        self._action_meas_angle.triggered.connect(
            lambda checked: self._set_interaction_mode(
                InteractionMode.MEASURE_ANGLE if checked else InteractionMode.NORMAL
            )
        )

        tm.addAction("清除测量", self.meas_tool.clear_all)
        tm.addAction("撤销上一步测量", self.meas_tool.undo_last)
        tm.addSeparator()
        tm.addAction("碰撞检测...", self._run_collision_check)
        tm.addSeparator()
        sm = tm.addMenu("坐标系")
        for cs in COORD_SYSTEMS:
            sm.addAction(
                f"{cs} ({COORD_SYSTEMS[cs]['label']})",
                lambda checked, c=cs: self._set_coord_system(c),
            )

        # ── Help ──
        hm = mb.addMenu("帮助 (&H)")
        hm.addAction("关于", self._show_about)

        # Register for mode-button syncing
        self._mode_buttons.append((self._action_meas_dist, InteractionMode.MEASURE_DISTANCE))
        self._mode_buttons.append((self._action_meas_angle, InteractionMode.MEASURE_ANGLE))

    # ── Toolbar ─────────────────────────────────────

    def _setup_toolbar(self):
        tb = self.addToolBar("主工具栏")
        tb.setObjectName("main_toolbar")
        tb.setToolButtonStyle(Qt.ToolButtonTextOnly)

        tb.addAction("俯视", lambda: self._set_view("top"))
        tb.addAction("侧视", lambda: self._set_view("side"))
        tb.addAction("相机复位", self._reset_camera)
        tb.addAction("全局复位", self._reset_all)
        tb.addSeparator()

        self._tb_meas_dist = tb.addAction("测距")
        self._tb_meas_dist.setCheckable(True)
        self._tb_meas_dist.triggered.connect(
            lambda checked: self._set_interaction_mode(
                InteractionMode.MEASURE_DISTANCE if checked else InteractionMode.NORMAL
            )
        )

        self._tb_meas_angle = tb.addAction("测角")
        self._tb_meas_angle.setCheckable(True)
        self._tb_meas_angle.triggered.connect(
            lambda checked: self._set_interaction_mode(
                InteractionMode.MEASURE_ANGLE if checked else InteractionMode.NORMAL
            )
        )

        tb.addSeparator()
        tb.addAction("清除测量", self.meas_tool.clear_all)
        tb.addAction("撤销测量", self.meas_tool.undo_last)
        tb.addSeparator()
        tb.addAction("截图", self._take_screenshot)
        tb.addAction("录制", self._toggle_recording)

        self._mode_buttons.append((self._tb_meas_dist, InteractionMode.MEASURE_DISTANCE))
        self._mode_buttons.append((self._tb_meas_angle, InteractionMode.MEASURE_ANGLE))

    # ── Docks ───────────────────────────────────────

    def _setup_docks(self):
        # ── Left: 图层管理 (layer mgmt, ID-18) ──
        dock_layers = QtWidgets.QDockWidget("图层管理", self)
        lm_widget = QtWidgets.QWidget()
        lm_layout = QtWidgets.QVBoxLayout(lm_widget)
        lm_layout.setContentsMargins(4, 4, 4, 4)
        lm_layout.setSpacing(3)

        # Transparency explanation: left=opaque, right=transparent
        hint = QtWidgets.QLabel("← 实心 ── 完全透明 →")
        hint.setStyleSheet("color: #888; font-size: 11px; padding: 0 4px;")
        hint.setAlignment(Qt.AlignRight)
        lm_layout.addWidget(hint)

        for layer_key, layer_label in self._terrain_layer_names.items():
            row = QtWidgets.QHBoxLayout()
            row.setSpacing(4)
            chk = QtWidgets.QCheckBox(layer_label)
            info = self.scene_objects.get(layer_key, {})
            chk.setChecked(info.get("visible", True))
            chk.toggled.connect(
                lambda checked, n=layer_key: self._toggle_terrain_layer(n, checked)
            )
            row.addWidget(chk)
            self._terrain_chks[layer_key] = chk

            slider_w, slider_setter = self._create_opacity_slider(
                0.0,
                lambda val, n=layer_key: self._on_terrain_opacity(n, val),
            )
            row.addWidget(slider_w)
            self._terrain_opacity_sliders[layer_key] = slider_w
            self._terrain_opacity_setters[layer_key] = slider_setter

            lm_layout.addLayout(row)

        dock_layers.setWidget(lm_widget)

        # ── Left: 场景对象显隐 ──
        dock_so = QtWidgets.QDockWidget("场景对象", self)
        so_widget = QtWidgets.QWidget()
        so_layout = QtWidgets.QVBoxLayout(so_widget)
        so_layout.setContentsMargins(4, 4, 4, 4)
        so_layout.setSpacing(3)

        self._scene_obj_chks = {}       # name → QCheckBox
        self._custom_obj_chks = {}      # name → QCheckBox
        self._so_layout = so_layout     # for _refresh_scene_objects_ui

        so_layout.addStretch()
        dock_so.setWidget(so_widget)

        # Register right-side docks (图层管理 above 场景对象 above 对象控制)
        self.addDockWidget(Qt.RightDockWidgetArea, dock_layers)
        self.addDockWidget(Qt.RightDockWidgetArea, dock_so)

        # ── Left: 路径规划 ──
        dock_path = QtWidgets.QDockWidget("路径规划", self)
        pw = QtWidgets.QWidget()
        pl = QtWidgets.QVBoxLayout(pw)
        pl.setContentsMargins(4, 4, 4, 4)

        pl.addWidget(QtWidgets.QLabel("点击「添加路径点」后在场景中点击"))
        self._btn_wp = QtWidgets.QPushButton("添加路径点 (3D 点击)")
        self._btn_wp.setCheckable(True)
        self._btn_wp.clicked.connect(self._on_wp_button)
        pl.addWidget(self._btn_wp)

        self._btn_precise_wp = QtWidgets.QPushButton("精准添加路径点")
        self._btn_precise_wp.clicked.connect(self._open_precise_wp_dialog)
        pl.addWidget(self._btn_precise_wp)

        pl.addWidget(QtWidgets.QLabel("— 路径操作 —"))
        self._btn_clear_wp = QtWidgets.QPushButton("清除路径")
        self._btn_clear_wp.clicked.connect(self._clear_waypoints)
        pl.addWidget(self._btn_clear_wp)

        # Waypoint counter
        self._wp_count_label = QtWidgets.QLabel("当前路径点: 0")
        pl.addWidget(self._wp_count_label)

        # ── Flight animation (ID-11) ──
        pl.addWidget(QtWidgets.QLabel("— 飞行动画 —"))
        flight_row = QtWidgets.QHBoxLayout()
        flight_row.addWidget(QtWidgets.QLabel("选择飞机:"))
        self._flight_aircraft_combo = QtWidgets.QComboBox()
        flight_row.addWidget(self._flight_aircraft_combo)
        pl.addLayout(flight_row)

        self._btn_start_flight = QtWidgets.QPushButton("开始飞行")
        self._btn_start_flight.clicked.connect(self._start_flight)
        pl.addWidget(self._btn_start_flight)

        flight_save_row = QtWidgets.QHBoxLayout()
        self._btn_save_flight = QtWidgets.QPushButton("保存飞行数据")
        self._btn_save_flight.clicked.connect(self._save_flight_data)
        flight_save_row.addWidget(self._btn_save_flight)
        self._btn_load_flight = QtWidgets.QPushButton("载入飞行数据")
        self._btn_load_flight.clicked.connect(self._load_flight_data)
        flight_save_row.addWidget(self._btn_load_flight)
        self._btn_formation = QtWidgets.QPushButton("编队飞行")
        self._btn_formation.setCheckable(True)
        self._btn_formation.setStyleSheet(
            "QPushButton:checked { background-color: #4a90d9; color: white; font-weight: bold; }"
        )
        self._btn_formation.toggled.connect(self._on_formation_toggled)
        flight_save_row.addWidget(self._btn_formation)
        pl.addLayout(flight_save_row)

        # Formation aircraft list (hidden by default)
        self._formation_list = QtWidgets.QListWidget()
        self._formation_list.setMaximumHeight(100)
        self._formation_list.setVisible(False)
        pl.addWidget(self._formation_list)

        # ── Timeline container (ID-20, hidden until flight starts) ──
        self._timeline_container = QtWidgets.QWidget()
        tl = QtWidgets.QVBoxLayout(self._timeline_container)
        tl.setContentsMargins(0, 0, 0, 0)

        tl.addWidget(QtWidgets.QLabel("— 飞行时间轴 —"))

        # Keyframe marker labels row
        self._keyframe_widget = QtWidgets.QWidget()
        self._kf_layout = QtWidgets.QHBoxLayout(self._keyframe_widget)
        self._kf_layout.setContentsMargins(0, 0, 0, 0)
        tl.addWidget(self._keyframe_widget)

        # Time label
        self._tl_time_label = QtWidgets.QLabel("0.0 / 0.0 秒")
        tl.addWidget(self._tl_time_label)

        # Slider
        self._tl_slider = QtWidgets.QSlider(Qt.Horizontal)
        self._tl_slider.setRange(0, 1000)
        self._tl_slider.sliderPressed.connect(self._on_tl_pressed)
        self._tl_slider.valueChanged.connect(self._on_tl_seek)
        self._tl_slider.sliderReleased.connect(self._on_tl_released)
        tl.addWidget(self._tl_slider)

        pl.addWidget(self._timeline_container)

        pl.addStretch()
        dock_path.setWidget(pw)
        self.addDockWidget(Qt.LeftDockWidgetArea, dock_path)

        # ── Right: 对象控制 ──
        dock_ctrl = QtWidgets.QDockWidget("对象控制", self)
        cw = QtWidgets.QWidget()
        cl = QtWidgets.QVBoxLayout(cw)
        cl.setContentsMargins(4, 4, 4, 4)

        cl.addWidget(QtWidgets.QLabel("选中对象:"))
        self._obj_combo = QtWidgets.QComboBox()
        self._obj_combo.currentIndexChanged.connect(self._on_obj_select_changed)
        cl.addWidget(self._obj_combo)

        cl.addWidget(QtWidgets.QLabel("位置 X"))
        self._slider_obj_x = self._make_slider(-15, 15, 0, self._on_obj_pos_x)
        cl.addWidget(self._slider_obj_x)

        cl.addWidget(QtWidgets.QLabel("位置 Y"))
        self._slider_obj_y = self._make_slider(-15, 15, 0, self._on_obj_pos_y)
        cl.addWidget(self._slider_obj_y)

        cl.addWidget(QtWidgets.QLabel("位置 Z"))
        self._slider_obj_z = self._make_slider(-15, 15, 0, self._on_obj_pos_z)
        cl.addWidget(self._slider_obj_z)

        cl.addWidget(QtWidgets.QLabel("缩放"))
        self._slider_obj_s = self._make_slider(0.1, 5.0, 1.0, self._on_obj_scale)
        cl.addWidget(self._slider_obj_s)

        # ── Aircraft attitude (hidden for non-aircraft, ID-5) ──
        self._attitude_container = QtWidgets.QWidget()
        ac = QtWidgets.QVBoxLayout(self._attitude_container)
        ac.setContentsMargins(0, 0, 0, 0)
        self._s_yaw_lbl = QtWidgets.QLabel("航向角 (Yaw)")
        self._slider_obj_yaw = self._make_slider(0.0, 360.0, 0.0, self._on_obj_yaw)
        ac.addWidget(self._s_yaw_lbl)
        ac.addWidget(self._slider_obj_yaw)

        self._s_pitch_lbl = QtWidgets.QLabel("俯仰角 (Pitch)")
        self._slider_obj_pitch = self._make_slider(-90.0, 90.0, 0.0, self._on_obj_pitch)
        ac.addWidget(self._s_pitch_lbl)
        ac.addWidget(self._slider_obj_pitch)

        self._s_roll_lbl = QtWidgets.QLabel("滚转角 (Roll)")
        self._slider_obj_roll = self._make_slider(-180.0, 180.0, 0.0, self._on_obj_roll)
        ac.addWidget(self._s_roll_lbl)
        ac.addWidget(self._slider_obj_roll)
        self._attitude_container.hide()
        cl.addWidget(self._attitude_container)

        cl.addStretch()
        dock_ctrl.setWidget(cw)
        self.addDockWidget(Qt.RightDockWidgetArea, dock_ctrl)



        # ── Bottom: 坐标信息 ──
        dock_info = QtWidgets.QDockWidget("坐标信息", self)
        self._info_text = QtWidgets.QTextEdit()
        self._info_text.setReadOnly(True)
        self._info_text.setMaximumHeight(150)
        self._info_text.setFont(QtGui.QFont("Menlo", 10))
        dock_info.setWidget(self._info_text)
        self.addDockWidget(Qt.BottomDockWidgetArea, dock_info)

    # ── Slider factory ─────────────────────────────

    @staticmethod
    def _make_slider(vmin, vmax, initial, callback, steps=1000):
        """Create a horizontal slider with live value label.

        Returns the container widget (use ``.findChild(QtWidgets.QSlider)``
        to access the slider if needed).
        """
        w = QtWidgets.QWidget()
        lo = QtWidgets.QHBoxLayout(w)
        lo.setContentsMargins(0, 0, 0, 0)
        s = QtWidgets.QSlider(Qt.Horizontal)
        s.setRange(0, steps)
        frac = (initial - vmin) / (vmax - vmin)
        s.setValue(int(frac * steps))
        label = QtWidgets.QLabel(f"{initial:.2f}")
        label.setMinimumWidth(45)
        lo.addWidget(s, 1)
        lo.addWidget(label)

        def on_change(val):
            f = val / float(steps)
            real = vmin + f * (vmax - vmin)
            label.setText(f"{real:.2f}")
            callback(real)

        s.valueChanged.connect(on_change)
        return w

    def _slider_value(self, slider_widget):
        """Read the current float value from a slider widget made by ``_make_slider``."""
        s = slider_widget.findChild(QtWidgets.QSlider)
        if s is None:
            return 0.0
        return s.value() / 1000.0

    # ── Opacity slider factory (ID-18) ─────────────

    @staticmethod
    def _create_opacity_slider(initial_opacity, callback):
        """Create an opacity slider (0% → 1.0 transparent, 100% → 1.0 opaque).

        Returns ``(widget, setter_func)`` where *setter_func(v)* updates
        the slider display without triggering *callback*.
        """
        w = QtWidgets.QWidget()
        lo = QtWidgets.QHBoxLayout(w)
        lo.setContentsMargins(0, 0, 0, 0)
        s = QtWidgets.QSlider(Qt.Horizontal)
        s.setRange(0, 100)
        s.setValue(int(initial_opacity * 100))
        label = QtWidgets.QLabel(f"{int(initial_opacity * 100)}%")
        label.setMinimumWidth(35)
        label.setAlignment(Qt.AlignRight | Qt.AlignVCenter)
        lo.addWidget(s, 1)
        lo.addWidget(label)

        def on_change(val):
            t = val  # t = 0..100, 0=opaque, 100=transparent
            label.setText(f"{t}%透明")
            # Convert transparency to opacity for the callback
            opacity = (100 - t) / 100.0
            callback(opacity)

        def setter(val):
            """Set opacity directly: val is opacity [0..1]."""
            t = int((1.0 - val) * 100)  # Convert opacity → transparency
            s.blockSignals(True)
            s.setValue(t)
            s.blockSignals(False)
            label.setText(f"{t}%透明")

        s.valueChanged.connect(on_change)
        return w, setter

    # ── Terrain layer controls (ID-18) ─────────────

    def _toggle_terrain_layer(self, name, visible):
        """Show / hide a terrain layer actor."""
        info = self.scene_objects.get(name)
        if info is None:
            return
        info["visible"] = visible
        if visible:
            self._rebuild_actor(name)
        else:
            self._remove_actor(name)
        self.plotter.render()

    def _toggle_layer(self, name, visible):
        """Show / hide a built-in scene object (tree, bird, …)."""
        info = self.scene_objects.get(name)
        if info is None:
            return
        info["visible"] = visible
        if visible:
            self._rebuild_actor(name)
            self._apply_obj_transform_to_actor(name)
        else:
            self._remove_actor(name)
        self.plotter.render()

    def _toggle_custom(self, name, visible):
        """Show / hide an imported custom object."""
        mesh = self.custom_objects.get(name)
        if mesh is None:
            return
        actor_name = f"_custom_{name}"
        if visible:
            actor = self.plotter.add_mesh(mesh, color="gold", style="wireframe", line_width=2, opacity=0.8)
            self.plotter_actors[actor_name] = actor
            self._register_actor_reverse_lookup(actor, actor_name)
            self._tag_mesh(mesh, actor_name)
        else:
            self._remove_actor(actor_name)
        self.plotter.render()

    def _on_terrain_opacity(self, name, value):
        """Update a terrain layer's opacity and persist to scene_objects params."""
        if name not in self.plotter_actors:
            return
        actor = self.plotter_actors[name]
        vtk_actor = self._resolve_vtk_actor(actor)
        vtk_actor.GetProperty().SetOpacity(value)
        # Persist so rebuilds (toggle off/on) keep the user's setting
        info = self.scene_objects.get(name)
        if info is not None:
            info["params"]["opacity"] = value
        self.plotter.render()

    def _refresh_terrain_ui(self):
        """Sync terrain layer checkboxes & sliders with current scene state."""
        for name, chk in self._terrain_chks.items():
            info = self.scene_objects.get(name)
            if info is not None:
                chk.blockSignals(True)
                chk.setChecked(info["visible"])
                chk.blockSignals(False)
        for name, setter in self._terrain_opacity_setters.items():
            actor = self.plotter_actors.get(name)
            if actor is not None:
                vtk_actor = self._resolve_vtk_actor(actor)
                op = vtk_actor.GetProperty().GetOpacity()
                setter(op)

    def _refresh_scene_objects_ui(self):
        """Rebuild the per-object visibility checkboxes in the 场景对象 dock."""
        # Clear existing checkboxes (keep stretch at the end)
        lo = self._so_layout
        for i in range(lo.count() - 1, -1, -1):
            item = lo.itemAt(i)
            if item.widget() is not None:
                w = item.widget()
                lo.removeWidget(w)
                w.deleteLater()

        terrain_names = set(self._terrain_layer_names.keys())
        scene_items = []
        for name in self.scene_objects:
            if name == "terrain" or name in terrain_names:
                continue
            label = name
            if name.startswith("layer_"):
                continue
            scene_items.append((name, label, False))
        for name in self.custom_objects:
            scene_items.append((name, name, True))

        if not scene_items:
            lbl = QtWidgets.QLabel("(无场景对象)")
            lbl.setStyleSheet("color: #888; font-size: 11px; padding: 4px;")
            lo.insertWidget(lo.count() - 1, lbl)
            return

        new_chks = {}
        new_custom_chks = {}
        for name, label, is_custom in scene_items:
            row = QtWidgets.QHBoxLayout()
            row.setSpacing(4)
            chk = QtWidgets.QCheckBox(label)
            info = self.scene_objects.get(name)
            if is_custom:
                chk.setChecked(name in self.plotter_actors)
                new_custom_chks[name] = chk
                chk.toggled.connect(lambda checked, n=name: self._toggle_custom(n, checked))
            else:
                chk.setChecked(info.get("visible", True) if info else True)
                new_chks[name] = chk
                chk.toggled.connect(lambda checked, n=name: self._toggle_layer(n, checked))
            row.addWidget(chk)
            row.addStretch()
            lo.insertLayout(lo.count() - 1, row)

        self._scene_obj_chks = new_chks
        self._custom_obj_chks = new_custom_chks

    def _rebuild_terrain_layers(self):
        """Re-create sand/grass/earth layer meshes from the terrain grid.

        Called after terrain elevation data is loaded so the thresholded
        layers reflect the new Z values.
        """
        terrain_info = self.scene_objects.get("terrain")
        if terrain_info is None:
            return
        grid = terrain_info["mesh"]
        extra = terrain_info.get("extra")
        if extra is None:
            return
        Z = extra.get("original_z")
        if Z is None:
            return

        for name in ["layer_sand", "layer_grass", "layer_earth"]:
            self._remove_actor(name)
            self.scene_objects.pop(name, None)

        from src.scene_builder import build_terrain_layer_meshes
        new_layers = build_terrain_layer_meshes(grid, Z)
        for name, obj in new_layers.items():
            self.scene_objects[name] = obj
            if obj["visible"]:
                self._add_actor(name, obj)

        self._refresh_terrain_ui()
        self.plotter.render()


    # ═══════════════════════════════════════════════════════════════
    #  Layer / tree panels
    # ═══════════════════════════════════════════════════════════════

    def _refresh_ui(self):
        """Refresh flight combo, terrain UI, and scene-object checkboxes."""
        self._refresh_obj_combo()
        self._refresh_flight_combo()
        self._refresh_terrain_ui()
        self._refresh_scene_objects_ui()

    def _refresh_obj_combo(self):
        """Rebuild the object-control combo box."""
        current = self._obj_combo.currentText()
        self._obj_combo.blockSignals(True)
        self._obj_combo.clear()
        for name in self.scene_objects:
            self._obj_combo.addItem(name)
        for name in self.custom_objects:
            self._obj_combo.addItem(f"[自定义] {name}")
        idx = self._obj_combo.findText(current)
        if idx >= 0:
            self._obj_combo.setCurrentIndex(idx)
        self._obj_combo.blockSignals(False)


    # ═══════════════════════════════════════════════════════════════
    #  Object control (position + scale via VTK UserTransform)
    # ═══════════════════════════════════════════════════════════════

    def _get_or_init_transform(self, name):
        """Return the transform dict for *name*, initialising if needed."""
        if name not in self._obj_transforms:
            # Compute initial center from mesh
            info = self.scene_objects.get(name)
            mesh = info["mesh"] if info else None
            if mesh is None:
                mesh = self.custom_objects.get(name)
            if mesh is not None and mesh.n_points:
                center = np.mean(mesh.points, axis=0)
            else:
                center = np.zeros(3)
            self._obj_transforms[name] = {
                "offset": center.tolist(),
                "orig_center": center.tolist(),
                "yaw": 0.0,
                "pitch": 0.0,
                "roll": 0.0,
                "scale": 1.0,
            }
        return self._obj_transforms[name]

    def _compute_obj_center(self, name):
        """Return the base (pre-transform) center of an object's mesh."""
        info = self.scene_objects.get(name)
        mesh = info["mesh"] if info else None
        if mesh is None:
            mesh = self.custom_objects.get(name)
        if mesh is not None and mesh.n_points:
            return np.mean(mesh.points, axis=0)
        return np.zeros(3)

    def _on_obj_select_changed(self, idx):
        """Update slider positions to match the newly-selected object's transform."""
        name = self._obj_combo.currentText()
        if not name:
            return
        # Strip [自定义] prefix for lookup
        clean_name = name.replace("[自定义] ", "")
        if clean_name not in self.scene_objects and clean_name not in self.custom_objects:
            return
        # Check if we have a transform; if not, init from mesh center
        if clean_name not in self._obj_transforms:
            center = self._compute_obj_center(clean_name)
            self._obj_transforms[clean_name] = {
                "offset": center.tolist(),
                "orig_center": center.tolist(),
                "yaw": 0.0,
                "pitch": 0.0,
                "roll": 0.0,
                "scale": 1.0,
            }
        t = self._obj_transforms[clean_name]
        self._set_slider_value(self._slider_obj_x, t["offset"][0])
        self._set_slider_value(self._slider_obj_y, t["offset"][1])
        self._set_slider_value(self._slider_obj_z, t["offset"][2])
        self._set_slider_value(self._slider_obj_s, t["scale"])

        # Show attitude sliders only for aircraft (ID-5)
        is_aircraft = "aircraft" in clean_name.lower()
        self._attitude_container.setVisible(is_aircraft)
        if is_aircraft:
            self._set_slider_value(self._slider_obj_yaw, t.get("yaw", 0.0))
            self._set_slider_value(self._slider_obj_pitch, t.get("pitch", 0.0))
            self._set_slider_value(self._slider_obj_roll, t.get("roll", 0.0))

        # Apply transform immediately so rotation is in effect from the
        # moment the object is selected, not only after a slider move (ID-6).
        self._apply_obj_transform_to_actor(clean_name)

    def _set_slider_value(self, slider_widget, val):
        """Set a slider widget to a given value without triggering its callback."""
        s = slider_widget.findChild(QtWidgets.QSlider)
        if s is None:
            return
        s.blockSignals(True)
        vmin, vmax = -15, 15  # default range, will be overridden per slider
        if slider_widget is self._slider_obj_s:
            vmin, vmax = 0.1, 5.0
        elif slider_widget is self._slider_obj_yaw:
            vmin, vmax = 0.0, 360.0
        elif slider_widget is self._slider_obj_pitch:
            vmin, vmax = -90.0, 90.0
        elif slider_widget is self._slider_obj_roll:
            vmin, vmax = -180.0, 180.0
        frac = (val - vmin) / (vmax - vmin)
        s.setValue(int(frac * 1000))
        s.blockSignals(False)
        label = slider_widget.findChild(QtWidgets.QLabel)
        if label:
            label.setText(f"{val:.2f}")

    def _apply_obj_transform_to_actor(self, name):
        """Apply the stored transform for *name* to its VTK actor.

        Builds the 4×4 homogeneous matrix explicitly using numpy so the
        transform is independent of VTK's PreMultiply/PostMultiply mode.
        The effective per-point operation is::

            p' = offset  +  Rz(yaw)·Ry(pitch)·Rx(roll) · scale · (p - orig_center)

        This is intrinsic Z-Y-X Euler rotation (yaw→pitch→roll, standard
        aerospace convention).  Rotation always occurs around the object's
        own geometric centre (orig_center) — even after the object has been
        moved via the position sliders (ID-6).
        """
        if name not in self._obj_transforms:
            return
        if name not in self.plotter_actors:
            return
        t = self._obj_transforms[name]
        actor = self.plotter_actors[name]
        vtk_actor = self._resolve_vtk_actor(actor)

        offset = np.array(t["offset"], dtype=float)
        orig_center = np.array(t.get("orig_center", offset), dtype=float)
        s = float(t["scale"])
        yaw = float(t.get("yaw", 0.0))
        pitch = float(t.get("pitch", 0.0))
        roll = float(t.get("roll", 0.0))

        # --- build rotation matrices (right-hand rule) ---
        yaw_r = np.radians(yaw)
        pitch_r = np.radians(pitch)
        roll_r = np.radians(roll)

        # Roll  — rotation around local X
        Rx = np.array([
            [1, 0, 0],
            [0, np.cos(roll_r), -np.sin(roll_r)],
            [0, np.sin(roll_r), np.cos(roll_r)],
        ])
        # Pitch — rotation around local Y
        Ry = np.array([
            [np.cos(pitch_r), 0, np.sin(pitch_r)],
            [0, 1, 0],
            [-np.sin(pitch_r), 0, np.cos(pitch_r)],
        ])
        # Yaw   — rotation around local Z
        Rz = np.array([
            [np.cos(yaw_r), -np.sin(yaw_r), 0],
            [np.sin(yaw_r), np.cos(yaw_r), 0],
            [0, 0, 1],
        ])

        # Intrinsic Z-Y-X Euler: R = Rz(yaw) @ Ry(pitch) @ Rx(roll)
        R = Rz @ Ry @ Rx

        # --- build 4×4 homogeneous matrix ---
        #   p' = offset  +  R·s · (p - orig_center)
        #       = (R·s)·p  +  (offset - R·s·orig_center)
        H = np.eye(4)
        H[:3, :3] = R * s
        H[:3, 3] = offset - (R * s) @ orig_center

        # --- write into VTK matrix and set as UserTransform ---
        vtk_matrix = vtk.vtkMatrix4x4()
        for i in range(4):
            for j in range(4):
                vtk_matrix.SetElement(i, j, float(H[i, j]))

        transform = vtk.vtkTransform()
        transform.SetMatrix(vtk_matrix)
        vtk_actor.SetUserTransform(transform)
        self.plotter.render()

    def _on_obj_pos_x(self, val):
        name = self._obj_combo.currentText()
        clean = name.replace("[自定义] ", "")
        if clean not in self._obj_transforms:
            return
        self._obj_transforms[clean]["offset"][0] = val
        self._apply_obj_transform_to_actor(clean)

    def _on_obj_pos_y(self, val):
        name = self._obj_combo.currentText()
        clean = name.replace("[自定义] ", "")
        if clean not in self._obj_transforms:
            return
        self._obj_transforms[clean]["offset"][1] = val
        self._apply_obj_transform_to_actor(clean)

    def _on_obj_pos_z(self, val):
        name = self._obj_combo.currentText()
        clean = name.replace("[自定义] ", "")
        if clean not in self._obj_transforms:
            return
        self._obj_transforms[clean]["offset"][2] = val
        self._apply_obj_transform_to_actor(clean)

    def _on_obj_scale(self, val):
        name = self._obj_combo.currentText()
        clean = name.replace("[自定义] ", "")
        if clean not in self._obj_transforms:
            return
        self._obj_transforms[clean]["scale"] = val
        self._apply_obj_transform_to_actor(clean)

    def _on_obj_yaw(self, val):
        name = self._obj_combo.currentText()
        clean = name.replace("[自定义] ", "")
        if clean not in self._obj_transforms:
            return
        self._obj_transforms[clean]["yaw"] = val
        self._apply_obj_transform_to_actor(clean)

    def _on_obj_pitch(self, val):
        name = self._obj_combo.currentText()
        clean = name.replace("[自定义] ", "")
        if clean not in self._obj_transforms:
            return
        self._obj_transforms[clean]["pitch"] = val
        self._apply_obj_transform_to_actor(clean)

    def _on_obj_roll(self, val):
        name = self._obj_combo.currentText()
        clean = name.replace("[自定义] ", "")
        if clean not in self._obj_transforms:
            return
        self._obj_transforms[clean]["roll"] = val
        self._apply_obj_transform_to_actor(clean)


    # ═══════════════════════════════════════════════════════════════
    #  3D Interaction callbacks (Qt-level mouse events)
    # ═══════════════════════════════════════════════════════════════

    def _on_3d_click(self, x, y, world_pos, vtk_actor):
        """Route a 3D viewport click based on the current interaction mode."""
        mode = self._current_mode

        if mode == InteractionMode.NORMAL:
            # Object selection
            if vtk_actor is not None:
                name = self._get_name_from_vtk_actor(vtk_actor)
                if name:
                    self._select_object(name)
                    return
            self._clear_highlight()

        elif mode in (InteractionMode.MEASURE_DISTANCE, InteractionMode.MEASURE_ANGLE):
            if np.linalg.norm(world_pos) < 1e-6:
                return
            self.meas_tool.add_point(world_pos)

        elif mode == InteractionMode.WAYPOINT:
            if np.linalg.norm(world_pos) < 1e-6:
                return
            self._add_waypoint(world_pos)

    def _on_3d_move(self, x, y):
        """Update coordinate info on mouse move."""
        picker = vtk.vtkWorldPointPicker()
        vtk_x, vtk_y = self.plotter._to_vtk_display(x, y)
        picker.Pick(vtk_x, vtk_y, 0, self.plotter.renderer)
        world = np.array(picker.GetPickPosition())
        self._update_info(world)

    # ── Mode-switching controller ──────────────────

    def _set_interaction_mode(self, mode):
        """Central mode-switch — updates mode, UI, and cursor."""
        old_mode = self._current_mode
        self._current_mode = mode

        # Sync checkable buttons
        for btn, btn_mode in self._mode_buttons:
            btn.setChecked(btn_mode == mode and mode != InteractionMode.NORMAL)

        # Sync waypoint button
        self._btn_wp.setChecked(mode == InteractionMode.WAYPOINT)

        # Measurement mode internal
        if mode in (InteractionMode.MEASURE_DISTANCE, InteractionMode.MEASURE_ANGLE):
            self.meas_tool.set_mode(
                "distance" if mode == InteractionMode.MEASURE_DISTANCE else "angle"
            )
            self.setCursor(QtGui.QCursor(Qt.CrossCursor))
            self.statusBar().showMessage(
                "测量模式 — 点击 3D 场景放置测量点", 5000
            )
        elif mode == InteractionMode.WAYPOINT:
            self.setCursor(QtGui.QCursor(Qt.CrossCursor))
            self.statusBar().showMessage(
                "路径点模式 — 点击 3D 场景添加路径点", 5000
            )
        else:
            self.setCursor(QtGui.QCursor(Qt.ArrowCursor))
            self.statusBar().showMessage(
                "就绪  |  左键旋转 · 滚轮缩放 · 中键平移", 5000
            )

    def _on_wp_button(self, checked):
        """Bridge from waypoint button to mode switch."""
        self._set_interaction_mode(
            InteractionMode.WAYPOINT if checked else InteractionMode.NORMAL
        )


    # ═══════════════════════════════════════════════════════════════
    #  View & coordinate system
    # ═══════════════════════════════════════════════════════════════

    def _set_view(self, direction):
        p = self.plotter
        fp = p.camera.focal_point
        dist = 25
        views = {
            "top":    ((0, 0, dist), (0, 0, 0), (0, 1, 0)),
            "bottom": ((0, 0, -dist), (0, 0, 0), (0, 1, 0)),
            "front":  ((dist, 0, 0), (0, 0, 0), (0, 0, 1)),
            "side":   ((0, -dist, 0), (0, 0, 0), (0, 0, 1)),
        }
        if direction in views:
            p.camera_position = views[direction]
            p.render()

    def _reset_camera(self):
        """Reset only the camera to default position (ID-7)."""
        p = self.plotter
        p.camera_position = [(18, -16, 8), (0, 0, 2), (0, 0, 1)]
        p.camera.focal_point = (0, 0, 1.5)
        p.render()

    def _reset_all(self):
        """Reset all objects to their initial transforms and reset camera (ID-7)."""
        for name in list(self._obj_transforms.keys()):
            center = self._compute_obj_center(name)
            self._obj_transforms[name] = {
                "offset": center.tolist(),
                "orig_center": center.tolist(),
                "yaw": 0.0,
                "pitch": 0.0,
                "roll": 0.0,
                "scale": 1.0,
            }
            self._apply_obj_transform_to_actor(name)
        self._refresh_obj_combo()
        self._reset_camera()

    def _set_coord_system(self, cs):
        self.coord_system = cs
        self.config["coordinate_system"] = cs
        self.statusBar().showMessage(
            f"坐标系: {cs}  ", 3000
        )
        self._update_info()


    # ═══════════════════════════════════════════════════════════════
    #  File I/O
    # ═══════════════════════════════════════════════════════════════

    # ── Scene save / load ──────────────────────────

    def _save_scene(self):
        path, _ = QtWidgets.QFileDialog.getSaveFileName(
            self, "保存场景", "", "JSON (*.json)"
        )
        if not path:
            return
        data = {
            "config": self.config,
            "camera": list(self.plotter.camera_position),
            "waypoints": [wp.tolist() for wp in self.waypoints],
            "obj_transforms": self._obj_transforms,
        }
        with open(path, "w") as f:
            json.dump(data, f, indent=2)
        self.statusBar().showMessage(f"场景已保存: {path}", 3000)

    def _load_scene(self):
        path, _ = QtWidgets.QFileDialog.getOpenFileName(
            self, "加载场景", "", "JSON (*.json)"
        )
        if not path:
            return
        try:
            with open(path) as f:
                data = json.load(f)
        except Exception as e:
            QtWidgets.QMessageBox.warning(self, "错误", f"加载失败: {e}")
            return
        self.config.update(data.get("config", {}))
        # Sync coordinate system after loading (ID-3)
        loaded_cs = self.config.get("coordinate_system", "ENU")
        if loaded_cs != self.coord_system:
            self._set_coord_system(loaded_cs)
        if "camera" in data:
            self.plotter.camera_position = tuple(data["camera"])
        self.waypoints = [np.array(wp) for wp in data.get("waypoints", [])]
        # Restore object transforms
        loaded_transforms = data.get("obj_transforms", {})
        for name, t in loaded_transforms.items():
            if name in self.scene_objects or name in self.custom_objects:
                self._obj_transforms[name] = t
                self._apply_obj_transform_to_actor(name)
        self._refresh_obj_combo()
        self.plotter.render()
        self.statusBar().showMessage(f"场景已加载: {path}", 3000)

    # ── Data save / load (aircraft + terrain, separate JSONs) ──

    def _save_data(self):
        """Save aircraft transforms + terrain data as a matched JSON pair.

        Aircraft data → ``data/aircraft/<name>.json``
        Terrain data  → ``data/terrain/<name>.json``
        Both share the same *name* entered by the user.
        """
        name, ok = QtWidgets.QInputDialog.getText(
            self, "保存数据", "请输入数据名称 (例如 mission1):"
        )
        if not ok or not name:
            return
        name = name.strip()
        if not name:
            return

        base_dir = os.path.dirname(os.path.abspath(__file__))
        aircraft_dir = os.path.join(base_dir, "..", self.SAVE_DIR_AIRCRAFT)
        terrain_dir = os.path.join(base_dir, "..", self.SAVE_DIR_TERRAIN)
        os.makedirs(aircraft_dir, exist_ok=True)
        os.makedirs(terrain_dir, exist_ok=True)

        # ── Collect aircraft transforms ──
        aircraft_data = {}
        for obj_name in self.scene_objects:
            if "aircraft" in obj_name.lower():
                t = self._get_or_init_transform(obj_name)
                aircraft_data[obj_name] = dict(t)
        for obj_name in self.custom_objects:
            if "aircraft" in obj_name.lower():
                t = self._get_or_init_transform(obj_name)
                aircraft_data[obj_name] = dict(t)

        if not aircraft_data:
            QtWidgets.QMessageBox.warning(self, "警告", "场景中没有包含 \"aircraft\" 的物体")
            return

        aircraft_path = os.path.join(aircraft_dir, f"{name}.json")
        with open(aircraft_path, "w") as f:
            json.dump(aircraft_data, f, indent=2, ensure_ascii=False)

        # ── Collect terrain data (all non-aircraft objects) ──
        terrain_info = self.scene_objects.get("terrain")
        if terrain_info is None:
            QtWidgets.QMessageBox.warning(self, "警告", "场景中不存在地形")
            return
        extra = terrain_info.get("extra")
        if extra is None or "original_z" not in extra:
            QtWidgets.QMessageBox.warning(self, "警告", "地形数据不完整 (缺少 original_z)")
            return

        # Save transforms for ALL non-aircraft objects (river, vegetation, bird, tree, custom, etc.)
        scene_transforms = {}
        for obj_name in self.scene_objects:
            if "aircraft" not in obj_name.lower():
                t = self._get_or_init_transform(obj_name)
                scene_transforms[obj_name] = dict(t)
        for obj_name in self.custom_objects:
            if "aircraft" not in obj_name.lower():
                t = self._get_or_init_transform(obj_name)
                scene_transforms[obj_name] = dict(t)

        terrain_data = {
            "original_z": extra["original_z"].tolist(),
            "config": dict(self.config),
            "camera": list(self.plotter.camera_position),
            "objects": scene_transforms,
        }

        terrain_path = os.path.join(terrain_dir, f"{name}.json")
        with open(terrain_path, "w") as f:
            json.dump(terrain_data, f, indent=2, ensure_ascii=False)

        self.statusBar().showMessage(
            f"数据已保存: {name} (飞行器 → {aircraft_path}, 地形 → {terrain_path})",
            5000,
        )

    def _load_data(self):
        """Load aircraft OR terrain data from a JSON file (never both).

        The user picks a JSON from ``data/aircraft/`` or ``data/terrain/``;
        only the corresponding data type is restored.
        """
        base_dir = os.path.dirname(os.path.abspath(__file__))
        data_dir = os.path.join(base_dir, "..", "data")
        aircraft_dir = os.path.join(base_dir, "..", self.SAVE_DIR_AIRCRAFT)
        terrain_dir = os.path.join(base_dir, "..", self.SAVE_DIR_TERRAIN)

        if not os.path.isdir(data_dir):
            QtWidgets.QMessageBox.warning(self, "错误", f"数据目录不存在: {data_dir}")
            return

        path, _ = QtWidgets.QFileDialog.getOpenFileName(
            self, "载入数据 (选择 aircraft 或 terrain JSON)", data_dir, "JSON (*.json)"
        )
        if not path:
            return

        base_name = os.path.splitext(os.path.basename(path))[0]
        path_dir = os.path.dirname(os.path.abspath(path))

        # ── Determine data type from the selected file's directory ──
        abs_aircraft_dir = os.path.abspath(aircraft_dir)
        abs_terrain_dir = os.path.abspath(terrain_dir)

        if path_dir == abs_aircraft_dir:
            self._load_aircraft_data(path, base_name)
        elif path_dir == abs_terrain_dir:
            self._load_terrain_data(path, base_name)
        else:
            QtWidgets.QMessageBox.warning(
                self, "错误",
                f"所选文件不在 data/aircraft/ 或 data/terrain/ 目录下。\n"
                f"请从正确的目录选择文件。"
            )

    def _load_aircraft_data(self, path, base_name):
        """Restore aircraft transforms from a JSON file."""
        try:
            with open(path) as f:
                aircraft_data = json.load(f)
        except Exception as e:
            QtWidgets.QMessageBox.warning(self, "错误", f"加载飞行器数据失败: {e}")
            return

        for obj_name, t in aircraft_data.items():
            if obj_name in self.scene_objects or obj_name in self.custom_objects:
                self._obj_transforms[obj_name] = t
                self._apply_obj_transform_to_actor(obj_name)

        self._refresh_obj_combo()
        self._refresh_flight_combo()
        self._on_obj_select_changed(self._obj_combo.currentIndex())
        self.plotter.render()
        self.statusBar().showMessage(f"飞行器数据已载入: {base_name}", 5000)

    def _load_terrain_data(self, path, base_name):
        """Restore terrain mesh + all non-aircraft objects + config + camera."""
        try:
            with open(path) as f:
                terrain_data = json.load(f)
        except Exception as e:
            QtWidgets.QMessageBox.warning(self, "错误", f"加载地形数据失败: {e}")
            return

        # Restore terrain mesh elevation
        terrain_info = self.scene_objects.get("terrain")
        if terrain_info is not None:
            mesh = terrain_info["mesh"]
            extra = terrain_info["extra"]
            Z_2d = np.array(terrain_data["original_z"])
            extra["original_z"] = Z_2d

            pts = mesh.points
            pts[:, 2] = Z_2d.flatten(order="F")
            mesh.points = pts
            mesh["elevation"] = Z_2d.flatten(order="F")
            self._rebuild_actor("terrain")
            self._rebuild_terrain_layers()

        # Restore transforms for ALL non-aircraft objects
        objects_data = terrain_data.get("objects", {})
        for obj_name, t in objects_data.items():
            if obj_name in self.scene_objects or obj_name in self.custom_objects:
                self._obj_transforms[obj_name] = t
                self._apply_obj_transform_to_actor(obj_name)

        # Restore config
        self.config.update(terrain_data.get("config", {}))
        loaded_cs = self.config.get("coordinate_system", "ENU")
        if loaded_cs != self.coord_system:
            self._set_coord_system(loaded_cs)

        # Restore camera
        if "camera" in terrain_data:
            self.plotter.camera_position = tuple(terrain_data["camera"])

        self._refresh_obj_combo()
        self.plotter.render()
        self.statusBar().showMessage(f"场景数据已载入 (excl. aircraft): {base_name}", 5000)

    # ── Screenshot ─────────────────────────────────

    def _take_screenshot(self):
        path, _ = QtWidgets.QFileDialog.getSaveFileName(
            self, "保存截图", "", "PNG (*.png);;JPG (*.jpg)"
        )
        if path:
            self.plotter.screenshot(path)
            self.statusBar().showMessage(f"截图已保存: {path}", 3000)

    # ── Continuous recording ───────────────────────

    def _toggle_recording(self):
        if not self._recording:
            self._recording = True
            self._frame_count = 0
            ts = time.strftime("%Y%m%d_%H%M%S")
            self._rec_dir = os.path.join(
                os.path.dirname(os.path.abspath(__file__)),
                "..",
                f"_recording_{ts}",
            )
            os.makedirs(self._rec_dir, exist_ok=True)
            self._rec_timer.start(200)
            self.statusBar().showMessage(
                f"🎥 录制中 → {self._rec_dir}", 5000
            )
        else:
            self._recording = False
            self._rec_timer.stop()
            self.statusBar().showMessage(
                f"⏹ 录制停止，共 {self._frame_count} 帧", 5000
            )

    def _capture_frame(self):
        if not self._recording:
            return
        self._frame_count += 1
        fname = f"frame_{self._frame_count:05d}.png"
        fpath = os.path.join(self._rec_dir, fname)
        try:
            self.plotter.screenshot(fpath)
        except Exception as e:
            print(f"[recording] frame capture failed: {e}")
            self._rec_timer.stop()
            self._recording = False

    # ── Import / Export ────────────────────────────

    def _import_model(self):
        path, _ = QtWidgets.QFileDialog.getOpenFileName(
            self, "导入模型", "",
            "模型文件 (*.stl *.obj);;STL (*.stl);;OBJ (*.obj)"
        )
        if not path:
            return
        try:
            mesh = pv.read(path)
        except Exception as e:
            QtWidgets.QMessageBox.warning(self, "错误", f"导入失败: {e}")
            return
        name = os.path.splitext(os.path.basename(path))[0]
        orig = name
        idx = 1
        while name in self.custom_objects:
            name = f"{orig}_{idx}"
            idx += 1
        self.custom_objects[name] = mesh
        actor_name = f"_custom_{name}"
        actor = self.plotter.add_mesh(
            mesh, color="gold", style="wireframe", line_width=2, opacity=0.8
        )
        self.plotter_actors[actor_name] = actor
        self._register_actor_reverse_lookup(actor, actor_name)
        self._tag_mesh(mesh, actor_name)
        self.plotter.render()
        self.statusBar().showMessage(f"已导入: {name}", 3000)

    def _export_selected(self):
        if not self.selected_name:
            QtWidgets.QMessageBox.information(
                self, "提示", "请先在场景树或 3D 视口中选中一个模型"
            )
            return
        name = self.selected_name
        mesh = None
        for n, info in self.scene_objects.items():
            if n == name:
                mesh = info["mesh"]
                break
        if mesh is None and name in self.custom_objects:
            mesh = self.custom_objects[name]
        if mesh is None:
            for n, m in self.custom_objects.items():
                if f"[模型] {n}" == name or f"_custom_{n}" == name:
                    mesh = m
                    break
        if mesh is None:
            QtWidgets.QMessageBox.warning(self, "错误", "未找到选中模型的数据")
            return
        path, _ = QtWidgets.QFileDialog.getSaveFileName(
            self, "导出模型", name, "STL (*.stl);;OBJ (*.obj);;VTK (*.vtp)"
        )
        if not path:
            return
        try:
            mesh.save(path)
            self.statusBar().showMessage(f"已导出: {path}", 3000)
        except Exception as e:
            QtWidgets.QMessageBox.warning(self, "错误", f"导出失败: {e}")


    # ═══════════════════════════════════════════════════════════════
    #  Path planning
    # ═══════════════════════════════════════════════════════════════

    def _add_waypoint(self, world_pos):
        """Add a waypoint from a 3D click."""
        self.waypoints.append(np.asarray(world_pos))
        idx = len(self.waypoints)

        sphere = pv.Sphere(radius=0.15, center=world_pos)
        actor = self.plotter.add_mesh(
            sphere, color="red", smooth_shading=True
        )
        self._wp_actors.append(actor)

        label_actor = self.plotter.add_point_labels(
            np.array([world_pos]),
            [str(idx)],
            show_points=False,
            font_size=14,
            text_color="red",
            shape_opacity=0.0,
            always_visible=True,
        )
        self._wp_actors.append(label_actor)

        self.plotter.render()
        self._wp_count_label.setText(f"当前路径点: {idx}")
        self.statusBar().showMessage(
            f"路径点 #{idx}:  {world_pos[0]:.2f}, {world_pos[1]:.2f}, {world_pos[2]:.2f}",
            3000,
        )

    def _open_precise_wp_dialog(self):
        if not _HAS_MPL:
            QtWidgets.QMessageBox.warning(
                self, "提示",
                "精准添加路径点需要 matplotlib 支持。\n请安装: pip install matplotlib"
            )
            return
        dlg = WaypointPreciseDialog(self)
        if dlg.exec_() == QtWidgets.QDialog.Accepted:
            coords_list = dlg.get_coords()
            for xyz in coords_list:
                self._add_waypoint(np.array(xyz))

    def _clear_waypoints(self):
        self.waypoints.clear()
        for a in self._wp_actors:
            self.plotter.remove_actor(a)
        self._wp_actors.clear()
        if self._path_actor is not None:
            self.plotter.remove_actor(self._path_actor)
            self._path_actor = None
        self._wp_count_label.setText("当前路径点: 0")
        self.plotter.render()

    # ═══════════════════════════════════════════════════════════════
    #  Flight animation (ID-11)
    # ═══════════════════════════════════════════════════════════════

    def _refresh_flight_combo(self):
        """Populate the flight aircraft combo with aircraft objects only."""
        current = self._flight_aircraft_combo.currentText()
        self._flight_aircraft_combo.blockSignals(True)
        self._flight_aircraft_combo.clear()
        for name in self.scene_objects:
            if "aircraft" in name.lower():
                self._flight_aircraft_combo.addItem(name)
        idx = self._flight_aircraft_combo.findText(current)
        if idx >= 0:
            self._flight_aircraft_combo.setCurrentIndex(idx)
        self._flight_aircraft_combo.blockSignals(False)

    # ── Formation flight (ID-27) ──────────────────────────

    def _on_formation_toggled(self, checked):
        """Show/hide the formation aircraft list when toggle button is clicked."""
        self._formation_mode = checked
        self._formation_list.setVisible(checked)
        if checked:
            self._refresh_formation_list()

    def _refresh_formation_list(self):
        """Populate the formation list with checkable aircraft items."""
        self._formation_list.clear()
        for name in self.scene_objects:
            if "aircraft" in name.lower():
                item = QtWidgets.QListWidgetItem(name)
                item.setFlags(item.flags() | QtCore.Qt.ItemIsUserCheckable)
                item.setCheckState(QtCore.Qt.Unchecked)
                self._formation_list.addItem(item)

    def _get_formation_selection(self):
        """Return list of checked aircraft names from the formation list.

        First checked item is the leader, rest are followers.
        """
        selected = []
        for i in range(self._formation_list.count()):
            item = self._formation_list.item(i)
            if item.checkState() == QtCore.Qt.Checked:
                selected.append(item.text())
        return selected

    # ────────────────────────────────────────────────────────────────
    #  ID-15: Flight speed model (systematic, physics-aware)
    # ────────────────────────────────────────────────────────────────

    CRUISE_SPEED = 5.0
    FLIGHT_INTERVAL_MS = 50
    _MIN_FLIGHT_STEPS = 10
    _MAX_FLIGHT_STEPS = 300

    def _flight_speed(self, pitch_deg, yaw_rate_deg_s):
        """Systematic speed model returning effective velocity.

        V = V₀ × k_pitch(pitch) × k_turn(yaw_rate)

        k_pitch = max(0.30, 1.0 − 0.35 × sin(pitch_rad))
          pitch =  0° → k_pitch = 1.00  (level)
          pitch = +30° → k_pitch = 0.83  (climbing, −17 %)
          pitch = +60° → k_pitch = 0.70  (steep climb, −30 %)
          pitch = −30° → k_pitch = 1.17  (diving, +17 %)
          pitch = −60° → k_pitch = 1.30  (steep dive, +30 %)

        k_turn = max(0.30, 1.0 − 0.006 × |yaw_rate|)
          yaw_rate =   0 °/s → k_turn = 1.00
          yaw_rate =  50 °/s → k_turn = 0.70
          yaw_rate = 100 °/s → k_turn = 0.30  (clamped)
        """
        pitch_rad = math.radians(pitch_deg)
        k_pitch = max(0.30, 1.0 - 0.35 * math.sin(pitch_rad))
        k_turn = max(0.30, 1.0 - 0.006 * abs(yaw_rate_deg_s))
        return self.CRUISE_SPEED * k_pitch * k_turn

    def _start_flight(self):
        """Animate the selected aircraft through all waypoints.

        In formation mode, the leader (first checked aircraft) follows the path;
        followers maintain their initial offset from the leader.
        """
        if self._flight_active:
            self._stop_flight()
            return

        # Determine which aircraft to fly
        if self._formation_mode:
            selected = self._get_formation_selection()
            if len(selected) < 1:
                self.statusBar().showMessage("编队模式：请至少选择一架飞机", 3000)
                return
            name = selected[0]  # leader
        else:
            name = self._flight_aircraft_combo.currentText()
            selected = [name] if name else []

        if not name:
            self.statusBar().showMessage("请先选择一架飞机", 3000)
            return

        # Get waypoints for the leader (ID-20: per-aircraft waypoints)
        aircraft_wps = self._get_aircraft_waypoints(name)
        if len(aircraft_wps) < 2:
            self.statusBar().showMessage("路径点不足（至少需要 2 个点）", 3000)
            return

        # Build flight path: start at waypoint[0], end at waypoint[-1] (ID-20)
        path = [np.array(wp) for wp in aircraft_wps]

        # Ensure leader transform exists
        t = self._get_or_init_transform(name)

        # ── Build segments with speed-aware step count ──
        segments = []
        interval_s = self.FLIGHT_INTERVAL_MS / 1000.0

        # First pass: compute raw yaw/pitch for each segment
        raw = []
        for i in range(len(path) - 1):
            p0 = path[i]
            p1 = path[i + 1]
            dx = p1[0] - p0[0]
            dy = p1[1] - p0[1]
            dz = p1[2] - p0[2]
            horiz_dist = np.sqrt(dx * dx + dy * dy) + 1e-12
            dist = np.linalg.norm(p1 - p0) + 1e-12

            yaw = math.degrees(math.atan2(dy, dx)) % 360.0
            pitch = math.degrees(math.atan2(-dz, horiz_dist))
            pitch = max(-90.0, min(90.0, pitch))

            raw.append({"dist": dist, "yaw": yaw, "pitch": pitch, "from": p0.tolist(), "to": p1.tolist()})

        # Second pass: compute yaw-rate-aware speed & steps per segment
        for i, r in enumerate(raw):
            # Yaw change from previous segment (for turn factor)
            if i > 0:
                prev_yaw = raw[i - 1]["yaw"]
                d_yaw = (r["yaw"] - prev_yaw) % 360.0
                if d_yaw > 180.0:
                    d_yaw -= 360.0
            else:
                d_yaw = 0.0

            # Pitch factor → approximate speed for yaw-rate estimate
            pitch_rad = math.radians(r["pitch"])
            k_pitch_est = max(0.30, 1.0 - 0.35 * math.sin(pitch_rad))
            approx_speed = self.CRUISE_SPEED * k_pitch_est
            approx_time = r["dist"] / approx_speed if approx_speed > 0 else 999.0
            yaw_rate = abs(d_yaw) / approx_time if approx_time > 0 else 0.0

            # Final speed with turn factor
            V = self._flight_speed(r["pitch"], yaw_rate)
            segment_time = r["dist"] / V if V > 0 else 999.0
            steps = max(self._MIN_FLIGHT_STEPS,
                        min(self._MAX_FLIGHT_STEPS,
                            int(round(segment_time / interval_s))))

            segments.append({
                "from": r["from"],
                "to": r["to"],
                "yaw": r["yaw"],
                "pitch": r["pitch"],
                "roll": 0.0,
                "steps": steps,
            })

        # Pre-compute total time for timeline (ID-20)
        total_steps = sum(seg["steps"] for seg in segments)
        total_time_ms = total_steps * self.FLIGHT_INTERVAL_MS

        # Teleport leader to waypoint[0]
        t["offset"] = list(aircraft_wps[0])
        self._apply_obj_transform_to_actor(name)

        # Compute tail-chase trail distances (ID-27)
        self._formation_offsets.clear()
        leader_start = np.array(aircraft_wps[0])
        for fname in selected[1:]:
            ft = self._get_or_init_transform(fname)
            fpos = np.array(ft.get("offset", leader_start))
            d = float(np.linalg.norm(fpos - leader_start)) or 8.0
            self._formation_offsets[fname] = d
            # Place follower directly behind leader at initial heading
            ft["offset"] = (leader_start + np.array([-d, 0, 0])).tolist()
            ft["yaw"] = 0.0
            self._apply_obj_transform_to_actor(fname)

        cache = {
            "aircraft_name": name,
            "formation": selected,  # all aircraft in formation (ID-27)
            "start_position": aircraft_wps[0],
            "waypoints": [wp.tolist() for wp in aircraft_wps],
            "segments": segments,
            "interval_ms": self.FLIGHT_INTERVAL_MS,
        }

        # Start animation
        self._flight_active = True
        self._flight_aircraft = name
        self._flight_aircraft_list = selected  # all aircraft in this flight (ID-27)
        self._flight_path = path
        self._flight_segments = segments
        self._flight_segment_idx = 0
        self._flight_step = 0
        self._flight_data_cache = cache

        # Timeline state (ID-20)
        self._total_flight_steps = total_steps
        self._total_flight_time_ms = total_time_ms

        self._tl_slider.setRange(0, total_time_ms)
        self._tl_slider.setValue(0)
        self._update_timeline_label(0)
        self._setup_keyframe_labels(len(aircraft_wps))
        self._btn_start_flight.setText("停止飞行")
        self._flight_aircraft_combo.setEnabled(False)
        self._btn_formation.setEnabled(False)
        self._btn_save_flight.setEnabled(False)
        self._btn_load_flight.setEnabled(False)

        self._flight_window = None

        self.plotter.render()
        count = len(selected)
        label = f"编队 {count}机" if count > 1 else name
        self.statusBar().showMessage(
            f"飞行开始: {label} → {len(aircraft_wps)} 个路径点 ({total_time_ms/1000:.1f}s)", 3000
        )
        self._flight_timer.start(self.FLIGHT_INTERVAL_MS)

    def _stop_flight(self):
        """Stop the current flight animation."""
        self._flight_timer.stop()
        self._flight_active = False
        self._flight_segment_idx = 0
        self._flight_step = 0

        self._btn_start_flight.setText("开始飞行")
        self._flight_aircraft_combo.setEnabled(True)
        self._btn_formation.setEnabled(True)
        self._btn_save_flight.setEnabled(True)
        self._btn_load_flight.setEnabled(True)

    # ────────────────────────────────────────────────────────────────
    #  ID-14: Smooth flight path & attitude
    # ────────────────────────────────────────────────────────────────

    @staticmethod
    def _catmull_rom_position(path, seg_idx, t):
        """Catmull-Rom spline interpolated position for segment seg_idx at t (0..1).

        Produces smooth C1-continuous curves through all waypoints.
        For boundary segments, phantom control points are mirrored to maintain tangency.
        """
        n = len(path)
        p1 = path[seg_idx]
        p2 = path[seg_idx + 1]

        # Left neighbour (mirrored at boundary)
        if seg_idx > 0:
            p0 = path[seg_idx - 1]
        else:
            p0 = 2 * p1 - p2

        # Right neighbour (mirrored at boundary)
        if seg_idx + 2 < n:
            p3 = path[seg_idx + 2]
        else:
            p3 = 2 * p2 - p1

        t2 = t * t
        t3 = t2 * t
        return 0.5 * (
            (2 * p1)
            + (-p0 + p2) * t
            + (2 * p0 - 5 * p1 + 4 * p2 - p3) * t2
            + (-p0 + 3 * p1 - 3 * p2 + p3) * t3
        )

    @staticmethod
    def _lerp_angle(a, b, t):
        """Linearly interpolate between two angles (degrees), taking the shortest path across 0/360."""
        d = (b - a) % 360.0
        if d > 180.0:
            d -= 360.0
        return (a + d * t) % 360.0

    def _compute_flight_state_at(self, seg_idx, step):
        """Compute aircraft position and attitude at (segment, step).

        Returns a dict with ``pos`` (np.array), ``yaw``, ``pitch``, ``roll``,
        or ``None`` if the segment index is out of range.
        """
        segments = self._flight_segments
        if seg_idx >= len(segments):
            return None

        seg = segments[seg_idx]
        steps = seg["steps"]
        t_val = step / float(steps) if steps > 0 else 1.0
        t_val = min(t_val, 1.0)

        pos = self._catmull_rom_position(self._flight_path, seg_idx, t_val)

        entry_yaw = float(seg["yaw"])
        if seg_idx + 1 < len(segments):
            exit_yaw = float(segments[seg_idx + 1]["yaw"])
        else:
            exit_yaw = entry_yaw
        yaw = self._lerp_angle(entry_yaw, exit_yaw, t_val)

        entry_pitch = float(seg["pitch"])
        if seg_idx + 1 < len(segments):
            exit_pitch = float(segments[seg_idx + 1]["pitch"])
        else:
            exit_pitch = entry_pitch
        pitch = entry_pitch + (exit_pitch - entry_pitch) * t_val
        pitch = max(-90.0, min(90.0, pitch))

        roll = 0.0

        return {"pos": pos, "yaw": yaw, "pitch": pitch, "roll": roll}

    def _apply_flight_state(self, state):
        """Apply a flight state dict to the aircraft actor(s) + UI sliders.

        In formation mode, also updates all followers relative to the leader.
        """
        if state is None:
            return
        name = self._flight_aircraft
        pos = state["pos"]
        yaw = state["yaw"]
        pitch = state["pitch"]
        roll = state["roll"]

        # Update leader transform
        if name in self._obj_transforms:
            self._obj_transforms[name]["offset"] = pos.tolist()
            self._obj_transforms[name]["yaw"] = yaw
            self._obj_transforms[name]["pitch"] = pitch
            self._obj_transforms[name]["roll"] = roll

        # Update UI sliders (block signals to avoid double-trigger)
        self._set_slider_value(self._slider_obj_x, pos[0])
        self._set_slider_value(self._slider_obj_y, pos[1])
        self._set_slider_value(self._slider_obj_z, pos[2])
        self._set_slider_value(self._slider_obj_yaw, yaw)
        self._set_slider_value(self._slider_obj_pitch, pitch)
        self._set_slider_value(self._slider_obj_roll, roll)

        # Apply to VTK actor on main window
        self._apply_obj_transform_to_actor(name)

        # Update tail-chase followers: same altitude, fixed trail distance behind leader (ID-27)
        aircraft_list = getattr(self, '_flight_aircraft_list', [])
        if len(aircraft_list) > 1:
            yaw_rad = math.radians(yaw)
            hx, hy = math.cos(yaw_rad), math.sin(yaw_rad)  # leader heading
            for fname in aircraft_list[1:]:
                d = self._formation_offsets.get(fname)
                if d is None or fname not in self._obj_transforms:
                    continue
                # Follower position: directly behind leader along its heading
                fx = pos[0] - hx * d
                fy = pos[1] - hy * d
                fz = pos[2]  # same altitude
                self._obj_transforms[fname]["offset"] = [fx, fy, fz]
                self._obj_transforms[fname]["yaw"] = yaw
                self._obj_transforms[fname]["pitch"] = pitch
                self._apply_obj_transform_to_actor(fname)

    def _flight_tick(self):
        """Single animation step called by the flight timer."""
        if not self._flight_active:
            return

        try:
            seg_idx = self._flight_segment_idx
            step = self._flight_step
            segments = self._flight_segments
            name = self._flight_aircraft

            if seg_idx >= len(segments):
                self._stop_flight()
                self.statusBar().showMessage("飞行完成", 5000)
                return

            seg = segments[seg_idx]
            steps_in_seg = seg["steps"]

            state = self._compute_flight_state_at(seg_idx, step)
            self._apply_flight_state(state)

            # Update timeline (ID-20)
            self._update_timeline()

            # Advance step / segment
            self._flight_step += 1
            if self._flight_step >= steps_in_seg:
                self._flight_segment_idx += 1
                self._flight_step = 0
        except Exception as e:
            self._stop_flight()
            self.statusBar().showMessage(f"飞行错误: {e}", 5000)

    # ═══════════════════════════════════════════════════════════════
    #  Flight data persistence (ID-11)
    # ═══════════════════════════════════════════════════════════════

    def _ensure_flight_dir(self):
        """Create data/flight directory under the project root."""
        base = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
        d = os.path.join(base, "data", "flight")
        os.makedirs(d, exist_ok=True)
        return d

    def _save_flight_data(self):
        """Save the last flight data to a JSON file."""
        cache = self._flight_data_cache
        if cache is None:
            self.statusBar().showMessage("没有可保存的飞行数据，请先执行一次飞行", 3000)
            return

        name, _ = QtWidgets.QFileDialog.getSaveFileName(
            self, "保存飞行数据",
            os.path.join(self._ensure_flight_dir(), "flight_data"),
            "JSON (*.json)"
        )
        if not name:
            return

        segments = cache.get("segments", [])
        required_keys = {"from", "to", "yaw", "pitch", "roll", "steps"}
        for i, seg in enumerate(segments):
            if not isinstance(seg, dict) or not required_keys.issubset(seg.keys()):
                QtWidgets.QMessageBox.warning(
                    self, "错误",
                    f"飞行数据中第 {i} 段格式无效，缺少必要字段，保存终止"
                )
                return

        data = {
            "aircraft_name": cache["aircraft_name"],
            "start_position": cache["start_position"],
            "waypoints": cache["waypoints"],
            "segments": segments,
            "interval_ms": cache["interval_ms"],
            "description": (
                "Flight path data for 3DSceneSoftware. "
                "segments[i] contains yaw/pitch/roll for travel from 'from' to 'to'."
            ),
        }
        with open(name, "w") as f:
            json.dump(data, f, indent=2)

        # ── Save companion 3D scatter plot ──
        if _HAS_MPL:
            try:
                fig = plt.figure(figsize=(10, 8))
                ax = fig.add_subplot(111, projection="3d")
                wps = cache["waypoints"]
                all_pts = [cache["start_position"]] + wps
                xs = [p[0] for p in all_pts]
                ys = [p[1] for p in all_pts]
                zs = [p[2] for p in all_pts]
                ax.scatter(xs[1:], ys[1:], zs[1:], c="red", s=50, label="Waypoints")
                ax.plot(xs, ys, zs, "b-", linewidth=2, label="Flight Path")
                for i, (x, y, z) in enumerate(wps, 1):
                    ax.text(x, y, z, f" {i}", color="red", fontsize=9)
                ax.set_xlabel("X")
                ax.set_ylabel("Y")
                ax.set_zlabel("Z")
                ax.set_title(f'Flight Path: {cache["aircraft_name"]}')
                ax.legend()
                png_path = name.replace(".json", "_3dplot.png")
                fig.savefig(png_path, dpi=150, bbox_inches="tight")
                plt.close(fig)
            except Exception as e:
                print(f"Failed to generate 3D plot: {e}")

        self.statusBar().showMessage(f"飞行数据已保存: {name}", 3000)

    def _load_flight_data(self):
        """Load a flight data JSON and replay the animation."""
        path, _ = QtWidgets.QFileDialog.getOpenFileName(
            self, "载入飞行数据",
            self._ensure_flight_dir(),
            "JSON (*.json)"
        )
        if not path:
            return

        try:
            with open(path) as f:
                data = json.load(f)
        except Exception as e:
            QtWidgets.QMessageBox.warning(self, "错误", f"加载飞行数据失败: {e}")
            return

        name = data.get("aircraft_name", "")
        if name not in self.scene_objects and name not in self.custom_objects:
            QtWidgets.QMessageBox.warning(
                self, "错误",
                f"飞机 '{name}' 不存在于当前场景中"
            )
            return

        segments = data.get("segments", [])
        if not segments:
            self.statusBar().showMessage("飞行数据中没有有效的路径段", 3000)
            return

        # Validate each segment has the required keys (ID-11 fix)
        required_keys = {"from", "to", "yaw", "pitch", "roll", "steps"}
        for i, seg in enumerate(segments):
            if not isinstance(seg, dict) or not required_keys.issubset(seg.keys()):
                QtWidgets.QMessageBox.warning(
                    self, "错误",
                    f"飞行数据中第 {i} 段格式无效，缺少必要字段"
                )
                return

        # Select the aircraft in the flight combo
        idx = self._flight_aircraft_combo.findText(name)
        if idx >= 0:
            self._flight_aircraft_combo.setCurrentIndex(idx)

        interval_ms = data.get("interval_ms", 50)

        # Rebuild waypoints from the data for visual reference
        saved_wps = data.get("waypoints", [])
        # Don't modify self.waypoints — just replay the animation

        # Start flight with loaded segments
        t = self._get_or_init_transform(name)
        start_pos = data.get("start_position", t["offset"])
        # Apply saved start position so the aircraft is correctly positioned
        # before the first tick fires (ID-11 fix).
        t["offset"] = list(start_pos)
        self._apply_obj_transform_to_actor(name)
        self.plotter.render()

        # Pre-compute total time for timeline (ID-20)
        total_steps = sum(seg["steps"] for seg in segments)
        total_time_ms = total_steps * interval_ms

        self._flight_active = True
        self._flight_aircraft = name
        self._flight_path = [np.array(p) for p in [start_pos] + saved_wps]
        self._flight_segments = segments
        self._flight_segment_idx = 0
        self._flight_step = 0
        self._flight_steps_per_segment = segments[0].get("steps", 50)
        self._flight_data_cache = data

        # Timeline state (ID-20)
        self._total_flight_steps = total_steps
        self._total_flight_time_ms = total_time_ms

        self._tl_slider.setRange(0, total_time_ms)
        self._tl_slider.setValue(0)
        self._update_timeline_label(0)
        num_wp = len(saved_wps) + 1  # +1 for start position
        self._setup_keyframe_labels(num_wp)
        self._btn_start_flight.setText("停止飞行")
        self._flight_aircraft_combo.setEnabled(False)
        self._btn_save_flight.setEnabled(False)
        self._btn_load_flight.setEnabled(False)

        self.statusBar().showMessage(
            f"载入飞行数据: {name} ({len(segments)} 个段)", 3000
        )
        self._flight_timer.start(interval_ms)


    # ═══════════════════════════════════════════════════════════════
    #  ID-20: Timeline / seek / per-aircraft waypoints
    # ═══════════════════════════════════════════════════════════════

    def _get_aircraft_waypoints(self, name):
        """Return waypoint list for *name* (per-aircraft or global fallback)."""
        if name in self._aircraft_waypoints and self._aircraft_waypoints[name]:
            return self._aircraft_waypoints[name]
        return self.waypoints

    def _setup_keyframe_labels(self, num_wp):
        """Place keyframe markers (⬤1 ⬤2 …) above the timeline slider."""
        while self._kf_layout.count():
            item = self._kf_layout.takeAt(0)
            if item.widget():
                item.widget().deleteLater()
        for i in range(num_wp):
            lbl = QtWidgets.QLabel(f"\u2b24 {i+1}")
            lbl.setStyleSheet("color: #cc3333; font-size: 9px;")
            self._kf_layout.addWidget(lbl)
        self._kf_layout.addStretch()

    def _update_timeline(self):
        """Sync the timeline slider with current flight progress."""
        accum = 0
        for i in range(self._flight_segment_idx):
            accum += self._flight_segments[i]["steps"]
        accum += self._flight_step

        current_ms = 0
        if self._total_flight_steps > 0:
            progress = accum / self._total_flight_steps
            current_ms = int(progress * self._total_flight_time_ms)

        self._tl_slider.blockSignals(True)
        self._tl_slider.setValue(current_ms)
        self._tl_slider.blockSignals(False)
        self._update_timeline_label(current_ms)

    def _update_timeline_label(self, current_ms=None):
        """Update the time label showing current / total seconds."""
        if current_ms is None:
            current_ms = self._tl_slider.value()
        current_s = current_ms / 1000.0
        total_s = self._total_flight_time_ms / 1000.0
        self._tl_time_label.setText(f"{current_s:.1f} / {total_s:.1f} 秒")

    # ── Timeline seek handlers ──────────────────────────────

    def _on_tl_pressed(self):
        """Pause the flight timer while the user drags the timeline."""
        if self._flight_timer.isActive():
            self._flight_timer.stop()

    def _on_tl_seek(self, value_ms):
        """Seek to a specific time position (value in ms) during slider drag."""
        if not self._flight_active or not self._flight_segments:
            return
        total_ms = self._total_flight_time_ms
        if total_ms <= 0:
            return

        progress = value_ms / total_ms
        target_step = int(progress * self._total_flight_steps)
        target_step = max(0, min(self._total_flight_steps - 1, target_step))

        # Find segment and step
        accum = 0
        for seg_idx, seg in enumerate(self._flight_segments):
            if accum + seg["steps"] > target_step:
                self._flight_segment_idx = seg_idx
                self._flight_step = target_step - accum
                break
            accum += seg["steps"]

        # Update slider value before apply so _flight_window reads correct time
        self._tl_slider.blockSignals(True)
        self._tl_slider.setValue(value_ms)
        self._tl_slider.blockSignals(False)

        state = self._compute_flight_state_at(
            self._flight_segment_idx, self._flight_step
        )
        self._apply_flight_state(state)
        self._update_timeline_label(value_ms)

    def _on_tl_released(self):
        """Resume the flight timer after the user releases the slider."""
        if self._flight_active:
            self._flight_timer.start(self.FLIGHT_INTERVAL_MS)

    # ── Independent flight window (ID-20) ──────────────────

    # ═══════════════════════════════════════════════════════════════
    #  Collision detection
    # ═══════════════════════════════════════════════════════════════

    def _run_collision_check(self):
        meshes = []
        names = []
        for n, info in self.scene_objects.items():
            if info["visible"]:
                meshes.append(info["mesh"])
                names.append(f"[场景] {n}")
        for n, m in self.custom_objects.items():
            meshes.append(m)
            names.append(f"[模型] {n}")

        results = find_collisions(meshes, names)

        if results:
            msg = "🟡 碰撞检测结果 (AABB 包围盒):\n\n" + "\n".join(
                f"  • {a}  ↔  {b}" for a, b in results
            )
        else:
            msg = "✅ 未检测到碰撞（基于包围盒 AABB）"

        QtWidgets.QMessageBox.information(self, "碰撞检测", msg)


    # ═══════════════════════════════════════════════════════════════
    #  Highlight / selection
    # ═══════════════════════════════════════════════════════════════

    def _select_object(self, name):
        """Highlight a scene object by name."""
        if name == self.selected_name:
            return

        self._clear_highlight()

        actor = None
        lookup_name = name
        if name in self.plotter_actors:
            actor = self.plotter_actors[name]
        elif f"_custom_{name}" in self.plotter_actors:
            actor = self.plotter_actors[f"_custom_{name}"]
            lookup_name = f"_custom_{name}"
        elif name in self.custom_objects:
            actor = self.plotter_actors.get(f"_custom_{name}")
            lookup_name = f"_custom_{name}"

        if actor is None:
            return

        vtk_actor = self._resolve_vtk_actor(actor)
        prop = vtk_actor.GetProperty()
        self._highlight_props[lookup_name] = {
            "color": prop.GetColor(),
            "edge_color": prop.GetEdgeColor(),
            "line_width": prop.GetLineWidth(),
            "opacity": prop.GetOpacity(),
        }
        prop.SetColor(1.0, 0.84, 0.0)
        prop.SetEdgeColor(1.0, 0.0, 0.0)
        prop.SetLineWidth(3)
        prop.SetOpacity(0.9)
        self._highlighted_vtk_actor = vtk_actor
        self.selected_name = name
        self.plotter.render()
        self._update_info()

    def _clear_highlight(self):
        """Restore any previously highlighted actor's original properties."""
        if self._highlighted_vtk_actor is not None:
            for actor_name, saved in list(self._highlight_props.items()):
                if actor_name in self.plotter_actors:
                    try:
                        va = self._resolve_vtk_actor(self.plotter_actors[actor_name])
                        if va is self._highlighted_vtk_actor:
                            prop = va.GetProperty()
                            prop.SetColor(*saved["color"])
                            prop.SetEdgeColor(*saved["edge_color"])
                            prop.SetLineWidth(saved["line_width"])
                            prop.SetOpacity(saved["opacity"])
                            break
                    except Exception:
                        pass
            self._highlight_props.clear()
            self._highlighted_vtk_actor = None
            self.selected_name = None
            self.plotter.render()


    # ═══════════════════════════════════════════════════════════════
    #  Coordinate info display
    # ═══════════════════════════════════════════════════════════════

    def _update_info(self, mouse_world=None):
        """Update the coordinate-information dock."""
        cs = self.coord_system
        lines = [
            f"坐标系: {cs}  ({COORD_SYSTEMS[cs]['label']})",
            "",
        ]

        # ── Mouse coordinates FIRST (per ID-4) ──
        if mouse_world is not None:
            try:
                lines.append("🖱 鼠标 (世界坐标)")
                lines.append(f"  {world_to_coord_str(cs, *mouse_world)}")
            except Exception:
                pass

        # ── Camera info SECOND ──
        try:
            cam = self.plotter.camera
            pos = cam.GetPosition()
            fp = cam.GetFocalPoint()
            lines.append("")
            lines.append("📷 相机")
            lines.append(f"  位置:   {world_to_coord_str(cs, *pos)}")
            lines.append(f"  目标:   {world_to_coord_str(cs, *fp)}")
            dist = np.linalg.norm(np.array(pos) - np.array(fp))
            lines.append(f"  距离:   {dist:.2f}")
        except Exception:
            pass

        if self.selected_name is not None:
            lines.append("")
            lines.append(f"✅ 选中: {self.selected_name}")

        self._info_text.setText("\n".join(lines))


    # ═══════════════════════════════════════════════════════════════
    #  Timers & cleanup
    # ═══════════════════════════════════════════════════════════════

    def _setup_timers(self):
        self._rec_timer = QTimer(self)
        self._rec_timer.timeout.connect(self._capture_frame)

        self._river_timer = QTimer(self)
        self._river_timer.timeout.connect(self._animate_river)
        self._river_timer.start(100)

    def _animate_river(self):
        if not self._flowing:
            return
        info = self.scene_objects.get("river")
        if info is None:
            return
        extra = info.get("extra")
        if extra is None:
            return
        Ry = extra["Ry"]
        phase = extra.get("phase", 0)
        phase += 0.30
        extra["phase"] = phase

        base_z = -0.05
        wave = 0.07 * np.sin(Ry * 1.5 + phase) + 0.04 * np.sin(Ry * 3.0 + phase * 1.8)
        mesh = info["mesh"]
        pts = mesh.points
        new_z = np.full_like(extra["Rz_base"], base_z) + wave
        pts[:, 2] = new_z.flatten(order="F")
        mesh.points = pts
        self.plotter.render()

    def _cleanup(self):
        """Release VTK resources."""
        self._rec_timer.stop()
        self._river_timer.stop()
        self._flight_timer.stop()
        self._recording = False
        try:
            self.plotter.close()
        except Exception:
            pass

    def _show_about(self):
        QtWidgets.QMessageBox.about(
            self,
            "关于 3DSceneSoftware",
            "<h3>3DSceneSoftware v2.0</h3>"
            "<p>基于 PyQt5 + PyVista (VTK 引擎)</p>"
            "<p>可视化 3D 大场景 + 目标建模</p>"
            "<hr>"
            "<p><b>功能:</b></p>"
            "<ul>"
            "<li>场景渲染 · 视角控制 · 图层管理</li>"
            "<li>参数调节 · 3D 元素选择 · 坐标显示</li>"
            "<li>测量工具 (距离/角度) · 碰撞检测</li>"
            "<li>路径规划 (点选 + 坐标输入 + 样条曲线)</li>"
            "<li>场景保存/加载 · 截图/连续录制</li>"
            "<li>模型导入 (STL/OBJ) / 导出</li>"
            "<li>对象变换 (位置/缩放) · 坐标系切换</li>"
            "<li>多对象场景 (地形/河流/植被/飞行器/鸟/树)</li>"
            "</ul>",
        )




class WaypointPreciseDialog(QtWidgets.QDialog):
    def __init__(self, parent=None):
        super().__init__(parent)
        self.setWindowTitle("精准添加路径点")
        self.setMinimumSize(640, 520)

        self._points = []
        self._z_values = []
        self._selected_z_idx = 0

        self._stack = QtWidgets.QStackedWidget()
        self._stack.addWidget(self._create_step1())
        self._stack.addWidget(self._create_step2())

        layout = QtWidgets.QVBoxLayout(self)
        layout.addWidget(self._stack)
        self._stack.setCurrentIndex(0)

    def _label_for(self, idx):
        return chr(65 + idx) if idx < 26 else f"?{idx}"

    def _create_step1(self):
        widget = QtWidgets.QWidget()
        layout = QtWidgets.QVBoxLayout(widget)

        title = QtWidgets.QLabel("精准添加路径点 — 选择 XY 坐标（点击添加多点）")
        title.setStyleSheet("font-weight: bold; font-size: 14px;")
        layout.addWidget(title)

        self._fig_xy = Figure(figsize=(5, 4))
        self._ax_xy = self._fig_xy.add_subplot(111)
        self._ax_xy.set_xlim(-10, 10)
        self._ax_xy.set_ylim(-10, 10)
        self._ax_xy.set_aspect("equal")
        self._ax_xy.grid(True, linestyle="--", alpha=0.7)
        self._ax_xy.set_xlabel("X")
        self._ax_xy.set_ylabel("Y")
        self._ax_xy.axhline(0, color="gray", linewidth=0.5)
        self._ax_xy.axvline(0, color="gray", linewidth=0.5)

        self._canvas_xy = FigureCanvasQTAgg(self._fig_xy)
        self._canvas_xy.mpl_connect("button_press_event", self._on_xy_click)
        layout.addWidget(self._canvas_xy)

        # ── Coordinate list table (ID-19) ──
        self._point_table = QtWidgets.QTableWidget()
        self._point_table.setColumnCount(4)
        self._point_table.setHorizontalHeaderLabels(["点", "X", "Y", "操作"])
        self._point_table.horizontalHeader().setStretchLastSection(True)
        self._point_table.setMaximumHeight(150)
        self._point_table.setAlternatingRowColors(True)
        layout.addWidget(self._point_table)

        spin_layout = QtWidgets.QHBoxLayout()
        self._spin_x = QtWidgets.QDoubleSpinBox()
        self._spin_x.setRange(-20, 20)
        self._spin_x.setDecimals(2)
        self._spin_x.setPrefix("X: ")
        self._spin_x.valueChanged.connect(self._on_xy_spin_changed)
        self._spin_y = QtWidgets.QDoubleSpinBox()
        self._spin_y.setRange(-20, 20)
        self._spin_y.setDecimals(2)
        self._spin_y.setPrefix("Y: ")
        self._spin_y.valueChanged.connect(self._on_xy_spin_changed)
        spin_layout.addWidget(self._spin_x)
        spin_layout.addWidget(self._spin_y)
        spin_layout.addStretch()
        layout.addLayout(spin_layout)

        btn_layout = QtWidgets.QHBoxLayout()
        self._btn_undo = QtWidgets.QPushButton("撤销上一点")
        self._btn_undo.clicked.connect(self._undo_last_point)
        self._btn_undo.setEnabled(False)
        btn_cancel = QtWidgets.QPushButton("取消")
        btn_cancel.clicked.connect(self.reject)
        self._btn_next = QtWidgets.QPushButton("下一步")
        self._btn_next.clicked.connect(self._go_step2)
        self._btn_next.setEnabled(False)
        btn_layout.addWidget(self._btn_undo)
        btn_layout.addStretch()
        btn_layout.addWidget(btn_cancel)
        btn_layout.addWidget(self._btn_next)
        layout.addLayout(btn_layout)

        return widget

    def _create_step2(self):
        widget = QtWidgets.QWidget()
        layout = QtWidgets.QVBoxLayout(widget)

        title = QtWidgets.QLabel("精准添加路径点 — 选择高度 Z")
        title.setStyleSheet("font-weight: bold; font-size: 14px;")
        layout.addWidget(title)

        self._fig_z = Figure(figsize=(5, 3))
        self._ax_z = self._fig_z.add_subplot(111)
        self._ax_z.set_ylim(-20, 20)
        self._ax_z.set_xlabel("路径距离")
        self._ax_z.set_ylabel("高度 Z")
        self._ax_z.grid(True, linestyle="--", alpha=0.7)

        self._canvas_z = FigureCanvasQTAgg(self._fig_z)
        self._canvas_z.mpl_connect("button_press_event", self._on_z_click)
        layout.addWidget(self._canvas_z)

        info_layout = QtWidgets.QHBoxLayout()
        self._lbl_point_info = QtWidgets.QLabel("当前点: -, 距离: -")
        info_layout.addWidget(self._lbl_point_info)
        info_layout.addStretch()
        layout.addLayout(info_layout)

        self._spin_z = QtWidgets.QDoubleSpinBox()
        self._spin_z.setRange(-20, 20)
        self._spin_z.setDecimals(2)
        self._spin_z.setPrefix("Z: ")
        self._spin_z.valueChanged.connect(self._on_z_spin_changed)
        layout.addWidget(self._spin_z)

        # Z height slider for precise control (ID-19)
        z_slider_layout = QtWidgets.QHBoxLayout()
        z_slider_layout.addWidget(QtWidgets.QLabel("Z 高度:"))
        self._z_slider = QtWidgets.QSlider(Qt.Horizontal)
        self._z_slider.setRange(0, 1000)
        self._z_slider.valueChanged.connect(self._on_z_slider_changed)
        self._z_slider_label = QtWidgets.QLabel("0.00")
        self._z_slider_label.setMinimumWidth(50)
        z_slider_layout.addWidget(self._z_slider, 1)
        z_slider_layout.addWidget(self._z_slider_label)
        layout.addLayout(z_slider_layout)

        btn_layout = QtWidgets.QHBoxLayout()
        btn_prev = QtWidgets.QPushButton("上一步")
        btn_prev.clicked.connect(lambda: self._stack.setCurrentIndex(0))
        btn_ok = QtWidgets.QPushButton("确定")
        btn_ok.clicked.connect(self.accept)
        btn_layout.addWidget(btn_prev)
        btn_layout.addStretch()
        btn_layout.addWidget(btn_ok)
        layout.addLayout(btn_layout)

        return widget

    def _redraw_xy(self):
        self._ax_xy.clear()
        self._ax_xy.set_xlim(-10, 10)
        self._ax_xy.set_ylim(-10, 10)
        self._ax_xy.set_aspect("equal")
        self._ax_xy.grid(True, linestyle="--", alpha=0.7)
        self._ax_xy.set_xlabel("X")
        self._ax_xy.set_ylabel("Y")
        self._ax_xy.axhline(0, color="gray", linewidth=0.5)
        self._ax_xy.axvline(0, color="gray", linewidth=0.5)

        if self._points:
            pts = np.array(self._points)
            self._ax_xy.plot(pts[:, 0], pts[:, 1], "-o", color="steelblue",
                             markersize=6, linewidth=1.5)
            for i, (x, y) in enumerate(self._points):
                lbl = self._label_for(i)
                self._ax_xy.annotate(lbl, (x, y), xytext=(5, 5),
                                     textcoords="offset points",
                                     fontweight="bold", color="red", fontsize=10)

        self._canvas_xy.draw_idle()

    def _on_xy_click(self, event):
        if event.inaxes != self._ax_xy:
            return
        self._points.append([round(event.xdata, 2), round(event.ydata, 2)])
        block_x = self._spin_x.blockSignals(True)
        block_y = self._spin_y.blockSignals(True)
        self._spin_x.setValue(self._points[-1][0])
        self._spin_y.setValue(self._points[-1][1])
        self._spin_x.blockSignals(block_x)
        self._spin_y.blockSignals(block_y)
        self._redraw_xy()
        self._update_table()
        self._btn_undo.setEnabled(True)
        self._btn_next.setEnabled(len(self._points) >= 2)

    def _on_xy_spin_changed(self):
        if not self._points:
            return
        idx = len(self._points) - 1
        self._points[idx] = [round(self._spin_x.value(), 2),
                             round(self._spin_y.value(), 2)]
        self._redraw_xy()
        self._update_table()

    def _undo_last_point(self):
        if not self._points:
            return
        self._points.pop()
        if self._z_values and len(self._z_values) > len(self._points):
            self._z_values.pop()
        if self._selected_z_idx >= len(self._z_values):
            self._selected_z_idx = max(0, len(self._z_values) - 1)
        self._redraw_xy()
        self._update_table()
        self._btn_undo.setEnabled(bool(self._points))
        self._btn_next.setEnabled(len(self._points) >= 2)
        if self._points:
            block_x = self._spin_x.blockSignals(True)
            block_y = self._spin_y.blockSignals(True)
            self._spin_x.setValue(self._points[-1][0])
            self._spin_y.setValue(self._points[-1][1])
            self._spin_x.blockSignals(block_x)
            self._spin_y.blockSignals(block_y)

    def _update_table(self):
        """Refresh the point-list table from self._points."""
        self._point_table.setRowCount(len(self._points))
        for i, (x, y) in enumerate(self._points):
            lbl = self._label_for(i)
            item_id = QtWidgets.QTableWidgetItem(lbl)
            item_id.setFlags(item_id.flags() & ~Qt.ItemIsEditable)
            self._point_table.setItem(i, 0, item_id)
            item_x = QtWidgets.QTableWidgetItem(f"{x:.2f}")
            item_x.setFlags(item_x.flags() & ~Qt.ItemIsEditable)
            self._point_table.setItem(i, 1, item_x)
            item_y = QtWidgets.QTableWidgetItem(f"{y:.2f}")
            item_y.setFlags(item_y.flags() & ~Qt.ItemIsEditable)
            self._point_table.setItem(i, 2, item_y)
            btn_del = QtWidgets.QPushButton("删除")
            btn_del.clicked.connect(lambda checked, idx=i: self._delete_point(idx))
            self._point_table.setCellWidget(i, 3, btn_del)

    def _delete_point(self, idx):
        """Remove a point by index and re-number the remainder."""
        if idx < 0 or idx >= len(self._points):
            return
        self._points.pop(idx)
        if idx < len(self._z_values):
            self._z_values.pop(idx)
        if self._selected_z_idx >= len(self._z_values):
            self._selected_z_idx = max(0, len(self._z_values) - 1)
        self._redraw_xy()
        self._update_table()
        self._btn_undo.setEnabled(bool(self._points))
        self._btn_next.setEnabled(len(self._points) >= 2)

    def _compute_cumulative_distances(self):
        if len(self._points) < 2:
            return [0.0]
        dists = [0.0]
        total = 0.0
        for i in range(len(self._points) - 1):
            total += math.dist(self._points[i], self._points[i + 1])
            dists.append(total)
        return dists

    def _go_step2(self):
        if len(self._points) < 2:
            return
        if len(self._z_values) != len(self._points):
            self._z_values = [0.0] * len(self._points)
        if self._selected_z_idx >= len(self._z_values):
            self._selected_z_idx = 0
        self._stack.setCurrentIndex(1)
        self._redraw_z()

    def _redraw_z(self):
        self._ax_z.clear()
        if len(self._points) < 2:
            self._canvas_z.draw_idle()
            return

        cum_dists = self._compute_cumulative_distances()
        labels = [self._label_for(i) for i in range(len(self._points))]

        self._ax_z.plot(cum_dists, self._z_values, "-o", color="steelblue",
                        markersize=8, linewidth=2, zorder=3)

        sel_x = cum_dists[self._selected_z_idx]
        sel_y = self._z_values[self._selected_z_idx]
        self._ax_z.axvline(sel_x, color="red", linestyle="--", linewidth=1, zorder=1)

        for i, (cx, cy) in enumerate(zip(cum_dists, self._z_values)):
            self._ax_z.annotate(labels[i], (cx, cy), xytext=(5, 5),
                                textcoords="offset points",
                                fontweight="bold",
                                color="red" if i == self._selected_z_idx else "gray",
                                fontsize=10)

        self._ax_z.axhline(0, color="gray", linewidth=0.5)
        self._ax_z.set_xlabel("路径距离")
        self._ax_z.set_ylabel("高度 Z")
        self._ax_z.set_ylim(-20, 20)
        self._ax_z.grid(True, linestyle="--", alpha=0.7)
        self._ax_z.set_xticks(cum_dists)
        self._ax_z.set_xticklabels([f"{d:.2f}" for d in cum_dists], rotation=45)

        sel_label = self._label_for(self._selected_z_idx)
        self._lbl_point_info.setText(f"当前点: {sel_label}, 距离: {sel_x:.2f}")
        z_val = self._z_values[self._selected_z_idx]
        block = self._spin_z.blockSignals(True)
        self._spin_z.setValue(z_val)
        self._spin_z.blockSignals(block)
        z_min, z_max = -20, 20
        frac = (z_val - z_min) / (z_max - z_min)
        block_slider = self._z_slider.blockSignals(True)
        self._z_slider.setValue(int(frac * 1000))
        self._z_slider_label.setText(f"{z_val:.2f}")
        self._z_slider.blockSignals(block_slider)

        self._canvas_z.draw_idle()

    def _on_z_click(self, event):
        if event.inaxes != self._ax_z or len(self._points) < 2:
            return
        cum_dists = self._compute_cumulative_distances()
        idx = min(range(len(cum_dists)), key=lambda i: abs(cum_dists[i] - event.xdata))
        self._selected_z_idx = idx
        z_val = round(max(-20.0, min(20.0, event.ydata)), 2)
        self._z_values[idx] = z_val
        block = self._spin_z.blockSignals(True)
        self._spin_z.setValue(z_val)
        self._spin_z.blockSignals(block)
        z_min, z_max = -20, 20
        frac = (z_val - z_min) / (z_max - z_min)
        block_sl = self._z_slider.blockSignals(True)
        self._z_slider.setValue(int(frac * 1000))
        self._z_slider_label.setText(f"{z_val:.2f}")
        self._z_slider.blockSignals(block_sl)
        self._redraw_z()

    def _on_z_slider_changed(self, value):
        if not self._z_values or self._selected_z_idx >= len(self._z_values):
            return
        z_min, z_max = -20, 20
        frac = value / 1000.0
        z_val = round(z_min + frac * (z_max - z_min), 2)
        self._z_values[self._selected_z_idx] = z_val
        self._z_slider_label.setText(f"{z_val:.2f}")
        block = self._spin_z.blockSignals(True)
        self._spin_z.setValue(z_val)
        self._spin_z.blockSignals(block)
        self._redraw_z()

    def _on_z_spin_changed(self):
        if not self._z_values or self._selected_z_idx >= len(self._z_values):
            return
        z_val = round(self._spin_z.value(), 2)
        self._z_values[self._selected_z_idx] = z_val
        z_min, z_max = -20, 20
        frac = (z_val - z_min) / (z_max - z_min)
        block = self._z_slider.blockSignals(True)
        self._z_slider.setValue(int(frac * 1000))
        self._z_slider_label.setText(f"{z_val:.2f}")
        self._z_slider.blockSignals(block)
        self._redraw_z()

    def get_coords(self):
        result = []
        for i in range(len(self._points)):
            x, y = self._points[i]
            z = self._z_values[i] if i < len(self._z_values) else 0.0
            result.append([x, y, z])
        return result
