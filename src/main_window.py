"""
MainWindow — top-level QMainWindow integrating all UI, scene, and tool subsystems.

Key design decisions
--------------------
* Click detection uses a VTK observer (``LeftButtonPressEvent``) for press
  position and ``GetEventPosition()`` after ``super().mouseReleaseEvent()`` for
  release position — both give VTK-native display coordinates directly,
  avoiding fragile Qt‑to‑VTK conversion.
* World picking uses ``vtkHardwarePicker`` (GPU, pixel‑perfect) with
  ``vtkCellPicker`` fallback — never ``vtkWorldPointPicker``.
* Scene objects are initialised *before* the UI so docks and trees populate
  correctly on first render.
* Object transforms (position, scale) use VTK's ``UserTransform`` so mesh data
  is never mutated by the UI.
"""

import os
import json
import math
import time
import webbrowser
import numpy as np
import pyvista as pv
from vtkmodules.vtkCommonCore import vtkStringArray
from vtkmodules.vtkCommonMath import vtkMatrix4x4
from vtkmodules.vtkCommonTransforms import vtkTransform
from vtkmodules.vtkRenderingCore import vtkCellPicker, vtkPropPicker, vtkWorldPointPicker
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
    DEFAULT_CONFIG, load_config, save_config, COORD_SYSTEMS,
    AIRCRAFT_DEFAULT_SCALE_DEFAULT,
    DEFAULT_CAMERA_POSITION, DEFAULT_CAMERA_FOCAL, DEFAULT_CAMERA_UP,
    DEFAULT_SLIDER_RANGE_XY, DEFAULT_SLIDER_RANGE_Z,
    DEM_SLIDER_Z_MIN, DEM_SLIDER_Z_MAX,
    FLIGHT_TIMER_INTERVAL, FLIGHT_CRUISE_SPEED,
    FORMATION_TRAIL_DISTANCE, FORMATION_TRAIL_DISTANCE_DEM,
    CELL_PICKER_TOLERANCE, TRANSFORM_LOG_DEBOUNCE_MS,
    DEFAULT_GROUND_PLANE_SIZE,
    DEM_DEFAULT_STEP, DEM_DEFAULT_VERT_EXAG, DEM_VERT_EXAG_MIN, DEM_VERT_EXAG_MAX,
    SAVE_DIR_AIRCRAFT, SAVE_DIR_TERRAIN, SAVE_DIR_FLIGHT,
)
from src.interaction import InteractionMode
from src.scene_tree import SceneNodeType, NodeData, SceneTreeFactory
from src.scene_builder import build_default_scene
from src.measurement import MeasurementTool
from src.collision import find_collisions
from src.flight_math import compute_flight_speed, compute_flight_state, lerp_angle, catmull_rom_position
from src.dem_loader import (
    load_dem,
    build_dem_scene,
    dem_to_mesh,
    _build_airplane_mesh,
    HAS_RASTERIO,
    AIRCRAFT_DEFAULT_SCALE,
)


# ═══════════════════════════════════════════════════════════════
#  ClickablePlotter — Qt-level mouse handler
# ═══════════════════════════════════════════════════════════════

class ClickablePlotter(pvqt.QtInteractor):
    """``QtInteractor`` subclass with reliable world picking.

    Click detection
    ---------------
    A "click" is defined as a left-button press followed by a release
    within 5 screen pixels and 0.5 seconds (i.e. not a drag intended
    for camera orbit).  The click callback receives VTK display
    coordinates, the world position (via ``vtkHardwarePicker``), and
    the hit ``vtkActor`` (or ``None``).

    World position accuracy
    -----------------------
    Uses ``vtkHardwarePicker`` (GPU pixel-level, zero tolerance) as
    primary, ``vtkCellPicker`` as secondary, ``vtkWorldPointPicker``
    as last resort.  ``_snap_to_terrain`` refines Z via vertical ray
    intersection with the terrain mesh.
    """

    def __init__(self, parent=None):
        super().__init__(parent)
        self.setMouseTracking(True)
        self._press_pos = None          # (x, y) at press (Qt widget coords)
        self._press_time = 0.0
        self.click_callback = None      # f(vtk_x, vtk_y, world_pos, vtkActor)
        self.move_callback = None       # f(x, y, world_pos)

    # ── Qt event overrides ──────────────────────────

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

    def _to_vtk_display(self, qt_x, qt_y):
        """Convert Qt widget coords → VTK display coords (pixels, bottom-left origin).

        Qt origin is top-left, VTK display origin is bottom-left.
        VTK's render window reports its size in logical pixels (not Retina-physical),
        so we pass logical pixel coordinates directly — no devicePixelRatio scaling.
        """
        vtk_x = qt_x
        vtk_y = self.height() - qt_y - 1
        return vtk_x, vtk_y

    def _process_click(self, x, y):
        if self.click_callback is None:
            return
        vtk_x, vtk_y = self._to_vtk_display(x, y)
        picker = vtkCellPicker()
        picker.SetTolerance(0.001)
        if picker.Pick(vtk_x, vtk_y, 0, self.renderer) and picker.GetCellId() >= 0:
            world = np.array(picker.GetPickPosition())
            actor = picker.GetActor()
        else:
            pp = vtkPropPicker()
            if pp.Pick(vtk_x, vtk_y, 0, self.renderer):
                world = np.array(pp.GetPickPosition())
                actor = pp.GetActor()
            else:
                wp = vtkWorldPointPicker()
                wp.Pick(vtk_x, vtk_y, 0, self.renderer)
                world = np.array(wp.GetPickPosition())
                actor = None
        self.click_callback(x, y, world, actor)


# ═══════════════════════════════════════════════════════════════
#  MainWindow
# ═══════════════════════════════════════════════════════════════

class MainWindow(QtWidgets.QMainWindow):
    """Top-level application window."""

    # Save/load data directories
    SAVE_DIR_AIRCRAFT = SAVE_DIR_AIRCRAFT
    SAVE_DIR_TERRAIN = SAVE_DIR_TERRAIN

    # ── lifecycle ─────────────────────────────────────

    def __init__(self):
        super().__init__()
        self.setWindowTitle("3DSceneSoftware — 3D 大场景可视化与目标建模")
        self.resize(1296, 816)

        # ── Config ──
        self.config = load_config()

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

        # DEM scene flag (set by _import_dem_model, used to distinguish from default scene)
        self._dem_scene_active = False

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

        # Flight animation — multi-aircraft support
        self._flights = {}   # name → {timer, path, segments, seg_idx, step, formation, cache, ...}
        self._flight_active = False  # property; check _flights
        self._flight_data_cache = None
        self._flight_camera_follow = False

        # Per-aircraft waypoints (ID-20)
        self._aircraft_waypoints = {}   # name → list of waypoints (shared path attribute)
        # Saved flight states (ID-20)

        self._flight_window = None

        # Transform log throttle (single-shot timer, resets on each slider move)
        self._pending_transform_log = None
        self._transform_log_timer = QTimer(self)
        self._transform_log_timer.setSingleShot(True)
        self._transform_log_timer.timeout.connect(self._flush_transform_log)

        # Formation state
        self._formations = {}  # name → {members, waypoints, leader, tree_node}
        self._formation_saved_wp = {}  # snapshot for cancel/restore

        # Undo stack for slider adjustments (max 20 entries per object)
        self._undo_stack = []           # [(name, {"offset": [...], "scale": ..., "yaw": ..., "pitch": ..., "roll": ...})]

        # Per-aircraft waypoint colours (for visual distinction)
        self._aircraft_colors = {}      # name → colour hex string
        self._color_palette = ["#E53935", "#1E88E5", "#43A047", "#FB8C00",
                               "#8E24AA", "#00ACC1", "#F4511E", "#546E7A"]

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
        self._prop_pages = {}
        self._tree_items = {}
        self._setup_menus()
        self._setup_toolbar()
        self._setup_docks()
        self._refresh_ui()

        self.meas_tool.on_measurement = self._on_measurement_log

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

        # ── Ctrl+Z undo shortcut ──
        undo_shortcut = QtWidgets.QShortcut(QtGui.QKeySequence("Ctrl+Z"), self)
        undo_shortcut.activated.connect(self._undo_last_transform)


    # ═══════════════════════════════════════════════════════════════
    #  Scene management
    # ═══════════════════════════════════════════════════════════════

    def _init_scene(self):
        """Build and display the default scene. Resets camera to fit default scale."""
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

        # Always reset camera to default — critical when switching from DEM
        p.camera_position = [
            DEFAULT_CAMERA_POSITION,
            DEFAULT_CAMERA_FOCAL,
            DEFAULT_CAMERA_UP,
        ]
        p.camera.focal_point = DEFAULT_CAMERA_FOCAL
        p.camera.clipping_range = (0.1, 100.0)
        p.render()
        self._dem_scene_active = False
        self._log_action("加载默认场景")

    # ── Event-driven terrain change broadcast ─────────

    def _on_terrain_changed(self, source_path=""):
        """Event handler: broadcast terrain change to all dependent subsystems."""
        # 0. Stop any active flight (old paths invalid on new terrain)
        if self._flight_active or self._flights:
            self._stop_flight()

        # 1. Clear stale waypoints (global + per-aircraft)
        self._clear_waypoints()

        # 2. Update config
        if source_path:
            self.config["dem_source_path"] = source_path
            save_config(self.config)

        # 3. Refresh UI
        self._refresh_obj_combo()
        self._refresh_scene_objects_ui()
        self._sync_obj_combo_selection("aircraft")
        self._lazy_load_waypoints(
            self._tree_items.get(SceneNodeType.AIRCRAFT), "aircraft")

        # 4. Log
        mode = "DEM" if self._dem_scene_active else "默认"
        self._log_action(f"地形已变更 ({mode}场景) — 飞行已停止，路径点已清空")

        if self._dem_scene_active:
            self.statusBar().showMessage("DEM 地形已加载 — 路径点已清空，相机已适配", 5000)

    def _reapply_elevation_scale(self):
        """Allow user to re-adjust Z exaggeration after DEM import — live update.
        
        Rebuilds terrain mesh with new vert_exag, updates actors, and logs."""
        if not self._dem_scene_active:
            QtWidgets.QMessageBox.information(self, "提示",
                "当前未加载 DEM 地形，无需调整垂直夸张系数。\n"
                "请先通过 文件 → 导入 DEM 模型 加载地形。")
            return

        info = self.scene_objects.get("terrain")
        if info is None:
            return

        original_z = info.get("extra", {}).get("original_z")
        if original_z is None:
            QtWidgets.QMessageBox.warning(self, "无法调整",
                "未找到原始高程数据（original_z），无法重新计算夸张系数。")
            return

        current = self.config.get("elevation_scale", DEM_DEFAULT_VERT_EXAG)
        new_ve, ok = QtWidgets.QInputDialog.getDouble(
            self, "重新调整 Z 夸张",
            "垂直夸张系数 (Z exaggeration):\n"
            ">1 拉高山脉, <1 压低地形",
            value=current, min=DEM_VERT_EXAG_MIN, max=DEM_VERT_EXAG_MAX, decimals=1,
        )
        if not ok or abs(new_ve - current) < 0.01:
            return

        try:
            import numpy as np
            xx = info["extra"]["X"]
            yy = info["extra"]["Y"]
            z_scaled = original_z * new_ve
            pts = np.column_stack((xx.ravel(), yy.ravel(), z_scaled.ravel()))
            grid = pv.StructuredGrid()
            grid.points = pts.astype(np.float32)
            grid.dimensions = (original_z.shape[1], original_z.shape[0], 1)
            grid["elevation"] = z_scaled.ravel()
            info["mesh"] = grid

            self._rebuild_actor("terrain")
            self.config["elevation_scale"] = new_ve
            save_config(self.config)
            self.plotter.render()
            self._log_action(
                f"重新调整 Z 垂直夸张: {current:.1f} → {new_ve:.1f}")
            self.statusBar().showMessage(
                f"Z 夸张已更新: {current:.1f} → {new_ve:.1f}", 5000)
        except Exception as e:
            QtWidgets.QMessageBox.warning(self, "调整失败",
                f"无法应用新的垂直夸张系数:\n{e}")

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
                arr = vtkStringArray()
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
        picker = vtkPropPicker()
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
        fm.addAction("导入 DEM 模型 (HFA/GeoTIFF)...", self._import_dem_model)
        fm.addAction("导入 ASC 格网数据...", self._import_asc_grid)
        fm.addAction("导出 ASC 格网数据...", self._export_asc_grid)
        fm.addSeparator()
        fm.addAction("导出选中模型...", self._export_selected)
        fm.addSeparator()
        fm.addAction("导出操作日志...", self._export_log)
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
        tm.addAction("重新调整 Z 垂直夸张...", self._reapply_elevation_scale)
        tm.addSeparator()

        # ── Layer management ──
        lm = mb.addMenu("图层 (&L)")
        lm.addAction("增加图层...", self._open_layer_dialog)
        lm.addAction("图层管理...", self._open_layer_manager)

        # ── Help ──
        hm = mb.addMenu("帮助 (&H)")
        hm.addAction("用户手册", self._open_user_manual)
        hm.addSeparator()
        hm.addAction("关于", self._show_about)

        # Register for mode-button syncing
        self._mode_buttons.append((self._action_meas_dist, InteractionMode.MEASURE_DISTANCE))
        self._mode_buttons.append((self._action_meas_angle, InteractionMode.MEASURE_ANGLE))

    def _build_layer_menu_action(self, layer_key, layer_label):
        """Build a QWidgetAction embedding a checkbox + opacity slider for *layer_key*.

        This preserves the same two-control layout (visibility toggle +
        transparency slider) that the dock panel provided, now inside the
        Layer menu.
        """
        widget = QtWidgets.QWidget()
        lo = QtWidgets.QHBoxLayout(widget)
        lo.setContentsMargins(4, 1, 4, 1)
        lo.setSpacing(4)

        chk = QtWidgets.QCheckBox(layer_label)
        info = self.scene_objects.get(layer_key, {})
        chk.setChecked(info.get("visible", True))
        chk.toggled.connect(
            lambda checked, n=layer_key: self._toggle_terrain_layer(n, checked)
        )
        lo.addWidget(chk)
        self._terrain_chks[layer_key] = chk

        slider_w, slider_setter = self._create_opacity_slider(
            0.0,
            lambda val, n=layer_key: self._on_terrain_opacity(n, val),
        )
        lo.addWidget(slider_w)
        self._terrain_opacity_sliders[layer_key] = slider_w
        self._terrain_opacity_setters[layer_key] = slider_setter

        act = QtWidgets.QWidgetAction(self)
        act.setDefaultWidget(widget)
        return act

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
        self._create_shared_widgets()

        dock_sb = QtWidgets.QDockWidget("场景浏览器", self)
        dock_sb.setFeatures(QtWidgets.QDockWidget.DockWidgetMovable)

        self._scene_browser = QtWidgets.QTreeWidget()
        self._scene_browser.setHeaderHidden(True)
        self._scene_browser.setAnimated(True)
        self._scene_browser.setIndentation(15)
        self._scene_browser.setContextMenuPolicy(Qt.CustomContextMenu)
        self._scene_browser.customContextMenuRequested.connect(self._on_tree_context_menu)
        self._scene_browser.itemClicked.connect(self._on_tree_item_clicked)
        self._scene_browser.itemDoubleClicked.connect(self._on_tree_item_double_clicked)
        self._scene_browser.itemExpanded.connect(self._on_tree_item_expanded)

        tb_widget = QtWidgets.QWidget()
        tb_layout = QtWidgets.QHBoxLayout(tb_widget)
        tb_layout.setContentsMargins(4, 2, 4, 2)
        self._btn_add = QtWidgets.QPushButton("+ 添加")
        self._btn_delete = QtWidgets.QPushButton("- 删除")
        self._btn_add.setFixedWidth(60)
        self._btn_delete.setFixedWidth(60)
        tb_layout.addWidget(self._btn_add)
        tb_layout.addWidget(self._btn_delete)
        tb_layout.addStretch()

        container = QtWidgets.QWidget()
        container_layout = QtWidgets.QVBoxLayout(container)
        container_layout.setContentsMargins(0, 0, 0, 0)
        container_layout.setSpacing(0)
        container_layout.addWidget(tb_widget)
        container_layout.addWidget(self._scene_browser)
        dock_sb.setWidget(container)
        self.addDockWidget(Qt.LeftDockWidgetArea, dock_sb)

        self._tree_items = SceneTreeFactory.build_tree(self._scene_browser)

        self._btn_add.clicked.connect(self._on_tree_add)
        self._btn_delete.clicked.connect(self._on_tree_delete)

        self._property_dock = QtWidgets.QDockWidget("属性", self)
        self._property_stack = QtWidgets.QStackedWidget()
        self._property_dock.setWidget(self._property_stack)
        self._property_dock.setMinimumWidth(180)
        self.addDockWidget(Qt.RightDockWidgetArea, self._property_dock)

        self._setup_property_pages()
        self._property_stack.setCurrentIndex(0)

        self._timeline_dock = QtWidgets.QDockWidget("飞行时间轴", self)
        self._timeline_container = QtWidgets.QWidget()
        tl = QtWidgets.QVBoxLayout(self._timeline_container)
        tl.setContentsMargins(0, 0, 0, 0)
        tl.addWidget(QtWidgets.QLabel("— 飞行时间轴 —"))
        self._keyframe_widget = QtWidgets.QWidget()
        self._kf_layout = QtWidgets.QHBoxLayout(self._keyframe_widget)
        self._kf_layout.setContentsMargins(0, 0, 0, 0)
        tl.addWidget(self._keyframe_widget)
        self._tl_time_label = QtWidgets.QLabel("0.0 / 0.0 秒")
        tl.addWidget(self._tl_time_label)
        self._tl_slider = QtWidgets.QSlider(Qt.Horizontal)
        self._tl_slider.setRange(0, 1000)
        self._tl_slider.sliderPressed.connect(self._on_tl_pressed)
        self._tl_slider.valueChanged.connect(self._on_tl_seek)
        self._tl_slider.sliderReleased.connect(self._on_tl_released)
        tl.addWidget(self._tl_slider)
        self._timeline_dock.setWidget(self._timeline_container)
        self._timeline_dock.setVisible(False)
        self.addDockWidget(Qt.BottomDockWidgetArea, self._timeline_dock)
        self._timeline_dock.raise_()

        # ── Bottom: 操作日志 (左) + 坐标信息 (右) ──
        dock_bottom = QtWidgets.QDockWidget("操作日志 & 坐标信息", self)
        bottom_widget = QtWidgets.QWidget()
        bottom_layout = QtWidgets.QHBoxLayout(bottom_widget)
        bottom_layout.setContentsMargins(4, 2, 4, 2)

        self._log_text = QtWidgets.QTextEdit()
        self._log_text.setReadOnly(True)
        self._log_text.setMaximumHeight(150)
        self._log_text.setFont(QtGui.QFont("Menlo", 9))
        bottom_layout.addWidget(self._log_text, 2)

        self._info_text = QtWidgets.QTextEdit()
        self._info_text.setReadOnly(True)
        self._info_text.setMaximumHeight(150)
        self._info_text.setFont(QtGui.QFont("Menlo", 10))
        self._info_text.setMaximumWidth(400)
        bottom_layout.addWidget(self._info_text, 1)

        dock_bottom.setWidget(bottom_widget)
        self.addDockWidget(Qt.BottomDockWidgetArea, dock_bottom)

    def _create_shared_widgets(self):
        self._obj_combo = QtWidgets.QComboBox()
        self._slider_obj_x = self._make_slider(-15, 15, 0, self._on_obj_pos_x)
        self._slider_obj_y = self._make_slider(-15, 15, 0, self._on_obj_pos_y)
        self._slider_obj_z = self._make_slider(-15, 15, 0, self._on_obj_pos_z)
        self._slider_obj_s = self._make_slider(0.1, 5.0, 1.0, self._on_obj_scale)

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

        self._scene_obj_chks = {}
        self._custom_obj_chks = {}

        self._so_layout_host = QtWidgets.QWidget()
        self._so_layout_host.setVisible(False)
        self._so_layout = QtWidgets.QVBoxLayout(self._so_layout_host)
        self._so_layout.setContentsMargins(4, 4, 4, 4)
        self._so_layout.setSpacing(3)
        self._so_layout.addStretch()

    def _setup_property_pages(self):
        blank = QtWidgets.QWidget()
        self._property_stack.addWidget(blank)
        self._prop_pages[""] = blank

        pages = [
            (SceneNodeType.AIRCRAFT, self._build_aircraft_property_page),
            (SceneNodeType.WAYPOINT, self._build_waypoint_property_page),
            (SceneNodeType.ANIMATION_TASK, self._build_animation_property_page),
        ]
        for node_type, builder in pages:
            page = builder()
            self._property_stack.addWidget(page)
            self._prop_pages[node_type] = page

    def _build_aircraft_property_page(self):
        page = QtWidgets.QWidget()
        lo = QtWidgets.QVBoxLayout(page)
        lo.setContentsMargins(4, 4, 4, 4)
        lo.addWidget(QtWidgets.QLabel("选中对象:"))
        lo.addWidget(self._obj_combo)
        lo.addWidget(QtWidgets.QLabel("位置 X"))
        lo.addWidget(self._slider_obj_x)
        lo.addWidget(QtWidgets.QLabel("位置 Y"))
        lo.addWidget(self._slider_obj_y)
        lo.addWidget(QtWidgets.QLabel("位置 Z"))
        lo.addWidget(self._slider_obj_z)
        lo.addWidget(QtWidgets.QLabel("缩放"))
        lo.addWidget(self._slider_obj_s)
        lo.addWidget(self._attitude_container)
        lo.addStretch()
        return page

    def _build_waypoint_property_page(self):
        page = QtWidgets.QWidget()
        lo = QtWidgets.QVBoxLayout(page)
        lo.setSpacing(6)
        lo.addWidget(QtWidgets.QLabel("路径点"))
        self._wp_idx_label = QtWidgets.QLabel("序号: -")
        lo.addWidget(self._wp_idx_label)
        lo.addStretch()
        return page

    def _build_animation_property_page(self):
        page = QtWidgets.QWidget()
        lo = QtWidgets.QVBoxLayout(page)
        lo.setSpacing(6)
        lo.addWidget(QtWidgets.QLabel("飞行动画控制"))
        flight_row = QtWidgets.QHBoxLayout()
        flight_row.addWidget(QtWidgets.QLabel("选择飞机:"))
        self._flight_aircraft_combo = QtWidgets.QComboBox()
        flight_row.addWidget(self._flight_aircraft_combo)
        lo.addLayout(flight_row)
        self._btn_start_flight = QtWidgets.QPushButton("开始飞行")
        self._btn_start_flight.clicked.connect(self._toggle_flight)
        lo.addWidget(self._btn_start_flight)
        self._chk_camera_follow = QtWidgets.QCheckBox("相机自动跟随")
        self._chk_camera_follow.setChecked(False)
        self._chk_camera_follow.toggled.connect(
            lambda checked: setattr(self, '_flight_camera_follow', checked))
        lo.addWidget(self._chk_camera_follow)
        self._btn_save_flight = QtWidgets.QPushButton("保存飞行数据")
        lo.addWidget(self._btn_save_flight)
        self._btn_load_flight = QtWidgets.QPushButton("载入飞行数据")
        lo.addWidget(self._btn_load_flight)
        self._btn_formation = QtWidgets.QPushButton("开始编队")
        self._btn_formation.clicked.connect(self._start_formation_dialog)
        lo.addWidget(self._btn_formation)
        self._btn_cancel_formation = QtWidgets.QPushButton("取消编队")
        self._btn_cancel_formation.clicked.connect(self._cancel_formation)
        self._btn_cancel_formation.setVisible(False)
        lo.addWidget(self._btn_cancel_formation)
        lo.addStretch()
        return page

    def _on_tree_item_clicked(self, item, column):
        data = item.data(0, Qt.UserRole)
        if data is None:
            return
        if data.node_type == SceneNodeType.SCENE_SETTINGS:
            return
        page = self._prop_pages.get(data.node_type)
        if page is not None:
            self._property_stack.setCurrentWidget(page)
            self._property_dock.setVisible(True)
        else:
            self._property_dock.setVisible(False)
        self._timeline_dock.setVisible(data.node_type == SceneNodeType.ANIMATION_TASK)
        if data.node_type == SceneNodeType.WAYPOINT and data.aircraft_name is not None and data.waypoint_index is not None:
            self._load_waypoint_to_property_page(data.aircraft_name, data.waypoint_index)
        if data.node_type == SceneNodeType.AIRCRAFT and data.scene_obj_name:
            self._sync_obj_combo_selection(data.scene_obj_name)

    def _on_tree_item_double_clicked(self, item, column):
        data = item.data(0, Qt.UserRole)
        if data is None:
            return
        if data.node_type == SceneNodeType.WAYPOINT:
            self._on_waypoint_double_click(data)
        elif data.node_type == SceneNodeType.AIRCRAFT:
            self._on_aircraft_double_click(data)
        elif data.slot_name:
            self._route_slot_action(data)

    def _on_waypoint_double_click(self, data):
        ac_name = data.aircraft_name
        wp_idx = data.waypoint_index
        if ac_name is None or wp_idx is None:
            return
        # Show coordinate editing dialog
        self._edit_waypoint_dialog(ac_name, wp_idx)

    def _edit_waypoint_dialog(self, ac_name, wp_idx):
        waypoints = self._aircraft_waypoints.get(ac_name, [])
        if wp_idx < 0 or wp_idx >= len(waypoints):
            return
        wp = waypoints[wp_idx]
        dlg = QtWidgets.QDialog(self)
        dlg.setWindowTitle(f"编辑路径点 #{wp_idx + 1}")
        dlg.resize(320, 200)
        lo = QtWidgets.QVBoxLayout(dlg)
        lo.addWidget(QtWidgets.QLabel(f"路径点 #{wp_idx + 1} 坐标"))
        form = QtWidgets.QFormLayout()
        sx = QtWidgets.QDoubleSpinBox()
        sx.setRange(-1e9, 1e9); sx.setDecimals(2); sx.setValue(wp[0])
        sy = QtWidgets.QDoubleSpinBox()
        sy.setRange(-1e9, 1e9); sy.setDecimals(2); sy.setValue(wp[1])
        sz = QtWidgets.QDoubleSpinBox()
        sz.setRange(-1e9, 1e9); sz.setDecimals(2); sz.setValue(wp[2])
        form.addRow("X:", sx)
        form.addRow("Y:", sy)
        form.addRow("Z:", sz)
        lo.addLayout(form)
        btns = QtWidgets.QDialogButtonBox(
            QtWidgets.QDialogButtonBox.Ok | QtWidgets.QDialogButtonBox.Cancel)
        btns.accepted.connect(dlg.accept)
        btns.rejected.connect(dlg.reject)
        lo.addWidget(btns)
        if dlg.exec_() != QtWidgets.QDialog.Accepted:
            return
        new_wp = np.array([sx.value(), sy.value(), sz.value()])
        waypoints[wp_idx] = new_wp
        if ac_name and ac_name in self._aircraft_waypoints:
            self._aircraft_waypoints[ac_name][wp_idx] = new_wp
        self._rebuild_waypoint_actors()
        self._log_action(f"用户编辑了路径点 #{wp_idx+1}: ({new_wp[0]:.2f}, {new_wp[1]:.2f}, {new_wp[2]:.2f})")
        self.statusBar().showMessage(f"路径点 #{wp_idx+1} 已更新", 3000)

    def _on_aircraft_double_click(self, data):
        if not data.scene_obj_name:
            return
        try:
            self._sync_obj_combo_selection(data.scene_obj_name)
        except Exception as e:
            import traceback
            traceback.print_exc()
            QtWidgets.QMessageBox.warning(self, "错误",
                "选择飞机时出错: {}".format(e))
            return
        page = self._prop_pages.get(SceneNodeType.AIRCRAFT)
        if page:
            self._property_stack.setCurrentWidget(page)

    def _route_slot_action(self, data):
        slot = data.slot_name
        if slot == "_on_formation_toggled":
            self._start_formation_dialog()
        elif slot == "meas_tool.clear_all":
            self.meas_tool.clear_all()
        elif slot == "meas_tool.undo_last":
            self.meas_tool.undo_last()
        elif slot == "_clear_waypoints":
            self._confirm_clear_waypoints()
        else:
            method = getattr(self, slot, None)
            if method and callable(method):
                method()

    def _open_scene_settings_dialog(self):
        dlg = SceneSettingsDialog(self)
        dlg.setAttribute(QtCore.Qt.WA_DeleteOnClose)
        dlg.destroyed.connect(lambda: setattr(self, '_scene_settings_dialog', None))
        dlg.show()
        self._scene_settings_dialog = dlg

    def _on_tree_item_expanded(self, item):
        data = item.data(0, Qt.UserRole)
        if data is None:
            return
        if data.node_type == SceneNodeType.AIRCRAFT and data.scene_obj_name:
            self._lazy_load_waypoints(item, data.scene_obj_name)

    def _on_tree_context_menu(self, pos):
        item = self._scene_browser.itemAt(pos)
        if item is None:
            return
        data = item.data(0, Qt.UserRole)
        if data is None:
            return
        menu = QtWidgets.QMenu(self)
        if data.is_deletable:
            act_del = menu.addAction("删除" + data.label + "")
            act_del.triggered.connect(lambda: self._on_tree_delete_item(item))
        if data.is_editable:
            act_rename = menu.addAction("重命名")
            act_rename.triggered.connect(lambda: self._on_tree_rename_item(item))
        menu.exec_(self._scene_browser.viewport().mapToGlobal(pos))

    def _on_tree_delete_item(self, item):
        data = item.data(0, Qt.UserRole)
        if data is None:
            return
        if data.node_type == SceneNodeType.AIRCRAFT and data.scene_obj_name:
            self._delete_aircraft(data.scene_obj_name)
            parent = item.parent()
            if parent:
                parent.removeChild(item)
            self._refresh_ui()
        elif data.node_type == SceneNodeType.WAYPOINT:
            self._delete_waypoint(data.aircraft_name, data.waypoint_index)
            parent = item.parent()
            if parent:
                parent.removeChild(item)
            self._rebuild_waypoint_actors()
        elif "编队" in data.label:
            self._cancel_formation()
            # Tree node removal handled by _cancel_formation

    def _on_tree_rename_item(self, item):
        idx = self._scene_browser.indexOfTopLevelItem(item)
        if idx >= 0:
            return
        self._scene_browser.editItem(item, 0)

    def _on_tree_add(self):
        menu = QtWidgets.QMenu(self)
        act_ac = menu.addAction("添加新的飞机")
        act_ac.triggered.connect(self._add_new_aircraft)
        act_wp = menu.addAction("添加新的路径点")
        act_wp.triggered.connect(self._open_precise_wp_dialog)
        menu.exec_(self._btn_add.mapToGlobal(QtCore.QPoint(0, self._btn_add.height())))

    def _on_tree_delete(self):
        item = self._scene_browser.currentItem()
        if item is None:
            self.statusBar().showMessage("⚠ 请先在场景树中选中要删除的节点", 3000)
            return
        data = item.data(0, Qt.UserRole)
        if data is None:
            return
        if not data.is_deletable:
            QtWidgets.QMessageBox.information(self, "提示", "该节点不可删除（受系统保护）")
            return
        reply = QtWidgets.QMessageBox.question(self, "确认删除",
            "确定要删除此节点吗?")
        if reply != QtWidgets.QMessageBox.Yes:
            return
        self._on_tree_delete_item(item)

    def _add_new_aircraft(self):
        used = set()
        for name in self.scene_objects:
            low = name.lower()
            if "aircraft" not in low:
                continue
            if name == "aircraft":
                used.add(1)
                continue
            suffix = name[len("aircraft"):]
            if suffix.isdigit():
                used.add(int(suffix))
        idx = 1
        while idx in used:
            idx += 1
        new_name = "aircraft" if idx == 1 else "aircraft{}".format(idx)

        mesh = _build_airplane_mesh().copy()
        if self._dem_scene_active:
            mesh.scale(AIRCRAFT_DEFAULT_SCALE, inplace=True)
            offset = (idx - 1) * AIRCRAFT_DEFAULT_SCALE * 0.005
            mesh.translate((offset, offset * 0.5, 7000.0), inplace=True)
        else:
            mesh.scale(1.5, inplace=True)
            offset = (idx - 1) * 3.0
            mesh.translate((-6 + offset, -2 + offset * 0.5, 8), inplace=True)

        color = "orange" if idx > 2 else ("grey" if idx == 1 else "red")
        self.scene_objects[new_name] = {
            "mesh": mesh, "type": "mesh", "visible": True,
            "params": {
                "color": color, "smooth_shading": True,
                "ambient": 0.3, "diffuse": 0.7,
                "specular": 0.2, "specular_power": 30,
            },
            "extra": None, "name": new_name,
        }
        self._get_or_init_transform(new_name)
        self._add_actor(new_name, self.scene_objects[new_name])
        self._insert_aircraft_node(new_name)
        self._refresh_ui()
        # Assign color for new aircraft
        idx = len(self._aircraft_colors)
        self._aircraft_colors[new_name] = self._color_palette[idx % len(self._color_palette)]
        self._log_action(f"用户增加了飞机: {new_name}")
        self.statusBar().showMessage("已添加新飞机: {}".format(new_name), 3000)

    def _insert_aircraft_node(self, name):
        flight_root = self._tree_items.get(SceneNodeType.FLIGHT_PLATFORM)
        if flight_root is None:
            return
        SceneTreeFactory._create_item(
            flight_root,
            NodeData(
                node_type=SceneNodeType.AIRCRAFT,
                label=name,
                scene_obj_name=name,
                tooltip="飞机: {}".format(name),
                is_editable=True,
                is_deletable=True,
            ),
        )

    def _delete_aircraft(self, name):
        if name is None:
            return
        self._remove_actor(name)
        self.scene_objects.pop(name, None)
        self._obj_transforms.pop(name, None)
        self._aircraft_waypoints.pop(name, None)
        self._aircraft_colors.pop(name, None)
        self._stop_flight(name)
        # Remove from formation if member of any
        for fname, form in list(self._formations.items()):
            if name in form["members"]:
                form["members"].remove(name)
        if any(not f["members"] for f in self._formations.values()):
            self._formations.clear()
            self._btn_formation.setText("开始编队")
            self._btn_formation.setEnabled(True)
            self._btn_cancel_formation.setVisible(False)
        # Clean up flight preview line if present
        if self._path_actor is not None:
            try:
                self.plotter.remove_actor(self._path_actor)
            except Exception:
                pass
            self._path_actor = None
        self._rebuild_waypoint_actors()
        self._populate_aircraft_nodes()
        self._refresh_waypoint_tree()
        self._log_action(f"用户删除了飞机: {name}")

    def _delete_waypoint(self, ac_name, wp_idx):
        if wp_idx is None:
            return
        waypoints = self._aircraft_waypoints.get(ac_name, [])
        if 0 <= wp_idx < len(waypoints):
            waypoints.pop(wp_idx)
            if not waypoints:
                self._aircraft_waypoints.pop(ac_name, None)
        self._rebuild_waypoint_actors()
        self._log_action(f"用户删除了 {ac_name} 的路径点 #{wp_idx + 1}")

    def _rebuild_waypoint_actors(self):
        for a in self._wp_actors:
            try:
                self.plotter.remove_actor(a)
            except Exception:
                pass
        self._wp_actors.clear()
        if self._path_actor is not None:
            try:
                self.plotter.remove_actor(self._path_actor)
            except Exception:
                pass
            self._path_actor = None
        # Draw per-aircraft waypoints with distinct colours
        global_idx = 1
        for ac_name, wps in self._aircraft_waypoints.items():
            color = self._aircraft_colors.get(ac_name, "red")
            for wp in wps:
                wp_arr = np.asarray(wp)
                sphere = pv.Sphere(radius=0.15, center=wp_arr)
                actor = self.plotter.add_mesh(sphere, color=color, smooth_shading=True)
                self._wp_actors.append(actor)
                label_actor = self.plotter.add_point_labels(
                    np.array([wp_arr]), [f"{global_idx}"],
                    show_points=False, font_size=14, text_color=color,
                    shape_opacity=0.0, always_visible=True,
                )
                self._wp_actors.append(label_actor)
                global_idx += 1
        self.plotter.render()

    def _populate_aircraft_nodes(self):
        flight_root = self._tree_items.get(SceneNodeType.FLIGHT_PLATFORM)
        if flight_root is None:
            return
        # Remove only aircraft nodes (preserve formation nodes)
        i = 0
        while i < flight_root.childCount():
            ac = flight_root.child(i)
            ds = ac.data(0, Qt.UserRole)
            if ds and ds.scene_obj_name and "aircraft" in ds.scene_obj_name.lower():
                flight_root.removeChild(ac)
            else:
                i += 1
        for obj_name in self.scene_objects:
            if "aircraft" not in obj_name.lower():
                continue
            self._insert_aircraft_node(obj_name)
        flight_root.setExpanded(True)

    def _lazy_load_waypoints(self, aircraft_item, aircraft_name):
        if aircraft_item.childCount() > 0:
            return
        waypoints = self._aircraft_waypoints.get(aircraft_name, [])
        for i in range(len(waypoints)):
            wp = waypoints[i]
            time_label = "t={:.1f}s".format(i * 5.0)
            SceneTreeFactory._create_item(
                aircraft_item,
                NodeData(
                    node_type=SceneNodeType.WAYPOINT,
                    label="路径点 {}".format(i + 1),
                    scene_obj_name=None,
                    waypoint_index=i,
                    aircraft_name=aircraft_name,
                    tooltip="{}  {}  ({:.1f}, {:.1f}, {:.1f})".format(
                        time_label, i + 1, wp[0], wp[1], wp[2]),
                    is_editable=True,
                    is_deletable=True,
                ),
            )

    def _load_waypoint_to_property_page(self, ac_name, wp_idx):
        waypoints = self._aircraft_waypoints.get(ac_name) or self.waypoints
        if wp_idx < 0 or wp_idx >= len(waypoints):
            return
        wp = waypoints[wp_idx]
        page = self._prop_pages.get(SceneNodeType.WAYPOINT)
        if page is None:
            return
        self._property_stack.setCurrentWidget(page)
        self._property_dock.setVisible(True)
        if hasattr(self, '_wp_idx_label'):
            self._wp_idx_label.setText("序号: {}  ({:.1f}, {:.1f}, {:.1f})".format(
                wp_idx + 1, wp[0], wp[1], wp[2]))
        self._highlight_waypoint(wp_idx)

    def _highlight_waypoint(self, wp_idx):
        actor_idx = wp_idx * 2
        if actor_idx < len(self._wp_actors):
            sphere_actor = self._wp_actors[actor_idx]
            try:
                prop = sphere_actor.GetProperty()
                prop.SetColor(1.0, 0.84, 0.0)
                prop.SetEdgeColor(1.0, 0.0, 0.0)
                prop.SetLineWidth(3)
            except Exception:
                pass

    def _sync_obj_combo_selection(self, name):
        idx = self._obj_combo.findText(name)
        if idx >= 0:
            self._obj_combo.blockSignals(True)
            self._obj_combo.setCurrentIndex(idx)
            self._obj_combo.blockSignals(False)
            self._on_obj_select_changed(idx)

    def _open_flight_aircraft_dialog(self):
        names = [n for n in self.scene_objects if "aircraft" in n.lower()]
        if not names:
            self.statusBar().showMessage("⚠ 当前场景无飞机", 3000)
            return

        dlg = QtWidgets.QDialog(self)
        dlg.setWindowTitle("选择飞行飞机")
        dlg.setWindowFlags(dlg.windowFlags() | QtCore.Qt.Window)
        dlg.resize(350, 300)
        lo = QtWidgets.QVBoxLayout(dlg)
        lo.addWidget(QtWidgets.QLabel("勾选要飞行的飞机（可多选）:"))
        checks = {}
        for n in names:
            chk = QtWidgets.QCheckBox(n)
            if n == self._flight_aircraft_combo.currentText():
                chk.setChecked(True)
            lo.addWidget(chk)
            checks[n] = chk

        # Delay options (shown if >1 selected)
        delay_widget = QtWidgets.QWidget()
        delay_lo = QtWidgets.QVBoxLayout(delay_widget)
        delay_lo.setContentsMargins(0, 0, 0, 0)
        delay_chk = QtWidgets.QCheckBox("启用延迟出发")
        delay_lo.addWidget(delay_chk)
        delay_form = QtWidgets.QFormLayout()
        delay_spins = {}
        for n in names[1:]:
            sb = QtWidgets.QDoubleSpinBox()
            sb.setRange(0, 300); sb.setValue(0); sb.setSuffix(" 秒")
            delay_spins[n] = sb
            delay_form.addRow(f"{n} 延迟:", sb)
        delay_lo.addLayout(delay_form)
        delay_widget.setVisible(False)
        lo.addWidget(delay_widget)

        # Show/hide delay based on checkbox count
        def _update_delay_visibility():
            cnt = sum(1 for c in checks.values() if c.isChecked())
            delay_widget.setVisible(cnt > 1)
        for chk in checks.values():
            chk.toggled.connect(_update_delay_visibility)

        btns = QtWidgets.QDialogButtonBox(
            QtWidgets.QDialogButtonBox.Ok | QtWidgets.QDialogButtonBox.Cancel)
        btns.accepted.connect(dlg.accept)
        btns.rejected.connect(dlg.reject)
        lo.addWidget(btns)
        if dlg.exec_() != QtWidgets.QDialog.Accepted:
            return

        selected = [n for n in names if checks[n].isChecked()]
        if not selected:
            return
        delays = {}
        if delay_chk.isChecked():
            for n, sb in delay_spins.items():
                v = sb.value()
                if v > 0:
                    delays[n] = float(v)
        self._execute_flight(selected, delays)

    def _execute_flight(self, selected, delays=None):
        if delays is None:
            delays = {}
        self._start_flight(selected[0])
        for n in selected[1:]:
            d = delays.get(n, 0.0)
            if d > 0.1:
                QtCore.QTimer.singleShot(int(d * 1000),
                    lambda name=n: self._start_flight(name))
            else:
                self._start_flight(n)

    def _toggle_flight(self):
        if self._flight_active:
            self._stop_flight()
        else:
            self._open_flight_aircraft_dialog()

    # ── Slider factory ─────────────────────────────

    @staticmethod
    def _make_slider(vmin, vmax, initial, callback, steps=1000):
        """Create a horizontal slider with live value label.

        Stores ``vmin``/``vmax`` as attributes on the returned widget so
        the range can be updated dynamically via ``_update_slider_range``.

        Returns the container widget (use ``.findChild(QtWidgets.QSlider)``
        to access the slider if needed).
        """
        w = QtWidgets.QWidget()
        w._slider_vmin = vmin
        w._slider_vmax = vmax
        w._slider_steps = steps
        w._slider_callback = callback
        lo = QtWidgets.QHBoxLayout(w)
        lo.setContentsMargins(0, 0, 0, 0)
        s = QtWidgets.QSlider(Qt.Horizontal)
        s.setRange(0, steps)
        frac = (initial - vmin) / (vmax - vmin) if (vmax - vmin) != 0 else 0.0
        s.setValue(int(frac * steps))
        label = QtWidgets.QLabel(f"{initial:.2f}")
        label.setMinimumWidth(60)
        lo.addWidget(s, 1)
        lo.addWidget(label)

        def on_change(val):
            f = val / float(w._slider_steps)
            vmin_cur = w._slider_vmin
            vmax_cur = w._slider_vmax
            real = vmin_cur + f * (vmax_cur - vmin_cur)
            label.setText(f"{real:.2f}")
            w._slider_callback(real)

        s.valueChanged.connect(on_change)
        return w

    def _update_slider_range(self, slider_widget, vmin, vmax):
        """Change the range of a slider made by ``_make_slider`` in-place."""
        slider_widget._slider_vmin = vmin
        slider_widget._slider_vmax = vmax
        s = slider_widget.findChild(QtWidgets.QSlider)
        if s is None:
            return
        # Keep current value within new range
        label = slider_widget.findChild(QtWidgets.QLabel)
        try:
            val = float(label.text())
        except (ValueError, AttributeError):
            val = 0.0
        val = max(vmin, min(vmax, val))
        frac = (val - vmin) / (vmax - vmin) if (vmax - vmin) != 0 else 0.0
        s.blockSignals(True)
        s.setValue(int(frac * slider_widget._slider_steps))
        s.blockSignals(False)
        if label:
            label.setText(f"{val:.2f}")

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
        info = self.scene_objects.get(name)
        if info is None:
            return
        info["visible"] = visible
        if visible:
            self._rebuild_actor(name)
        else:
            self._remove_actor(name)
        self.plotter.render()
        action_label = "显示" if visible else "隐藏"
        self._log_action(f"用户{action_label}了图层：{name}")

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
        if name not in self.plotter_actors:
            return
        actor = self.plotter_actors[name]
        vtk_actor = self._resolve_vtk_actor(actor)
        vtk_actor.GetProperty().SetOpacity(value)
        info = self.scene_objects.get(name)
        if info is not None:
            info["params"]["opacity"] = value
        self.plotter.render()
        self._log_action(f"用户调整了图层 [{name}] 不透明度为 {value:.2f}")

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
        is_dem = self._is_dem_scene()
        scene_items = []

        if is_dem:
            # DEM mode: only aircraft, aircraft2, terrain
            for name in ["aircraft", "aircraft2", "terrain"]:
                if name in self.scene_objects:
                    scene_items.append((name, name, False))
        else:
            for name in self.scene_objects:
                if name == "terrain" or name in terrain_names:
                    if name in ("river", "vegetation"):
                        label = self._terrain_layer_names.get(name, name)
                        scene_items.append((name, label, False))
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
                row.addWidget(chk)
                row.addStretch()
            elif name in ("river", "vegetation"):
                chk.setChecked(info.get("visible", True) if info else True)
                new_chks[name] = chk
                chk.toggled.connect(lambda checked, n=name: self._toggle_terrain_layer(n, checked))
                row.addWidget(chk)
                row.addStretch()
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

    def _open_layer_dialog(self):
        """Open the Layer Management dialog for shape-based layer editing."""
        if not _HAS_MPL:
            QtWidgets.QMessageBox.warning(
                self, "提示",
                "图层管理需要 matplotlib 支持。\n请安装: pip install matplotlib"
            )
            return
        try:
            from src.layer_dialog import LayerManagementDialog
        except ImportError:
            QtWidgets.QMessageBox.warning(
                self, "错误",
                "无法加载图层管理模块 (src/layer_dialog.py)"
            )
            return

        is_dem = self._is_dem_scene()
        extent = self._compute_terrain_extent()
        terrain_mesh = self.scene_objects.get("terrain", {}).get("mesh")
        dlg = LayerManagementDialog(
            self,
            layer_names=dict(self._terrain_layer_names),
            terrain_extent=extent,
            is_dem_scene=is_dem,
            terrain_mesh=terrain_mesh,
        )
        if dlg.exec_() != QtWidgets.QDialog.Accepted:
            return

        results = dlg.get_results()

        shapes_by_layer = results.get("layer_shapes", {})

        # Apply layer visibility toggles (skip layers that get shape clips)
        for key, visible in results.get("layer_visibility", {}).items():
            if key in shapes_by_layer:
                continue
            if key in self.scene_objects:
                self._toggle_terrain_layer(key, visible)

        # Apply new shape-based layers and hide the source layer
        if shapes_by_layer:
            self._apply_layer_shapes(shapes_by_layer)
            for key in shapes_by_layer:
                if key in self.scene_objects:
                    self._toggle_terrain_layer(key, False)

        self.plotter.render()

    def _open_layer_manager(self):
        """Open the Clip Manager dialog to manage extracted clip layers."""
        if not _HAS_MPL:
            QtWidgets.QMessageBox.warning(
                self, "提示",
                "图层管理需要 matplotlib 支持。\n请安装: pip install matplotlib"
            )
            return
        try:
            from src.layer_dialog import ClipManagerDialog
        except ImportError:
            QtWidgets.QMessageBox.warning(
                self, "错误",
                "无法加载图层管理模块 (src/layer_dialog.py)"
            )
            return

        dlg = ClipManagerDialog(
            self,
            scene_objects=self.scene_objects,
            plotter_actors=self.plotter_actors,
            toggle_fn=self._toggle_terrain_layer,
            opacity_fn=self._on_terrain_opacity,
            remove_fn=self._remove_clip_layer,
            rebuild_fn=self._rebuild_actor,
        )
        dlg.exec_()

    def _remove_clip_layer(self, name):
        self._remove_actor(name)
        self.scene_objects.pop(name, None)
        self._obj_transforms.pop(name, None)
        self._terrain_chks.pop(name, None)
        self._terrain_opacity_setters.pop(name, None)
        self.plotter.render()
        self._log_action(f"用户移除了图层掩膜：{name}")

    _CLIP_LAYER_PARAMS = {
        "layer_sand":  {"color": "#e8c76a", "smooth_shading": True, "opacity": 1.0},
        "layer_grass": {"color": "#5a9e4c", "smooth_shading": True, "opacity": 1.0},
        "layer_earth": {"color": "#8b6f47", "smooth_shading": True, "opacity": 1.0},
    }

    def _apply_layer_shapes(self, shapes_by_layer):
        """Extract sub-meshes from the terrain surface based on XY shapes.

        Instead of clipping the elevation-thresholded layer mesh (which would
        limit the result to wherever that layer naturally occurs), this clips
        the full terrain surface and colours it with the target layer's
        parameters.  This way the polygon defines the *extent* of the layer.
        """
        terrain_info = self.scene_objects.get("terrain")
        if terrain_info is None:
            return
        grid = terrain_info["mesh"]
        surface = grid.extract_surface()

        from matplotlib.path import Path
        surface_pts = np.array(surface.points)

        z_off = max((surface_pts[:, 2].max() - surface_pts[:, 2].min()) * 0.005, 0.01)

        for layer_key, shapes in shapes_by_layer.items():
            if not shapes:
                continue

            if layer_key in self.scene_objects:
                layer_params = self.scene_objects[layer_key]["params"]
            else:
                layer_params = self._CLIP_LAYER_PARAMS.get(layer_key)
                if layer_params is None:
                    continue

            combined_mask = np.zeros(len(surface_pts), dtype=bool)

            for shape in shapes:
                poly = np.array(shape["xy"])
                if len(poly) < 3:
                    continue
                path = Path(poly)
                mask = path.contains_points(surface_pts[:, :2])
                combined_mask |= mask

            if not combined_mask.any():
                continue

            sub_mesh = surface.extract_points(combined_mask)
            if not sub_mesh.n_points:
                continue

            sub_mesh.translate((0, 0, z_off), inplace=True)

            clip_name = f"{layer_key}_clip"
            idx = 1
            while clip_name in self.scene_objects:
                clip_name = f"{layer_key}_clip_{idx}"
                idx += 1

            opacity = shapes[0].get("opacity", 1.0)
            base_color = layer_params.get("color") or (
                layer_params.get("cmap") and layer_params["cmap"][len(layer_params["cmap"]) // 2]
            ) or self._CLIP_LAYER_PARAMS.get(layer_key, {}).get("color", "#888888")
            clip_params = {
                "color": base_color,
                "smooth_shading": True,
                "opacity": opacity,
            }
            self.scene_objects[clip_name] = {
                "mesh": sub_mesh,
                "type": "mesh",
                "visible": True,
                "params": clip_params,
                "extra": None,
                "name": clip_name,
            }
            self._add_actor(clip_name, self.scene_objects[clip_name])
            self.scene_objects[clip_name]["visible"] = True
            layer_label = self._terrain_layer_names.get(layer_key, layer_key)
            self._log_action(f"用户在场景中应用了【{layer_label}】图层掩膜")

    def _refresh_ui(self):
        """Refresh flight combo, terrain UI, scene-object and tree."""
        self._refresh_obj_combo()
        self._refresh_flight_combo()
        self._refresh_terrain_ui()
        self._refresh_scene_objects_ui()
        self._populate_aircraft_nodes()

    def _refresh_obj_combo(self):
        """Rebuild the object-control combo box.

        In DEM scenes (detected by presence of 'X'/'Y' in terrain extra),
        only show aircraft, aircraft2, terrain — in that order.
        Default selection is aircraft1.
        """
        current = self._obj_combo.currentText()
        self._obj_combo.blockSignals(True)
        self._obj_combo.clear()

        is_dem = self._is_dem_scene()

        if is_dem:
            # DEM mode: terrain first, then all aircraft objects
            self._obj_combo.addItem("terrain")
            for name in self.scene_objects:
                if "aircraft" in name.lower():
                    self._obj_combo.addItem(name)
        else:
            for name in self.scene_objects:
                self._obj_combo.addItem(name)
            for name in self.custom_objects:
                self._obj_combo.addItem(f"[自定义] {name}")

        idx = self._obj_combo.findText(current)
        if idx >= 0:
            self._obj_combo.setCurrentIndex(idx)
        elif is_dem:
            # Default to aircraft1 in DEM mode
            ac_idx = self._obj_combo.findText("aircraft")
            if ac_idx >= 0:
                self._obj_combo.setCurrentIndex(ac_idx)
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

        clean = clean_name.replace("[自定义] ", "")
        t = self._obj_transforms.get(clean_name)
        if t:
            self._log_action(f"用户选中了对象【{clean_name}】，位置 ({t['offset'][0]:.1f}, {t['offset'][1]:.1f}, {t['offset'][2]:.1f})")

    def _set_slider_value(self, slider_widget, val):
        s = slider_widget.findChild(QtWidgets.QSlider)
        if s is None:
            return
        s.blockSignals(True)
        vmin = getattr(slider_widget, '_slider_vmin', -15)
        vmax = getattr(slider_widget, '_slider_vmax', 15)
        steps = getattr(slider_widget, '_slider_steps', 1000)
        frac = (val - vmin) / (vmax - vmin) if (vmax - vmin) != 0 else 0.0
        s.setValue(int(frac * steps))
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
        vtk_matrix = vtkMatrix4x4()
        for i in range(4):
            for j in range(4):
                vtk_matrix.SetElement(i, j, float(H[i, j]))

        transform = vtkTransform()
        transform.SetMatrix(vtk_matrix)
        vtk_actor.SetUserTransform(transform)
        self.plotter.render()

    def _schedule_transform_log(self, clean):
        self._pending_transform_log = clean
        self._transform_log_timer.start(1500)

    def _push_undo(self, name):
        """Snapshot current transform state before slider change for Ctrl+Z undo."""
        t = self._obj_transforms.get(name)
        if t is None:
            return
        snapshot = {
            "offset": list(t.get("offset", [0, 0, 0])),
            "scale": t.get("scale", 1.0),
            "yaw": t.get("yaw", 0.0),
            "pitch": t.get("pitch", 0.0),
            "roll": t.get("roll", 0.0),
        }
        self._undo_stack.append((name, snapshot))
        if len(self._undo_stack) > 50:  # cap at 50 entries
            self._undo_stack.pop(0)

    def _undo_last_transform(self):
        """Restore the last undo snapshot (Ctrl+Z handler)."""
        if not self._undo_stack:
            self.statusBar().showMessage("没有可撤销的操作", 2000)
            return
        name, snap = self._undo_stack.pop()
        t = self._obj_transforms.get(name)
        if t is None:
            return
        t["offset"] = snap["offset"]
        t["scale"] = snap["scale"]
        t["yaw"] = snap["yaw"]
        t["pitch"] = snap["pitch"]
        t["roll"] = snap["roll"]
        self._apply_obj_transform_to_actor(name)
        # Update UI sliders to reflect restored state
        self._set_slider_value(self._slider_obj_x, snap["offset"][0])
        self._set_slider_value(self._slider_obj_y, snap["offset"][1])
        self._set_slider_value(self._slider_obj_z, snap["offset"][2])
        self._set_slider_value(self._slider_obj_s, snap["scale"])
        self._set_slider_value(self._slider_obj_yaw, snap["yaw"])
        self._set_slider_value(self._slider_obj_pitch, snap["pitch"])
        self._set_slider_value(self._slider_obj_roll, snap["roll"])
        self._log_action(f"用户撤销了【{name}】的变换")
        self.statusBar().showMessage(f"已撤销【{name}】的变换", 2000)

    def _flush_transform_log(self):
        if self._pending_transform_log is None:
            return
        name = self._pending_transform_log
        self._pending_transform_log = None
        t = self._obj_transforms.get(name)
        if t is None:
            return
        offset = t["offset"]
        self._log_action(
            f"用户调整了【{name}】位置 ({offset[0]:.2f}, {offset[1]:.2f}, {offset[2]:.2f})"
        )

    def _on_obj_pos_x(self, val):
        name = self._obj_combo.currentText()
        clean = name.replace("[自定义] ", "")
        if clean not in self._obj_transforms:
            return
        self._push_undo(clean)
        self._obj_transforms[clean]["offset"][0] = val
        self._apply_obj_transform_to_actor(clean)
        self._schedule_transform_log(clean)

    def _on_obj_pos_y(self, val):
        name = self._obj_combo.currentText()
        clean = name.replace("[自定义] ", "")
        if clean not in self._obj_transforms:
            return
        self._push_undo(clean)
        self._obj_transforms[clean]["offset"][1] = val
        self._apply_obj_transform_to_actor(clean)
        self._schedule_transform_log(clean)

    def _on_obj_pos_z(self, val):
        name = self._obj_combo.currentText()
        clean = name.replace("[自定义] ", "")
        if clean not in self._obj_transforms:
            return
        self._push_undo(clean)
        self._obj_transforms[clean]["offset"][2] = val
        self._apply_obj_transform_to_actor(clean)
        self._schedule_transform_log(clean)

    def _on_obj_scale(self, val):
        name = self._obj_combo.currentText()
        clean = name.replace("[自定义] ", "")
        if clean not in self._obj_transforms:
            return
        self._push_undo(clean)
        self._obj_transforms[clean]["scale"] = val
        self._apply_obj_transform_to_actor(clean)

    def _on_obj_yaw(self, val):
        name = self._obj_combo.currentText()
        clean = name.replace("[自定义] ", "")
        if clean not in self._obj_transforms:
            return
        self._push_undo(clean)
        self._obj_transforms[clean]["yaw"] = val
        self._apply_obj_transform_to_actor(clean)

    def _on_obj_pitch(self, val):
        name = self._obj_combo.currentText()
        clean = name.replace("[自定义] ", "")
        if clean not in self._obj_transforms:
            return
        self._push_undo(clean)
        self._obj_transforms[clean]["pitch"] = val
        self._apply_obj_transform_to_actor(clean)

    def _on_obj_roll(self, val):
        name = self._obj_combo.currentText()
        clean = name.replace("[自定义] ", "")
        if clean not in self._obj_transforms:
            return
        self._push_undo(clean)
        self._obj_transforms[clean]["roll"] = val
        self._apply_obj_transform_to_actor(clean)


    # ═══════════════════════════════════════════════════════════════
    #  3D Interaction callbacks (Qt-level mouse events)
    # ═══════════════════════════════════════════════════════════════

    def _on_3d_click(self, x, y, world_pos, vtk_actor):
        """Route a 3D viewport click based on the current interaction mode."""
        mode = self._current_mode

        if mode == InteractionMode.NORMAL:
            if vtk_actor is not None:
                name = self._get_name_from_vtk_actor(vtk_actor)
                if name:
                    self._select_object(name)
                    return
            self._clear_highlight()

        elif mode in (InteractionMode.MEASURE_DISTANCE, InteractionMode.MEASURE_ANGLE):
            if np.linalg.norm(world_pos) < 1e-6:
                self.statusBar().showMessage("⚠ 点击位置未命中地形表面，请重试", 2000)
                return
            self.meas_tool.add_point(world_pos)

        elif mode == InteractionMode.WAYPOINT:
            if np.linalg.norm(world_pos) < 1e-6:
                self.statusBar().showMessage("⚠ 点击位置未命中地形表面，请重试", 2000)
                return
            self._add_waypoint(world_pos)

    def _on_3d_move(self, x, y):
        """Update coordinate info on mouse move."""
        picker = vtkWorldPointPicker()
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

        # Waypoint placement mode
        if mode == InteractionMode.WAYPOINT:
            self.setCursor(QtGui.QCursor(Qt.CrossCursor))
            self.statusBar().showMessage(
                "路径点模式 — 点击 3D 场景放置路径点  |  再次点击\"添加3D路径点\"退出", 0
            )
        # Measurement mode internal
        elif mode in (InteractionMode.MEASURE_DISTANCE, InteractionMode.MEASURE_ANGLE):
            self.meas_tool.set_mode(
                "distance" if mode == InteractionMode.MEASURE_DISTANCE else "angle"
            )
            self.setCursor(QtGui.QCursor(Qt.CrossCursor))
            self.statusBar().showMessage(
                "测量模式 — 点击 3D 场景放置测量点", 5000
            )
        else:
            self.setCursor(QtGui.QCursor(Qt.ArrowCursor))
            self.statusBar().showMessage(
                "就绪  |  左键旋转 · 滚轮缩放 · 中键平移", 5000
            )


    # ═══════════════════════════════════════════════════════════════
    #  View & coordinate system
    # ═══════════════════════════════════════════════════════════════

    def _set_view(self, direction):
        p = self.plotter
        extent = self._compute_terrain_extent()
        xy_half, z_half = extent
        dist = max(xy_half * 2.5, 25.0)
        fp = (0, 0, z_half * 0.5)
        views = {
            "top":    ((0, 0, dist), (0, 0, 0), (0, 1, 0)),
            "bottom": ((0, 0, -dist), (0, 0, 0), (0, 1, 0)),
            "front":  ((dist, 0, 0), (0, 0, 0), (0, 0, 1)),
            "side":   ((0, -dist, 0), (0, 0, 0), (0, 0, 1)),
        }
        if direction in views:
            p.camera_position = views[direction]
            p.camera.focal_point = fp
            p.render()
        view_names = {"top": "俯视图", "bottom": "仰视图", "front": "正视图", "side": "侧视图"}
        self._log_action(f"用户切换了相机视角，当前为：{view_names.get(direction, direction)}")
        

    def _reset_camera(self):
        p = self.plotter
        extent = self._compute_terrain_extent()
        xy_half, z_half = extent
        dist = max(xy_half * 2.5, 25.0)
        mid_z = z_half * 0.5
        p.camera_position = [(dist * 0.6, -dist * 0.5, dist * 0.4),
                             (0, 0, mid_z), (0, 0, 1)]
        p.camera.focal_point = (0, 0, mid_z)
        p.render()
        self._log_action("用户复位了相机")
        

    def _focus_camera_on_aircraft(self, pos, yaw_deg=0.0):
        """Position camera in tail-chase view behind and above the aircraft."""
        p = self.plotter
        extent = self._compute_terrain_extent()
        xy_half, z_half = extent
        dist = max(xy_half * 0.8, 50.0)
        # Behind the aircraft based on heading
        yaw_rad = math.radians(yaw_deg)
        behind_x = -math.cos(yaw_rad) * dist
        behind_y = -math.sin(yaw_rad) * dist
        above_z = dist * 0.4
        cam_pos = (pos[0] + behind_x, pos[1] + behind_y, pos[2] + above_z)
        p.camera_position = [cam_pos, (pos[0], pos[1], pos[2]), (0, 0, 1)]
        p.render()

    def _reset_all(self):
        reply = QtWidgets.QMessageBox.question(
            self, "确认全局复位",
            "确定要将所有对象恢复到初始位置和姿态吗？\n\n"
            "所有手动调整的位移、旋转、缩放将丢失。\n"
            "此操作不可撤销。",
            QtWidgets.QMessageBox.Yes | QtWidgets.QMessageBox.No)
        if reply != QtWidgets.QMessageBox.Yes:
            return
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
        self._log_action("用户点击了全局复位")

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

        # For DEM scenes, also save X, Y coordinate grids for proper reconstruction
        if "X" in extra and "Y" in extra:
            terrain_data["X"] = extra["X"].tolist()
            terrain_data["Y"] = extra["Y"].tolist()

        terrain_path = os.path.join(terrain_dir, f"{name}.json")
        with open(terrain_path, "w") as f:
            json.dump(terrain_data, f, indent=2, ensure_ascii=False)

        self._log_action(f"用户保存了数据：{name}")
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

        # Create directories if not exist so user can navigate there
        os.makedirs(aircraft_dir, exist_ok=True)
        os.makedirs(terrain_dir, exist_ok=True)

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

        if "aircraft" in os.path.basename(path_dir).lower():
            self._load_aircraft_data(path, base_name)
        elif "terrain" in os.path.basename(path_dir).lower():
            self._load_terrain_data(path, base_name)
        elif path_dir == abs_aircraft_dir:
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
        self._log_action(f"用户加载了飞机数据：{base_name}")
        self.statusBar().showMessage(f"飞行器数据已载入: {base_name}", 5000)

    def _load_terrain_data(self, path, base_name):
        """Restore terrain mesh + all non-aircraft objects + config + camera."""
        try:
            with open(path) as f:
                terrain_data = json.load(f)
        except Exception as e:
            QtWidgets.QMessageBox.warning(self, "错误", f"加载地形数据失败: {e}")
            return

        # Check if this is DEM terrain data (has X, Y grids)
        has_dem_grid = "X" in terrain_data and "Y" in terrain_data

        # Restore terrain mesh elevation
        terrain_info = self.scene_objects.get("terrain")
        if terrain_info is not None:
            extra = terrain_info["extra"]
            Z_2d = np.array(terrain_data["original_z"])

            if has_dem_grid and extra is not None:
                # DEM scene: reconstruct StructuredGrid from saved X, Y, Z
                X_2d = np.array(terrain_data["X"])
                Y_2d = np.array(terrain_data["Y"])
                rows, cols = Z_2d.shape
                points = np.column_stack((X_2d.ravel(), Y_2d.ravel(),
                                          Z_2d.ravel()))
                new_grid = pv.StructuredGrid()
                new_grid.points = points.astype(np.float32)
                new_grid.dimensions = (cols, rows, 1)
                new_grid["elevation"] = Z_2d.flatten(order="F")

                self.scene_objects["terrain"]["mesh"] = new_grid
                extra["original_z"] = Z_2d
                extra["X"] = X_2d
                extra["Y"] = Y_2d
                self._rebuild_actor("terrain")
            else:
                # Normal scene: just restore Z values on existing mesh
                mesh = terrain_info["mesh"]
                extra["original_z"] = Z_2d
                pts = mesh.points
                pts[:, 2] = Z_2d.flatten(order="F")
                mesh.points = pts
                mesh["elevation"] = Z_2d.flatten(order="F")
                self._rebuild_actor("terrain")

            # In DEM mode, skip sand/grass/earth layers (DEM scenes don't use them)
            if not self._is_dem_scene():
                self._rebuild_terrain_layers()

        # Restore transforms for ALL non-aircraft objects
        objects_data = terrain_data.get("objects", {})
        for obj_name, t in objects_data.items():
            if obj_name in self.scene_objects or obj_name in self.custom_objects:
                self._obj_transforms[obj_name] = t
                self._apply_obj_transform_to_actor(obj_name)

        # Restore config
        self.config.update(terrain_data.get("config", {}))

        # Restore camera
        if "camera" in terrain_data:
            self.plotter.camera_position = tuple(terrain_data["camera"])

        self._refresh_obj_combo()
        self.plotter.render()
        self._log_action(f"用户加载了地形数据：{base_name}")
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
    #  DEM model import
    # ═══════════════════════════════════════════════════════════════

    def _import_dem_model(self):
        """Import a DEM .img file and replace the scene with DEM terrain.

        Replaces terrain/river/vegetation/bird/tree with the DEM surface.
        Aircraft are repositioned at Z=1000m with 500x scale so they remain
        visible in the large-coordinate scene.
        """
        if not HAS_RASTERIO:
            import sys as _sys
            QtWidgets.QMessageBox.warning(
                self, "缺少依赖",
                "导入 DEM 模型需要 rasterio 库。\n"
                f"\n"
                f"当前 Python: {_sys.executable}\n"
                f"({_sys.version})\n"
                f"\n"
                f"请安装:\n"
                f"  {_sys.executable} -m pip install rasterio"
            )
            return

        path, _ = QtWidgets.QFileDialog.getOpenFileName(
            self, "导入 DEM 模型", "",
            "DEM 文件 (*.img *.tif *.tiff);;HFA/IMG (*.img);;GeoTIFF (*.tif *.tiff);;所有文件 (*)"
        )
        if not path:
            return

        # ── Load DEM data ────────────────────────────────
        try:
            dem = load_dem(path, step=2)
        except Exception as e:
            QtWidgets.QMessageBox.warning(self, "DEM 导入失败", f"无法读取 DEM 文件:\n{e}")
            return

        # ── Ask for vertical exaggeration ────────────────
        vert_exag, ok = QtWidgets.QInputDialog.getDouble(
            self, "DEM 垂直夸张",
            "垂直夸张系数 (Z exaggeration):\n"
            ">1 拉高山脉, <1 压低地形",
            value=2.0, min=0.1, max=20.0, decimals=1,
        )
        if not ok:
            return

        reply = QtWidgets.QMessageBox.question(
            self, "导入 DEM",
            f"DEM 数据加载完成:\n"
            f"  网格大小: {dem['rows']} × {dem['cols']} = {dem['rows']*dem['cols']:,} 点\n"
            f"  X 范围: {dem['x'][0]:.0f} ~ {dem['x'][-1]:.0f} m\n"
            f"  Y 范围: {dem['y'][0]:.0f} ~ {dem['y'][-1]:.0f} m\n"
            f"  高程范围: {dem['z'].min():.0f} ~ {dem['z'].max():.0f} m\n"
            f"  垂直夸张: {vert_exag:.1f}×\n"
            f"  坐标基准: {dem.get('crs', 'N/A')}\n\n"
            f"这将替换当前场景中的地形、河流、植被、鸟和树。\n"
            f"飞机将被置于 Z=7000m 并放大 {AIRCRAFT_DEFAULT_SCALE:,}×。\n\n"
            f"确认导入?",
            QtWidgets.QMessageBox.Yes | QtWidgets.QMessageBox.No
        )
        if reply != QtWidgets.QMessageBox.Yes:
            return

        # ── Persist user choices ──────────────────────────
        self.config["elevation_scale"] = vert_exag
        crs = dem.get("crs")
        if crs:
            self.config["dem_crs"] = crs

        # ── Build DEM scene objects ──────────────────────
        dem_scene = build_dem_scene(
            dem,
            vert_exag=vert_exag,
            aircraft_scale=AIRCRAFT_DEFAULT_SCALE,
            aircraft_z=7000.0,
        )
        self._dem_scene_active = True

        terrain_extent = max(abs(dem['x'][0]), abs(dem['x'][-1]),
                             abs(dem['y'][0]), abs(dem['y'][-1]))

        # ── Completely reset scene ───────────────────────
        self._flowing = False
        self._clear_waypoints()
        for name in list(self.plotter_actors.keys()):
            self._remove_actor(name)
        self.scene_objects.clear()
        self._obj_transforms.clear()
        self._terrain_chks.clear()
        self._terrain_opacity_sliders.clear()
        self._terrain_opacity_setters.clear()

        # ── Add only DEM objects (terrain, aircraft, aircraft2) ──
        for name, obj in dem_scene.items():
            self.scene_objects[name] = obj
            if obj["visible"]:
                self._add_actor(name, obj)

        # ── Update slider ranges for DEM-scale coords ────
        slider_margin = terrain_extent * 1.2
        self._update_slider_range(self._slider_obj_x, -slider_margin, slider_margin)
        self._update_slider_range(self._slider_obj_y, -slider_margin, slider_margin)
        self._update_slider_range(self._slider_obj_z, -500, 10000)

        # ── Reset camera for the new scale ───────────────
        cam_dist = terrain_extent * 2.5
        self.plotter.camera_position = [
            (cam_dist * 0.6, -cam_dist * 0.5, cam_dist * 0.4),
            (0, 0, 500),
            (0, 0, 1),
        ]
        self.plotter.camera.focal_point = (0, 0, 500)
        self.plotter.render()

        # ── Refresh UI ───────────────────────────────────
        self._refresh_ui()
        x_span = abs(dem['x'][-1] - dem['x'][0])
        y_span = abs(dem['y'][-1] - dem['y'][0])
        self._log_action(f"用户加载了新的 DEM 地形，尺寸：X={x_span:,.0f}m, Y={y_span:,.0f}m")
        self.statusBar().showMessage(
            f"DEM 已导入: {os.path.basename(path)}  "
            f"({dem['rows']}×{dem['cols']}, "
            f"Z {dem['z'].min():.0f}~{dem['z'].max():.0f}m)", 8000
        )
        self._on_terrain_changed(path)

    # ═══════════════════════════════════════════════════════════════
    #  ASC grid import/export
    # ═══════════════════════════════════════════════════════════════

    def _import_asc_grid(self):
        path, _ = QtWidgets.QFileDialog.getOpenFileName(
            self, "导入 ASC 格网数据", "",
            "ASC 格网数据 (*.asc);;所有文件 (*)"
        )
        if not path:
            return

        try:
            dem = self._parse_asc(path)
        except Exception as e:
            QtWidgets.QMessageBox.warning(self, "导入失败", f"无法解析 ASC 文件:\n{e}")
            return

        grid = dem_to_mesh(dem, vert_exag=1.0)

        # ── Clear scene ─────────────────────────────────
        self._flowing = False
        self._clear_waypoints()
        for name in list(self.plotter_actors.keys()):
            self._remove_actor(name)
        self.scene_objects.clear()
        self._obj_transforms.clear()
        self._terrain_chks.clear()
        self._terrain_opacity_sliders.clear()
        self._terrain_opacity_setters.clear()

        terrain_obj = {
            "mesh": grid,
            "type": "mesh",
            "visible": True,
            "params": {"color": "#f2e2a8", "smooth_shading": True, "opacity": 1.0},
            "extra": None,
            "name": "terrain",
        }
        self.scene_objects["terrain"] = terrain_obj
        self._add_actor("terrain", terrain_obj)

        ac = _build_airplane_mesh().copy()
        ac.scale(1.5, inplace=True)
        ac.translate((-6, -2, 8), inplace=True)
        self.scene_objects["aircraft"] = {
            "mesh": ac, "type": "mesh", "visible": True,
            "params": {"color": "#c4c4c4", "smooth_shading": True,
                       "ambient": 0.35, "diffuse": 0.75,
                       "specular": 0.6, "specular_power": 40},
            "extra": None, "name": "aircraft",
        }
        self._add_actor("aircraft", self.scene_objects["aircraft"])

        ac2 = _build_airplane_mesh().copy()
        ac2.scale(1.5, inplace=True)
        ac2.translate((-6, 3, 7.5), inplace=True)
        self.scene_objects["aircraft2"] = {
            "mesh": ac2, "type": "mesh", "visible": True,
            "params": {"color": "#cc3333", "smooth_shading": True,
                       "ambient": 0.35, "diffuse": 0.75,
                       "specular": 0.6, "specular_power": 40},
            "extra": None, "name": "aircraft2",
        }
        self._add_actor("aircraft2", self.scene_objects["aircraft2"])

        self.plotter.reset_camera()
        self.plotter.render()
        self._refresh_ui()
        self._log_action("用户导入了 ASC 格网数据")
        self.statusBar().showMessage(
            f"ASC 格网已导入: {os.path.basename(path)}  "
            f"({dem['rows']}×{dem['cols']}, "
            f"Z {dem['z'].min():.0f}~{dem['z'].max():.0f})", 8000
        )
        

    def _parse_asc(self, path):
        with open(path) as f:
            lines = f.readlines()

        # ── Parse 6-line header ─────────────────────────
        header = {}
        idx = 0
        while idx < len(lines) and len(header) < 6:
            parts = lines[idx].strip().split()
            if len(parts) >= 2:
                key = parts[0].lower()
                try:
                    header[key] = float(parts[1])
                except ValueError:
                    pass
            idx += 1

        ncols = int(header.get("ncols", 0))
        nrows = int(header.get("nrows", 0))
        xll = header.get("xllcorner", 0.0)
        yll = header.get("yllcorner", 0.0)
        cellsize = header.get("cellsize", 1.0)
        nodata = header.get("nodata_value", None)

        if ncols <= 0 or nrows <= 0:
            raise ValueError("无效的 ASC 头信息: ncols/nrows 缺失或为零")

        # ── Read grid data ──────────────────────────────
        data_lines = lines[idx:]
        values = []
        for line in data_lines:
            vals = line.strip().split()
            for v in vals:
                try:
                    values.append(float(v))
                except ValueError:
                    pass

        if len(values) != ncols * nrows:
            raise ValueError(
                f"数据点数量不匹配: 期望 {ncols*nrows}, 实际 {len(values)}"
            )

        z = np.array(values, dtype=np.float32).reshape(nrows, ncols)

        # ── Replace NODATA with 0 ───────────────────────
        if nodata is not None:
            z[z == nodata] = 0.0

        # ── Build XY grid (centred around origin) ───────
        x_raw = xll + cellsize / 2 + np.arange(ncols, dtype=np.float32) * cellsize
        y_raw = yll + cellsize / 2 + np.arange(nrows, dtype=np.float32) * cellsize
        y_raw = y_raw[::-1]

        x_centre = (x_raw[-1] + x_raw[0]) * 0.5
        y_centre = (y_raw[-1] + y_raw[0]) * 0.5
        x = x_raw - x_centre
        y = y_raw - y_centre

        xx, yy = np.meshgrid(x, y)

        return {
            "z": z,
            "x": x,
            "y": y,
            "xx": xx,
            "yy": yy,
            "rows": nrows,
            "cols": ncols,
            "pixel_size": cellsize,
            "crs": None,
            "full_resolution": (nrows, ncols),
            "source": path,
        }

    def _export_asc_grid(self):
        terrain_obj = self.scene_objects.get("terrain")
        if terrain_obj is None:
            QtWidgets.QMessageBox.information(self, "提示", "当前场景没有 DEM 地形数据可导出")
            return
        extra = terrain_obj.get("extra")
        if extra is None:
            QtWidgets.QMessageBox.information(self, "提示", "场景中缺少地形栅格数据，无法导出 ASC")
            return

        original_z = extra.get("original_z")
        X = extra.get("X")
        Y = extra.get("Y")
        if original_z is None or X is None or Y is None:
            QtWidgets.QMessageBox.information(self, "提示", "地形数据不完整，无法导出 ASC")
            return

        path, _ = QtWidgets.QFileDialog.getSaveFileName(
            self, "导出 ASC 格网数据", "terrain.asc",
            "ASC 格网数据 (*.asc);;所有文件 (*)"
        )
        if not path:
            return

        try:
            nrows, ncols = original_z.shape
            cellsize = abs(X[0, 1] - X[0, 0]) if ncols > 1 else 1.0
            x_centers = X[0, :]
            y_centers = Y[:, 0]
            xll = x_centers[0] - cellsize / 2
            yll = y_centers[-1] - cellsize / 2

            with open(path, "w") as f:
                f.write(f"ncols         {ncols}\n")
                f.write(f"nrows         {nrows}\n")
                f.write(f"xllcorner     {xll:.6f}\n")
                f.write(f"yllcorner     {yll:.6f}\n")
                f.write(f"cellsize      {cellsize:.6f}\n")
                f.write("NODATA_value  -9999\n")
                for r in range(nrows):
                    row_vals = " ".join(
                        f"{original_z[r, c]:.1f}" for c in range(ncols)
                    )
                    f.write(row_vals + "\n")

            self.statusBar().showMessage(f"ASC 已导出: {path}", 3000)
        except Exception as e:
            QtWidgets.QMessageBox.warning(self, "导出失败", f"导出 ASC 文件时出错:\n{e}")

    # ═══════════════════════════════════════════════════════════════
    #  Path planning
    # ═══════════════════════════════════════════════════════════════

    def _add_waypoint(self, world_pos):
        """Add a 3D waypoint → show dialog to select target aircraft(s)."""
        names = [n for n in self.scene_objects if "aircraft" in n.lower()]
        if not names:
            self.statusBar().showMessage("⚠ 当前场景无飞机", 3000)
            return
        # Auto-assign colours for new aircraft
        for i, n in enumerate(names):
            if n not in self._aircraft_colors:
                self._aircraft_colors[n] = self._color_palette[i % len(self._color_palette)]

        dlg = QtWidgets.QDialog(self)
        dlg.setWindowTitle("选择目标飞机")
        dlg.setWindowFlags(dlg.windowFlags() | QtCore.Qt.Window)
        dlg.resize(300, 220)
        lo = QtWidgets.QVBoxLayout(dlg)
        lo.addWidget(QtWidgets.QLabel("将此路径点添加到:"))
        checks = {}
        for n in names:
            chk = QtWidgets.QCheckBox(n)
            chk.setChecked(True)
            lo.addWidget(chk)
            checks[n] = chk
        btns = QtWidgets.QDialogButtonBox(
            QtWidgets.QDialogButtonBox.Ok | QtWidgets.QDialogButtonBox.Cancel)
        btns.accepted.connect(dlg.accept)
        btns.rejected.connect(dlg.reject)
        lo.addWidget(btns)
        if dlg.exec_() != QtWidgets.QDialog.Accepted:
            return
        wp = np.asarray(world_pos)
        added = []
        for n in names:
            if checks[n].isChecked():
                self._aircraft_waypoints.setdefault(n, []).append(wp.copy())
                added.append(n)
        if not added:
            return
        self._rebuild_waypoint_actors()
        self._refresh_waypoint_tree()
        names_str = ", ".join(added)
        self._log_action(f"添加路径点 ({wp[0]:.2f}, {wp[1]:.2f}, {wp[2]:.2f}) → {names_str}")
        self.statusBar().showMessage(f"路径点已添加到: {names_str}", 3000)

    def _add_waypoints_batch(self, coords_list):
        """Add multiple waypoints at once — ask aircraft selection once."""
        names = [n for n in self.scene_objects if "aircraft" in n.lower()]
        if not names:
            self.statusBar().showMessage("⚠ 当前场景无飞机", 3000)
            return
        for i, n in enumerate(names):
            if n not in self._aircraft_colors:
                self._aircraft_colors[n] = self._color_palette[i % len(self._color_palette)]
        dlg = QtWidgets.QDialog(self)
        dlg.setWindowTitle("选择目标飞机")
        dlg.setWindowFlags(dlg.windowFlags() | QtCore.Qt.Window)
        dlg.resize(300, 220)
        lo = QtWidgets.QVBoxLayout(dlg)
        lo.addWidget(QtWidgets.QLabel(f"将 {len(coords_list)} 个路径点添加到:"))
        checks = {}
        for n in names:
            chk = QtWidgets.QCheckBox(n)
            chk.setChecked(True)
            lo.addWidget(chk)
            checks[n] = chk
        btns = QtWidgets.QDialogButtonBox(
            QtWidgets.QDialogButtonBox.Ok | QtWidgets.QDialogButtonBox.Cancel)
        btns.accepted.connect(dlg.accept)
        btns.rejected.connect(dlg.reject)
        lo.addWidget(btns)
        if dlg.exec_() != QtWidgets.QDialog.Accepted:
            return
        added = []
        for n in names:
            if checks[n].isChecked():
                for xyz in coords_list:
                    self._aircraft_waypoints.setdefault(n, []).append(np.array(xyz))
                added.append(n)
        if not added:
            return
        self._rebuild_waypoint_actors()
        self._refresh_waypoint_tree()
        names_str = ", ".join(added)
        self._log_action(f"批量添加 {len(coords_list)} 个路径点 → {names_str}")
        self.statusBar().showMessage(f"{len(coords_list)} 个路径点已添加到: {names_str}", 3000)

    def _refresh_waypoint_tree(self):
        flight_root = self._tree_items.get(SceneNodeType.FLIGHT_PLATFORM)
        if flight_root is None:
            return
        for i in range(flight_root.childCount()):
            ac = flight_root.child(i)
            ds = ac.data(0, Qt.UserRole)
            if ds and ds.scene_obj_name:
                ac_name = ds.scene_obj_name
                while ac.childCount() > 0:
                    ac.removeChild(ac.child(0))
                self._lazy_load_waypoints(ac, ac_name)
                if ac.childCount() > 0:
                    ac.setExpanded(True)

    def _toggle_wp_mode(self):
        """Toggle between WAYPOINT (click-to-place) and NORMAL mode."""
        if self._current_mode == InteractionMode.WAYPOINT:
            self._set_interaction_mode(InteractionMode.NORMAL)
        else:
            self._set_interaction_mode(InteractionMode.WAYPOINT)

    def _snap_to_terrain(self, world_pos):
        """Return (x, y, terrain_z) by sampling the terrain mesh Z at (x, y)."""
        terrain_info = self.scene_objects.get("terrain")
        if terrain_info is None:
            return world_pos
        try:
            mesh = terrain_info["mesh"]
            closest = mesh.find_closest_point(tuple(world_pos))
            if closest is not None:
                return np.array([world_pos[0], world_pos[1], closest[2]], dtype=float)
        except Exception:
            pass
        return world_pos

    def _compute_terrain_extent(self):
        """Return ``(xy_half, z_half)`` of the terrain mesh, or ``(10, 20)``."""
        info = self.scene_objects.get("terrain")
        if info is None:
            return (10.0, 20.0)
        try:
            b = info["mesh"].bounds  # (xmin, xmax, ymin, ymax, zmin, zmax)
            xy_half = max(abs(b[0]), abs(b[1]), abs(b[2]), abs(b[3]))
            z_half = max(abs(b[4]), abs(b[5]))
            if xy_half < 1.0:
                return (10.0, 20.0)
            return (xy_half, z_half)
        except Exception:
            return (10.0, 20.0)

    def _is_dem_scene(self):
        """Check if current scene is a DEM-imported scene (uses boolean flag)."""
        return self._dem_scene_active

    def _open_precise_wp_dialog(self):
        if not _HAS_MPL:
            QtWidgets.QMessageBox.warning(
                self, "提示",
                "精准添加路径点需要 matplotlib 支持。\n请安装: pip install matplotlib"
            )
            return
        extent = self._compute_terrain_extent()
        terrain_mesh = self.scene_objects.get("terrain", {}).get("mesh")
        self._log_action("用户打开了精准添加路径点对话框")
        dlg = WaypointPreciseDialog(self, terrain_extent=extent, terrain_mesh=terrain_mesh)
        if dlg.exec_() == QtWidgets.QDialog.Accepted:
            coords_list = dlg.get_coords()
            if coords_list:
                # Check if this is for a formation
                fname = getattr(self, '_pending_formation_path', None)
                if fname and fname in self._formations:
                    self._pending_formation_path = None
                    form = self._formations[fname]
                    form["waypoints"] = [np.array(xyz) for xyz in coords_list]
                    for n in form["members"]:
                        self._aircraft_waypoints[n] = [np.array(xyz) for xyz in coords_list]
                    self._rebuild_waypoint_actors()
                    self._refresh_waypoint_tree()
                    self._log_action(f"{fname} 自定义路径点: {len(coords_list)} 个")
                    self.statusBar().showMessage(f"{fname} 自定义路径点已设置", 3000)
                else:
                    self._add_waypoints_batch(coords_list)

    def _clear_waypoints(self):
        if not self.waypoints and not self._wp_actors and not any(self._aircraft_waypoints.values()):
            return
        count = len(self.waypoints)
        self.waypoints.clear()
        self._aircraft_waypoints.clear()  # clear per-aircraft waypoints too
        for a in self._wp_actors:
            self.plotter.remove_actor(a)
        self._wp_actors.clear()
        if self._path_actor is not None:
            self.plotter.remove_actor(self._path_actor)
            self._path_actor = None
        self.plotter.render()
        self._log_action(f"用户清除了 {count} 个飞行路径点")

    def _confirm_clear_waypoints(self):
        """Show confirmation dialog before clearing all waypoints."""
        total = len(self.waypoints) + sum(len(v) for v in self._aircraft_waypoints.values())
        if total == 0:
            self.statusBar().showMessage("⚠ 没有可清除的路径点", 3000)
            return
        reply = QtWidgets.QMessageBox.question(
            self, "确认清除",
            f"确定要清除所有路径点吗？\n\n"
            f"共 {total} 个路径点将被永久删除。\n"
            f"此操作不可撤销。",
            QtWidgets.QMessageBox.Yes | QtWidgets.QMessageBox.No)
        if reply == QtWidgets.QMessageBox.Yes:
            self._clear_waypoints()

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

    # ── Formation system (ID-27 rebuilt) ──────────────────────────

    def _start_formation_dialog(self):
        names = [n for n in self.scene_objects if "aircraft" in n.lower()]
        if len(names) < 1:
            self.statusBar().showMessage("⚠ 当前场景无飞机", 3000)
            return
        dlg = QtWidgets.QDialog(self)
        dlg.setWindowTitle("开始编队 — 选择编队成员")
        dlg.setWindowFlags(dlg.windowFlags() | QtCore.Qt.Window)
        dlg.resize(300, 220)
        lo = QtWidgets.QVBoxLayout(dlg)
        lo.addWidget(QtWidgets.QLabel("勾选编队成员（第一个为领队）:"))
        checks = {}
        for n in names:
            chk = QtWidgets.QCheckBox(n)
            lo.addWidget(chk)
            checks[n] = chk
        lo.addStretch()
        btns = QtWidgets.QDialogButtonBox(
            QtWidgets.QDialogButtonBox.Ok | QtWidgets.QDialogButtonBox.Cancel)
        btns.accepted.connect(dlg.accept)
        btns.rejected.connect(dlg.reject)
        lo.addWidget(btns)
        if dlg.exec_() != QtWidgets.QDialog.Accepted:
            return
        members = [n for n in names if checks[n].isChecked()]
        if len(members) < 1:
            return
        self._create_formation(members)

    def _create_formation(self, members):
        # Save waypoint snapshot for cancel
        self._formation_saved_wp = {
            n: [np.array(w) for w in self._aircraft_waypoints.get(n, [])]
            for n in self.scene_objects if "aircraft" in n.lower()
        }
        # Create formation name
        idx = len(self._formations) + 1
        fname = f"编队{idx}"
        label = f"{fname} ({'+'.join(members)})"

        # Add tree node under flight platform
        flight_root = self._tree_items.get(SceneNodeType.FLIGHT_PLATFORM)
        if flight_root is None:
            return
        tree_node = SceneTreeFactory._create_item(
            flight_root,
            NodeData(
                node_type=SceneNodeType.GLOBAL_TOOL,
                label=label, tooltip=f"编队成员: {', '.join(members)}",
                icon_name=SceneNodeType.GLOBAL_TOOL,
                is_deletable=True,
            ),
        )
        # Add sub-items: select waypoints + start flight
        SceneTreeFactory._create_item(tree_node, NodeData(
            node_type=SceneNodeType.PATH_ACTION, label="为编队选择路径点（双击）",
            slot_name="_select_formation_path", parent_node_type=SceneNodeType.GLOBAL_TOOL,
        ))
        SceneTreeFactory._create_item(tree_node, NodeData(
            node_type=SceneNodeType.PATH_ACTION, label="编队开始飞行（双击）",
            slot_name="_formation_start_flight", parent_node_type=SceneNodeType.GLOBAL_TOOL,
        ))
        tree_node.setExpanded(True)

        self._formations[fname] = {
            "members": members, "waypoints": [], "leader": members[0],
            "tree_node": tree_node, "label": label,
        }
        self._btn_formation.setText("编队中...")
        self._btn_formation.setEnabled(False)
        self._btn_cancel_formation.setVisible(True)
        self._log_action(f"用户创建了{fname}: {', '.join(members)}")
        self.statusBar().showMessage(f"{fname} 已创建: {', '.join(members)}", 4000)

    def _select_formation_path(self):
        if not self._formations:
            return
        fname = list(self._formations.keys())[-1]  # last created formation
        form = self._formations[fname]

        names = [n for n in self.scene_objects if "aircraft" in n.lower()]
        wp_options = []
        for n in names:
            wps = self._aircraft_waypoints.get(n, [])
            if wps:
                wp_options.append((n, f"{n} 的路径点 ({len(wps)} 个)"))
        dlg = QtWidgets.QDialog(self)
        dlg.setWindowTitle(f"{fname} — 选择路径点")
        dlg.setWindowFlags(dlg.windowFlags() | QtCore.Qt.Window)
        dlg.resize(360, 280)
        lo = QtWidgets.QVBoxLayout(dlg)
        lo.addWidget(QtWidgets.QLabel(f"为 {fname} ({', '.join(form['members'])}) 选择路径点:"))
        sel = QtWidgets.QListWidget()
        for n, label in wp_options:
            sel.addItem(label)
        if sel.count() > 0:
            sel.setCurrentRow(0)
        lo.addWidget(sel)
        lo.addWidget(QtWidgets.QLabel("或:"))
        btn_custom = QtWidgets.QPushButton("自定义编队路径点 (XY图)")
        lo.addWidget(btn_custom)
        btns = QtWidgets.QDialogButtonBox(QtWidgets.QDialogButtonBox.Ok | QtWidgets.QDialogButtonBox.Cancel)
        lo.addWidget(btns)

        # Result handling: 'custom'=XY dialog, index=int=use that aircraft's waypoints
        result_ref = [None]  # mutable container for closure capture
        def _on_ok():
            result_ref[0] = sel.currentRow() if sel.currentRow() >= 0 else None
            dlg.accept()
        def _on_custom():
            result_ref[0] = 'custom'
            dlg.accept()
        btns.accepted.connect(_on_ok)
        btns.rejected.connect(dlg.reject)
        btn_custom.clicked.connect(_on_custom)

        if dlg.exec_() != QtWidgets.QDialog.Accepted or result_ref[0] is None:
            return

        if result_ref[0] == 'custom':
            self._pending_formation_path = fname
            self._open_precise_wp_dialog()
            return

        row = result_ref[0]
        if row < 0 or row >= len(wp_options):
            return
        leader_name = wp_options[row][0]
        leader_wps = self._aircraft_waypoints.get(leader_name, [])
        form["waypoints"] = [np.array(w) for w in leader_wps]
        for n in form["members"]:
            self._aircraft_waypoints[n] = [np.array(w) for w in leader_wps]
        self._rebuild_waypoint_actors()
        self._refresh_waypoint_tree()
        self._log_action(f"{fname} 路径点来源: {leader_name} ({len(leader_wps)} 点)")
        self.statusBar().showMessage(f"{fname} 路径点已设置: {leader_name}", 3000)

    def _formation_start_flight(self):
        if not self._formations:
            return
        fname = list(self._formations.keys())[-1]
        form = self._formations[fname]
        wps = form.get("waypoints", [])
        if len(wps) < 2:
            self.statusBar().showMessage(f"⚠ {fname} 路径点不足（≥2个）", 3000)
            return
        self._draw_flight_preview(wps)
        for n in form["members"]:
            self._aircraft_waypoints[n] = [np.array(w) for w in wps]
        self._flight_aircraft_combo.setCurrentText(form["leader"])
        self._start_flight(form["leader"])
        self._log_action(f"{fname} 开始飞行 ({len(form['members'])} 机)")

    def _cancel_formation(self):
        if not self._formations:
            return
        fname = list(self._formations.keys())[0]
        form = self._formations.pop(fname)
        if form.get("tree_node"):
            parent = form["tree_node"].parent()
            if parent:
                parent.removeChild(form["tree_node"])
        # Restore waypoints
        if self._formation_saved_wp:
            self._aircraft_waypoints.clear()
            for n, wps in self._formation_saved_wp.items():
                if wps:
                    self._aircraft_waypoints[n] = [np.array(w) for w in wps]
            self._formation_saved_wp.clear()
        self._rebuild_waypoint_actors()
        self._refresh_waypoint_tree()
        self._btn_formation.setText("开始编队")
        self._btn_formation.setEnabled(True)
        self._btn_cancel_formation.setVisible(False)
        self._log_action(f"用户取消了{fname}，路径点已恢复")
        self.statusBar().showMessage(f"{fname} 已取消，路径点已恢复", 3000)

    # ────────────────────────────────────────────────────────────────
    #  ID-15: Flight speed model (systematic, physics-aware)
    # ────────────────────────────────────────────────────────────────

    CRUISE_SPEED = 5.0
    FLIGHT_INTERVAL_MS = 50
    FORMATION_TRAIL_DIST = 3.0
    _MIN_FLIGHT_STEPS = 10
    _MAX_FLIGHT_STEPS = 300

    def _flight_speed(self, pitch_deg, yaw_rate_deg_s):
        return compute_flight_speed(pitch_deg, yaw_rate_deg_s, self.CRUISE_SPEED)

    def _draw_flight_preview(self, aircraft_wps):
        """Draw a semi-transparent preview line through all waypoints."""
        if len(aircraft_wps) < 2:
            return
        pts = np.array(aircraft_wps)
        poly = pv.PolyData()
        poly.points = pts
        cells = np.hstack([[len(pts)], list(range(len(pts)))])
        poly.lines = cells
        self._path_actor = self.plotter.add_mesh(
            poly, color="cyan", line_width=2, opacity=0.6)
        self.plotter.render()

    def _start_flight(self, name=None):
        """Animate an aircraft through its waypoints. Multi-flight safe."""
        if name is None:
            name = self._flight_aircraft_combo.currentText()
        if not name or "aircraft" not in name.lower():
            return
        aircraft_wps = self._get_aircraft_waypoints(name)
        if len(aircraft_wps) < 2:
            self.statusBar().showMessage(f"{name} 路径点不足（≥2个）", 3000)
            return
        self._draw_flight_preview(aircraft_wps)

        path = [np.array(wp) for wp in aircraft_wps]
        t = self._get_or_init_transform(name)
        interval_s = self.FLIGHT_INTERVAL_MS / 1000.0

        raw = []
        for i in range(len(path) - 1):
            p0, p1 = path[i], path[i+1]
            dx, dy, dz = p1[0]-p0[0], p1[1]-p0[1], p1[2]-p0[2]
            horiz_dist = np.sqrt(dx*dx + dy*dy) + 1e-12
            dist = np.linalg.norm(p1-p0) + 1e-12
            yaw = math.degrees(math.atan2(dy, dx)) % 360.0
            pitch = max(-90.0, min(90.0, math.degrees(math.atan2(-dz, horiz_dist))))
            raw.append({"dist": dist, "yaw": yaw, "pitch": pitch, "from": p0.tolist(), "to": p1.tolist()})

        segments = []
        for i, r in enumerate(raw):
            d_yaw = 0.0
            if i > 0:
                d_yaw = (r["yaw"] - raw[i-1]["yaw"]) % 360.0
                if d_yaw > 180.0: d_yaw -= 360.0
            pitch_rad = math.radians(r["pitch"])
            k_pitch_est = max(0.30, 1.0 - 0.35*math.sin(pitch_rad))
            approx_speed = self.CRUISE_SPEED * k_pitch_est
            approx_time = r["dist"]/approx_speed if approx_speed > 0 else 999.0
            yaw_rate = abs(d_yaw)/approx_time if approx_time > 0 else 0.0
            V = self._flight_speed(r["pitch"], yaw_rate)
            seg_time = r["dist"]/V if V > 0 else 999.0
            steps = max(self._MIN_FLIGHT_STEPS, min(self._MAX_FLIGHT_STEPS, int(round(seg_time/interval_s))))
            segments.append({"from": r["from"], "to": r["to"], "yaw": r["yaw"], "pitch": r["pitch"], "roll": 0.0, "steps": steps})

        total_steps = sum(s["steps"] for s in segments)
        total_time_ms = total_steps * self.FLIGHT_INTERVAL_MS

        t["offset"] = list(aircraft_wps[0])
        self._apply_obj_transform_to_actor(name)

        timer = QTimer(self)
        ft = {
            "name": name, "timer": timer, "path": path, "segments": segments,
            "seg_idx": 0, "step": 0, "formation": [name],
            "cache": {"aircraft_name": name, "waypoints": [list(w) for w in aircraft_wps[1:]],
                       "segments": segments, "interval_ms": self.FLIGHT_INTERVAL_MS,
                       "start_position": list(aircraft_wps[0]), "formation": [name]},
            "total_steps": total_steps, "total_time_ms": total_time_ms,
        }
        timer.timeout.connect(lambda n=name: self._flight_tick(n))
        self._flights[name] = ft
        if not self._flight_active:
            self._set_flight_ui_active(True)
        self._flight_active = True
        # Update timeline for first flight
        self._total_flight_steps = total_steps
        self._total_flight_time_ms = total_time_ms
        self._tl_slider.setRange(0, total_time_ms)
        self._tl_slider.setValue(0)
        self._setup_keyframe_labels(len(aircraft_wps))
        self._btn_start_flight.setText("停止飞行")
        timer.start(self.FLIGHT_INTERVAL_MS)
        self._log_action(f"开始飞行: {name} ({len(aircraft_wps)} 路径点, {total_time_ms/1000:.1f}s)")
        self.statusBar().showMessage(f"飞行: {name} ({total_time_ms/1000:.1f}s)", 3000)

    def _set_flight_ui_active(self, active):
        self._btn_start_flight.setText("停止飞行" if active else "开始飞行")
        self._flight_aircraft_combo.setEnabled(not active)
        self._btn_formation.setEnabled(not active)
        self._btn_save_flight.setEnabled(not active)
        self._btn_load_flight.setEnabled(not active)

    def _stop_flight(self, name=None):
        if name:
            f = self._flights.pop(name, None)
            if f: f["timer"].stop()
        else:
            for f in list(self._flights.values()):
                f["timer"].stop()
            self._flights.clear()
        if not self._flights:
            self._flight_active = False
            self._set_flight_ui_active(False)
            self._log_action("飞行停止")

    @staticmethod
    def _catmull_rom_position(path, seg_idx, t):
        n = len(path)
        p1, p2 = path[seg_idx], path[min(seg_idx+1, n-1)]
        p0 = path[seg_idx-1] if seg_idx > 0 else 2*p1 - p2
        p3 = path[seg_idx+2] if seg_idx+2 < n else 2*p2 - p1
        t2, t3 = t*t, t*t*t
        return 0.5 * ((2*p1) + (-p0+p2)*t + (2*p0-5*p1+4*p2-p3)*t2 + (-p0+3*p1-3*p2+p3)*t3)

    @staticmethod
    def _lerp_angle(a, b, t):
        d = (b - a) % 360.0
        if d > 180.0: d -= 360.0
        return (a + d * t) % 360.0

    def _flight_tick(self, name):
        f = self._flights.get(name)
        if f is None:
            return
        try:
            seg_idx, step = f["seg_idx"], f["step"]
            segments, path = f["segments"], f["path"]
            if seg_idx >= len(segments):
                f["timer"].stop()
                self._flights.pop(name, None)
                if not self._flights:
                    self._flight_active = False
                    self._set_flight_ui_active(False)
                self._log_action(f"{name} 飞行完成")
                self.statusBar().showMessage(f"{name} 飞行完成", 3000)
                return
            seg = segments[seg_idx]
            steps_in_seg = seg["steps"]
            t_val = step / float(steps_in_seg) if steps_in_seg > 0 else 1.0
            t_val = min(t_val, 1.0)
            pos = self._catmull_rom_position(path, seg_idx, t_val)
            entry_yaw = float(seg["yaw"])
            exit_yaw = float(segments[seg_idx+1]["yaw"]) if seg_idx+1 < len(segments) else entry_yaw
            yaw = self._lerp_angle(entry_yaw, exit_yaw, t_val)
            entry_pitch = float(seg["pitch"])
            exit_pitch = float(segments[seg_idx+1]["pitch"]) if seg_idx+1 < len(segments) else entry_pitch
            pitch = max(-90.0, min(90.0, entry_pitch + (exit_pitch - entry_pitch) * t_val))
            self._apply_flight_state_single(name, pos, yaw, pitch)
            self._update_timeline()
            f["step"] += 1
            if f["step"] >= steps_in_seg:
                f["seg_idx"] += 1
                f["step"] = 0
        except Exception as e:
            self._stop_flight(name)
            self.statusBar().showMessage(f"飞行错误({name}): {e}", 5000)

    def _apply_flight_state_single(self, name, pos, yaw, pitch):
        if name in self._obj_transforms:
            self._obj_transforms[name]["offset"] = pos.tolist()
            self._obj_transforms[name]["yaw"] = yaw
            self._obj_transforms[name]["pitch"] = pitch
            self._apply_obj_transform_to_actor(name)
        # Update slider if this is the selected object
        cur = self._obj_combo.currentText().replace("[自定义] ", "")
        if cur == name:
            self._set_slider_value(self._slider_obj_x, pos[0])
            self._set_slider_value(self._slider_obj_y, pos[1])
            self._set_slider_value(self._slider_obj_z, pos[2])
            self._set_slider_value(self._slider_obj_yaw, yaw)
            self._set_slider_value(self._slider_obj_pitch, pitch)

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
        if not self._flights:
            self.statusBar().showMessage("没有可保存的飞行数据，请先执行一次飞行", 3000)
            return
        # Use the first flight's cache
        first = next(iter(self._flights.values()))
        cache = first["cache"]

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

        # ── Convert numpy arrays to plain Python lists for JSON serialization ──
        def _to_py(val):
            if isinstance(val, np.ndarray):
                return val.tolist()
            if isinstance(val, list):
                return [_to_py(v) for v in val]
            return val

        data = {
            "aircraft_name": cache["aircraft_name"],
            "formation": cache.get("formation", [cache["aircraft_name"]]),
            "start_position": _to_py(cache["start_position"]),
            "waypoints": _to_py(cache["waypoints"]),
            "segments": segments,
            "interval_ms": cache["interval_ms"],
            "description": (
                "Flight path data for 3DSceneSoftware. "
                "segments[i] contains yaw/pitch/roll for travel from 'from' to 'to'."
            ),
        }
        try:
            with open(name, "w") as f:
                json.dump(data, f, indent=2)
        except Exception as e:
            QtWidgets.QMessageBox.warning(self, "保存失败", f"写入 JSON 文件时出错:\n{e}")
            return

        # ── Save companion 3D scatter plot ──
        if _HAS_MPL:
            try:
                fig = plt.figure(figsize=(10, 8))
                ax = fig.add_subplot(111, projection="3d")
                wps = data["waypoints"]
                all_pts = [data["start_position"]] + wps
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
                ax.set_title(f'Flight Path: {data["aircraft_name"]}')
                ax.legend()
                png_path = name.replace(".json", "_3dplot.png")
                fig.savefig(png_path, dpi=150, bbox_inches="tight")
                plt.close(fig)
            except Exception as e:
                print(f"Failed to generate 3D plot: {e}")

        self._log_action(f"用户保存了飞行数据：{os.path.basename(name)}")
        self.statusBar().showMessage(f"飞行数据已保存: {name}", 3000)

    def _load_flight_data(self):
        path, _ = QtWidgets.QFileDialog.getOpenFileName(
            self, "载入飞行数据",
            self._ensure_flight_dir(),
            "JSON (*.json)"
        )
        if not path:
            return
        self._loaded_flight_path = path

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

        # Build flight path: start_position + waypoints (backward compat)
        # Old saved files have waypoints including start_position — deduplicate.
        saved_wps = data.get("waypoints", [])
        if start_pos is not None and saved_wps and saved_wps[0] == list(start_pos):
            saved_wps = saved_wps[1:]
        path_arr = [np.array(p) for p in [start_pos] + saved_wps]

        # Create flight entry in _flights dict
        timer = QTimer(self)
        ft = {
            "name": name, "timer": timer, "path": path_arr, "segments": segments,
            "seg_idx": 0, "step": 0, "formation": formation,
            "cache": data,
            "total_steps": total_steps, "total_time_ms": total_time_ms,
        }
        timer.timeout.connect(lambda n=name: self._flight_tick(n))
        self._flights[name] = ft
        self._flight_active = True
        self._set_flight_ui_active(True)

        # Timeline
        self._total_flight_steps = total_steps
        self._total_flight_time_ms = total_time_ms
        self._tl_slider.setRange(0, total_time_ms)
        self._tl_slider.setValue(0)
        self._setup_keyframe_labels(len(saved_wps) + 1)

        timer.start(interval_ms)


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
        if not self._flights:
            return
        f = next(iter(self._flights.values()))
        accum = sum(s["steps"] for i, s in enumerate(f["segments"]) if i < f["seg_idx"])
        accum += f["step"]
        current_ms = 0
        if f["total_steps"] > 0:
            current_ms = int((accum / f["total_steps"]) * f["total_time_ms"])
        self._tl_slider.blockSignals(True)
        self._tl_slider.setValue(current_ms)
        self._tl_slider.blockSignals(False)
        self._update_timeline_label(current_ms)

    def _update_timeline_label(self, current_ms=None):
        if current_ms is None:
            current_ms = self._tl_slider.value()
        total_s = (self._total_flight_time_ms / 1000.0) if self._total_flight_time_ms > 0 else 0
        self._tl_time_label.setText(f"{current_ms/1000:.1f} / {total_s:.1f} 秒")

    # ── Timeline seek handlers ──────────────────────────────

    def _on_tl_pressed(self):
        for f in self._flights.values():
            f["timer"].stop()

    def _on_tl_seek(self, value_ms):
        if not self._flights:
            return
        f = next(iter(self._flights.values()))
        total_ms = f["total_time_ms"]
        total_steps = f["total_steps"]
        if total_ms <= 0:
            return
        target_step = int((value_ms / total_ms) * total_steps)
        target_step = max(0, min(total_steps - 1, target_step))
        accum = 0
        for seg_idx, seg in enumerate(f["segments"]):
            if accum + seg["steps"] > target_step:
                f["seg_idx"] = seg_idx
                f["step"] = target_step - accum
                break
            accum += seg["steps"]
        self._tl_slider.blockSignals(True)
        self._tl_slider.setValue(value_ms)
        self._tl_slider.blockSignals(False)
        self._update_timeline_label(value_ms)

    def _on_tl_released(self):
        for f in self._flights.values():
            f["timer"].start(self.FLIGHT_INTERVAL_MS)

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

        self._log_action("用户执行了碰撞检测")
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
        self._log_action(f"用户选中了对象：{name}")

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

    def _log_action(self, msg):
        if not hasattr(self, '_log_text') or self._log_text is None:
            return
        import datetime
        ts = datetime.datetime.now().strftime("%H:%M:%S")
        self._log_text.append(f"[{ts}] {msg}")
        sb = self._log_text.verticalScrollBar()
        sb.setValue(sb.maximum())

    def _export_log(self):
        path, _ = QtWidgets.QFileDialog.getSaveFileName(
            self, "导出操作日志", "operation.log", "LOG (*.log *.txt)"
        )
        if not path:
            return
        try:
            with open(path, "w", encoding="utf-8") as f:
                f.write(self._log_text.toPlainText())
            self.statusBar().showMessage(f"操作日志已导出: {path}", 3000)
        except Exception as e:
            QtWidgets.QMessageBox.warning(self, "导出失败", f"导出日志时出错:\n{e}")

    def _on_measurement_log(self, text):
        self._log_action(f"用户使用了测量工具，{text}")

    def _update_info(self, mouse_world=None):
        lines = [""]
        if mouse_world is not None:
            try:
                x, y, z = mouse_world
                lines.append("🖱 鼠标 (世界坐标)")
                lines.append(f"  X={x:.2f},  Y={y:.2f},  Z={z:.2f}")
            except Exception:
                pass
        try:
            cam = self.plotter.camera
            pos = cam.GetPosition()
            fp = cam.GetFocalPoint()
            lines.append("")
            lines.append("📷 相机")
            lines.append(f"  位置:   X={pos[0]:.2f},  Y={pos[1]:.2f},  Z={pos[2]:.2f}")
            lines.append(f"  目标:   X={fp[0]:.2f},  Y={fp[1]:.2f},  Z={fp[2]:.2f}")
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
        if info is None or not info.get("visible", True):
            return
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
        self._stop_flight()
        self._recording = False
        try:
            self.plotter.close()
        except Exception:
            pass

    def _open_user_manual(self):
        manual_path = os.path.join(os.path.dirname(os.path.dirname(
            os.path.abspath(__file__))), "docs", "user_manual.html")
        try:
            webbrowser.open("file://" + manual_path)
        except Exception:
            QtWidgets.QMessageBox.warning(self, "打开失败",
                f"无法打开用户手册。\n路径: {manual_path}")

    def _show_about(self):
        dlg = AboutDialog(self)
        dlg.setAttribute(QtCore.Qt.WA_DeleteOnClose)
        dlg.show()



class AboutDialog(QtWidgets.QDialog):
    def __init__(self, parent=None):
        super().__init__(parent)
        self.setWindowTitle("关于 3DSceneSoftware")
        self.setWindowFlags(self.windowFlags() | QtCore.Qt.Window)
        self.resize(500, 600)

        lo = QtWidgets.QVBoxLayout(self)
        lo.setSpacing(8)

        title = QtWidgets.QLabel("3DSceneSoftware v2.4")
        title.setStyleSheet("font-size: 20px; font-weight: bold; color: #1a1a2e;")
        lo.addWidget(title)

        subtitle = QtWidgets.QLabel("基于 PyQt5 + PyVista (VTK 引擎) | 可视化 3D 大场景建模平台")
        subtitle.setStyleSheet("font-size: 12px; color: #666;")
        lo.addWidget(subtitle)
        lo.addSpacing(10)

        notes = QtWidgets.QTextEdit()
        notes.setReadOnly(True)
        notes.setStyleSheet("font-family: Menlo, monospace; font-size: 11px;")
        notes.setHtml("""
<h3>Release Notes</h3>
<hr>
<h4>v2.4 — 2026-07-08</h4>
<ul>
<li><b>修复:</b> macOS Apple Silicon + Rosetta 2 环境下 VTK 启动死锁 — 用 4 个精确导入替代 <code>import vtk</code>（144 个 x86_64 .so 模块），启动时间从 30s+（超时）降至 ~8s</li>
<li><b>修复:</b> Retina 屏鼠标拾取偏移 — <code>_to_vtk_display()</code> 去掉 devicePixelRatio 倍乘，VTK render window 报告逻辑像素，提逻辑坐标即可</li>
<li><b>新增:</b> 场景信息独立窗口（双击场景设置 → 场景信息）— 自动刷新坐标系/地形尺寸/Z偏移/垂直夸张/相机视角</li>
<li><b>新增:</b> 操作日志导出（文件 → 导出操作日志... → .log 文件）</li>
<li><b>恢复:</b> 添加3D路径点（单击场景）— 路径规划树节点支持单击 scene 放置路径点</li>
<li><b>恢复:</b> 路径规划 WaypointPreciseDialog + LayerManagementDialog 等高线支持</li>
<li><b>修复:</b> DEM 导入后垂直夸张系数正确保存到 config，场景信息实时显示</li>
<li><b>修复:</b> 坐标系从硬编码 "WGS84 (经纬度)" 改为动态读取 config (ENU + DEM CRS)</li>
</ul>

<h4>v2.3 — 2026-07-07</h4>
<ul>
<li>UI 重构：场景树 + 属性面板 QStackedWidget + 坐标简化</li>
<li>ASC 格网导入/导出 (ESRI ASCII Grid 格式)</li>
<li>飞行按钮修复 / 编队僚机重叠修复</li>
<li>删除坐标系切换 (ENU/FLU/NED/NWU)</li>
</ul>

<h4>v2.2 — 2026-07-06</h4>
<ul>
<li>图层管理对话框 (XY 绘图工具 + 矩形/圆形/多边形选区)</li>
<li>3D 交互鲁棒性改进 (picker 精度提升)</li>
<li>DEM 保存/载入修复 (X,Y 网格完整重构)</li>
<li>DEM 相机视图动态范围 / 飞机缩放到 2000×</li>
<li>删除 STL/OBJ 导入</li>
</ul>

<h4>v2.1 — 2026-07-06</h4>
<ul>
<li>鼠标坐标精度修复 (Retina 屏幕 1px 偏差)</li>
<li>图层管理移至菜单 / Z 夸张选项 / 飞机默认高度 7000m</li>
<li>精准路径点 XY 自动范围</li>
</ul>

<h4>v2.0 — 2026-07-06</h4>
<ul>
<li>PyVista extract_surface() 崩溃修复</li>
</ul>

<h4>v1.x — 2026-06-29 ~ 2026-07-03</h4>
<ul>
<li>飞机姿态控制 (Yaw/Pitch/Roll 局部旋转)</li>
<li>路径点动态飞行 (Catmull-Rom 样条 + 速度模型)</li>
<li>时间轴 / 编队飞行 / DEM 图层管理</li>
<li>保存/载入 JSON 数据 / 测量工具 / 碰撞检测</li>
<li>FlightWindow 独立窗口 (macOS 双 QVTK 已废弃)</li>
</ul>
""")
        lo.addWidget(notes)

        btn_close = QtWidgets.QPushButton("关闭")
        btn_close.clicked.connect(self.close)
        lo.addWidget(btn_close)



class WaypointPreciseDialog(QtWidgets.QDialog):
    """Precise waypoint dialog with terrain-extent-aware XY/Z ranges."""

    def __init__(self, parent=None, terrain_extent=None, terrain_mesh=None):
        """*terrain_extent* = ``(xy_half, z_half)`` or ``None`` for defaults."""
        super().__init__(parent)
        self.setWindowTitle("精准添加路径点")
        self.setMinimumSize(640, 520)

        if terrain_extent is not None:
            xy_half, z_half = terrain_extent
        else:
            xy_half, z_half = 10.0, 20.0

        self._xy_limit = xy_half * 1.2
        self._z_limit = z_half * 1.5
        self._spin_xy_range = xy_half * 2.0
        self._terrain_mesh = terrain_mesh

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
        self._ax_xy.set_xlim(-self._xy_limit, self._xy_limit)
        self._ax_xy.set_ylim(-self._xy_limit, self._xy_limit)
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
        self._spin_x.setRange(-self._spin_xy_range, self._spin_xy_range)
        self._spin_x.setDecimals(2)
        self._spin_x.setPrefix("X: ")
        self._spin_x.valueChanged.connect(self._on_xy_spin_changed)
        self._spin_y = QtWidgets.QDoubleSpinBox()
        self._spin_y.setRange(-self._spin_xy_range, self._spin_xy_range)
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
        self._ax_z.set_ylim(-self._z_limit, self._z_limit)
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
        self._spin_z.setRange(-self._z_limit, self._z_limit)
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

    def _draw_contours(self, ax):
        mesh = self._terrain_mesh
        if mesh is None:
            return
        try:
            surface = mesh.extract_surface()
            pts = np.asarray(surface.points)
            if len(pts) < 50:
                return
            if len(pts) > 10000:
                step = max(1, len(pts) // 8000)
                pts = pts[::step]
            import matplotlib.tri as tri
            triang = tri.Triangulation(pts[:, 0], pts[:, 1])
            z = pts[:, 2]
            z_min, z_max = z.min(), z.max()
            if z_max - z_min < 1.0:
                return
            n_levels = 8 if len(pts) > 3000 else 12
            levels = np.linspace(z_min, z_max, n_levels)
            ax.tricontour(triang, z, levels=levels,
                          colors="gray", linewidths=0.4, alpha=0.4)
        except Exception:
            pass

    def _redraw_xy(self):
        self._ax_xy.clear()
        self._ax_xy.set_xlim(-self._xy_limit, self._xy_limit)
        self._ax_xy.set_ylim(-self._xy_limit, self._xy_limit)
        self._ax_xy.set_aspect("equal")
        self._ax_xy.grid(True, linestyle="--", alpha=0.7)
        self._ax_xy.set_xlabel("X")
        self._ax_xy.set_ylabel("Y")
        self._ax_xy.axhline(0, color="gray", linewidth=0.5)
        self._ax_xy.axvline(0, color="gray", linewidth=0.5)
        self._draw_contours(self._ax_xy)

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
        self._ax_z.set_ylim(-self._z_limit, self._z_limit)
        self._ax_z.grid(True, linestyle="--", alpha=0.7)
        self._ax_z.set_xticks(cum_dists)
        self._ax_z.set_xticklabels([f"{d:.2f}" for d in cum_dists], rotation=45)

        sel_label = self._label_for(self._selected_z_idx)
        self._lbl_point_info.setText(f"当前点: {sel_label}, 距离: {sel_x:.2f}")
        z_val = self._z_values[self._selected_z_idx]
        block = self._spin_z.blockSignals(True)
        self._spin_z.setValue(z_val)
        self._spin_z.blockSignals(block)
        z_min, z_max = -self._z_limit, self._z_limit
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
        z_val = round(max(-self._z_limit, min(self._z_limit, event.ydata)), 2)
        self._z_values[idx] = z_val
        block = self._spin_z.blockSignals(True)
        self._spin_z.setValue(z_val)
        self._spin_z.blockSignals(block)
        z_min, z_max = -self._z_limit, self._z_limit
        frac = (z_val - z_min) / (z_max - z_min)
        block_sl = self._z_slider.blockSignals(True)
        self._z_slider.setValue(int(frac * 1000))
        self._z_slider_label.setText(f"{z_val:.2f}")
        self._z_slider.blockSignals(block_sl)
        self._redraw_z()

    def _on_z_slider_changed(self, value):
        if not self._z_values or self._selected_z_idx >= len(self._z_values):
            return
        z_min, z_max = -self._z_limit, self._z_limit
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
        z_min, z_max = -self._z_limit, self._z_limit
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


class SceneSettingsDialog(QtWidgets.QDialog):
    def __init__(self, main_window, parent=None):
        super().__init__(parent)
        self.mw = main_window
        self.setWindowTitle("场景设置")
        self.setWindowFlags(self.windowFlags() | QtCore.Qt.Window)
        self.resize(420, 520)

        lo = QtWidgets.QVBoxLayout(self)
        lo.setSpacing(10)

        self._labels = {}
        fields = [
            ("作者", "Eric"),
            ("日期", ""),
            ("坐标系", ""),
            ("当前地形尺寸", ""),
            ("Z 轴基准偏移", ""),
            ("边界锁定范围", ""),
            ("地形垂直夸张系数", ""),
            ("默认相机视角", ""),
        ]
        for label, default in fields:
            lo.addWidget(QtWidgets.QLabel(label))
            val = QtWidgets.QLabel(default)
            val.setStyleSheet("font-size: 15px; font-weight: bold; color: #2a66b0; "
                              "padding: 4px 8px; background: #f5f5f5; border-radius: 4px;")
            val.setTextInteractionFlags(Qt.TextSelectableByMouse)
            lo.addWidget(val)
            self._labels[label] = val

        lo.addStretch()

        self._refresh_lock = False  # guard against concurrent refreshes
        # Defer first refresh to avoid VTK/OpenGL stalls during dialog construction
        self._timer = QtCore.QTimer(self)
        self._timer.timeout.connect(self._refresh)
        self._timer.start(1000)
        QtCore.QTimer.singleShot(0, self._refresh)

    def _refresh(self) -> None:
        """Refresh all labels — called on init (deferred) and every 1 s."""
        if self._refresh_lock:
            return
        self._refresh_lock = True
        mw = self.mw
        now = QtCore.QDate.currentDate().toString("yyyy-MM-dd")
        self._labels["日期"].setText(now)
        self._labels["作者"].setText("Eric")
        cs = mw.config.get("coordinate_system", "ENU")
        dem_crs = mw.config.get("dem_crs", "")
        cs_info = COORD_SYSTEMS.get(cs, {})
        cs_label = cs_info.get("label", cs)
        if dem_crs:
            self._labels["坐标系"].setText(f"{dem_crs}  ({cs_label})")
        else:
            self._labels["坐标系"].setText(f"{cs_label}")
        info = mw.scene_objects.get("terrain")
        if info is not None:
            try:
                b = info["mesh"].bounds
                x_span = abs(b[1] - b[0])
                y_span = abs(b[3] - b[2])
                self._labels["当前地形尺寸"].setText(
                    f"X: {x_span:,.1f} 米,  Y: {y_span:,.1f} 米")
                self._labels["Z 轴基准偏移"].setText("海平面 0 米")
                self._labels["边界锁定范围"].setText(
                    f"X [{b[0]:,.1f}, {b[1]:,.1f}]\n"
                    f"Y [{b[2]:,.1f}, {b[3]:,.1f}]\n"
                    f"Z [{b[4]:,.1f}, {b[5]:,.1f}]")
            except Exception:
                pass
        ve = mw.config.get("elevation_scale", 1.0)
        self._labels["地形垂直夸张系数"].setText(f"{ve:.1f}")
        try:
            cam = mw.plotter.camera
            pos = cam.GetPosition()
            fp = cam.GetFocalPoint()
            dx, dy, dz = pos[0]-fp[0], pos[1]-fp[1], pos[2]-fp[2]
            if abs(dx) < 1 and abs(dz) < 1 and dy < 0:
                view = "俯视 (Top)"
            elif abs(dx) < 1 and abs(dz) < 1 and dy > 0:
                view = "仰视 (Bottom)"
            elif abs(dy) < 1 and abs(dz) < 1:
                view = "侧视 (Side)"
            elif abs(dy) < 1 and abs(dx) < 1:
                view = "正视 (Front)"
            else:
                view = "透视 (Perspective)"
        except Exception:
            view = "—"
        self._labels["默认相机视角"].setText(view)
        self._refresh_lock = False


