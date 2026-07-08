"""
LayerManagementDialog — shape-based terrain layer editor.

Two-page QDialog:
  1. Select which terrain layers (sand/grass/earth/river/vegetation) to manage.
  2. Draw XY shapes (rectangle, square, triangle, circle, polygon) to define
     where the selected layers should be placed.
"""
import math
import numpy as np
from PyQt5 import QtWidgets, QtCore, QtGui
from vtkmodules.vtkRenderingCore import vtkActor
import matplotlib.tri as tri

_HAS_MPL = True
try:
    import matplotlib
    matplotlib.use("Qt5Agg")
    import matplotlib.pyplot as plt
    from matplotlib.backends.backend_qt5agg import FigureCanvasQTAgg
    from matplotlib.figure import Figure
    from matplotlib.path import Path
except ImportError:
    _HAS_MPL = False


# ── shape colours ─────────────────────────────────────────
SHAPE_COLORS = [
    "#2196F3", "#4CAF50", "#FF9800", "#E91E63",
    "#9C27B0", "#00BCD4", "#FF5722", "#607D8B",
]


class LayerManagementDialog(QtWidgets.QDialog):
    """Manage terrain layers with XY shape drawing tools.

    Parameters
    ----------
    parent : QWidget
    layer_names : dict  {key: label}
        Available terrain layer definitions.
    terrain_extent : (float, float)
        (xy_half, z_half) from the terrain mesh.
    is_dem_scene : bool
        True → DEM scene (only sand/grass/earth available).
    """

    def __init__(self, parent, layer_names, terrain_extent, is_dem_scene, terrain_mesh=None):
        super().__init__(parent)
        self.setWindowTitle("增加图层")
        self.setMinimumSize(900, 640)

        self._layer_names = dict(layer_names)
        self._is_dem = is_dem_scene
        xy_half, z_half = terrain_extent if terrain_extent else (10.0, 20.0)
        self._xy_limit = xy_half * 1.2
        self._terrain_mesh = terrain_mesh

        # State
        self._layer_checks = {}       # key → QCheckBox
        self._shapes = []             # list of shape dicts
        self._selected_shape_idx = -1
        self._pend_pts = []           # pending points for current drawing
        self._pend_color = SHAPE_COLORS[0]
        self._current_tool = None

        # Build UI
        layout = QtWidgets.QVBoxLayout(self)
        self._stack = QtWidgets.QStackedWidget()
        layout.addWidget(self._stack)
        self._stack.addWidget(self._build_page1())
        self._stack.addWidget(self._build_page2())
        self._stack.setCurrentIndex(0)

    # ── Page 1: Layer selection ──────────────────────────────

    def _build_page1(self):
        w = QtWidgets.QWidget()
        lo = QtWidgets.QVBoxLayout(w)

        title = QtWidgets.QLabel("图层管理 — 选择需要管理的图层")
        title.setStyleSheet("font-weight: bold; font-size: 14px;")
        lo.addWidget(title)

        lo.addWidget(QtWidgets.QLabel("勾选需要编辑的图层，然后点击「下一步」："))

        for key, label in self._layer_names.items():
            if key in ("river", "vegetation"):
                continue
            # In DEM mode, only show sand/grass/earth
            if self._is_dem and key not in ("layer_sand", "layer_grass", "layer_earth"):
                continue
            if key.startswith("layer_"):
                display = label
            else:
                display = f"{label} ({key})"
            chk = QtWidgets.QCheckBox(display)
            chk.setChecked(False)
            self._layer_checks[key] = chk
            lo.addWidget(chk)

        lo.addStretch()
        btn_lo = QtWidgets.QHBoxLayout()
        btn_cancel = QtWidgets.QPushButton("取消")
        btn_cancel.clicked.connect(self.reject)
        self._btn_next = QtWidgets.QPushButton("下一步")
        self._btn_next.clicked.connect(self._on_next)
        btn_lo.addStretch()
        btn_lo.addWidget(btn_cancel)
        btn_lo.addWidget(self._btn_next)
        lo.addLayout(btn_lo)
        return w

    def _on_next(self):
        """Validate selection and go to page 2 or apply directly."""
        selected_layers = [
            k for k, c in self._layer_checks.items() if c.isChecked()
        ]
        if not selected_layers:
            QtWidgets.QMessageBox.warning(self, "提示", "请至少选择一个图层")
            return

        # Check if any sand/grass/earth is selected
        has_shape_layers = any(
            k in ("layer_sand", "layer_grass", "layer_earth")
            for k in selected_layers
        )
        if has_shape_layers:
            self._selected_layers = selected_layers
            self._stack.setCurrentIndex(1)
            self._redraw_canvas()
        else:
            # Only river/vegetation selected — just toggle visibility
            self._selected_layers = selected_layers
            self.accept()

    # ── Page 2: XY plane with drawing tools ────────────────

    def _build_page2(self):
        w = QtWidgets.QWidget()
        outer_lo = QtWidgets.QVBoxLayout(w)   # single layout on w

        h_main = QtWidgets.QHBoxLayout()

        # ── Left: shape list + controls ──
        left = QtWidgets.QVBoxLayout()
        left.addWidget(QtWidgets.QLabel("已绘制图形:"))
        self._shape_list = QtWidgets.QListWidget()
        self._shape_list.currentRowChanged.connect(self._on_shape_selected)
        left.addWidget(self._shape_list, 1)

        left.addWidget(QtWidgets.QLabel("透明度:"))
        self._opacity_slider = QtWidgets.QSlider(QtCore.Qt.Horizontal)
        self._opacity_slider.setRange(0, 100)
        self._opacity_slider.setValue(100)
        self._opacity_slider.valueChanged.connect(self._on_opacity_changed)
        left.addWidget(self._opacity_slider)
        self._opacity_label = QtWidgets.QLabel("100%")
        left.addWidget(self._opacity_label)

        self._btn_delete_shape = QtWidgets.QPushButton("删除选中图形")
        self._btn_delete_shape.clicked.connect(self._delete_selected_shape)
        self._btn_delete_shape.setEnabled(False)
        left.addWidget(self._btn_delete_shape)

        self._btn_clear_all = QtWidgets.QPushButton("清除全部图形")
        self._btn_clear_all.clicked.connect(self._clear_all_shapes)
        left.addWidget(self._btn_clear_all)

        left_w = QtWidgets.QWidget()
        left_w.setLayout(left)
        left_w.setFixedWidth(200)
        h_main.addWidget(left_w)

        # ── Center: canvas ──
        center = QtWidgets.QVBoxLayout()
        self._fig = Figure(figsize=(5, 5))
        self._ax = self._fig.add_subplot(111)
        self._setup_axes()
        self._canvas = FigureCanvasQTAgg(self._fig)
        self._canvas.mpl_connect("button_press_event", self._on_canvas_click)
        self._canvas.mpl_connect("motion_notify_event", self._on_canvas_move)
        self._canvas.mpl_connect("button_release_event", self._on_canvas_release)
        center.addWidget(self._canvas, 1)

        self._status_label = QtWidgets.QLabel("当前工具: 无")
        center.addWidget(self._status_label)
        self._count_label = QtWidgets.QLabel("已绘制: 0 个图形")
        center.addWidget(self._count_label)
        center_w = QtWidgets.QWidget()
        center_w.setLayout(center)
        h_main.addWidget(center_w, 1)

        # ── Right: tool buttons ──
        right = QtWidgets.QVBoxLayout()
        right.addWidget(QtWidgets.QLabel("绘图工具:"))
        self._tool_group = QtWidgets.QButtonGroup(self)
        # Only polygon tool
        key = "polygon"
        btn = QtWidgets.QPushButton("多边形 (P)")
        btn.setCheckable(True)
        btn.setChecked(True)
        btn.clicked.connect(lambda checked: self._set_tool("polygon"))
        self._tool_group.addButton(btn)
        right.addWidget(btn)
        self._tool_btns = {"polygon": btn}
        self._current_tool = "polygon"

        right.addStretch()
        right_w = QtWidgets.QWidget()
        right_w.setLayout(right)
        right_w.setFixedWidth(150)
        h_main.addWidget(right_w)

        outer_lo.addLayout(h_main, 1)

        # ── Bottom buttons ──
        bottom = QtWidgets.QHBoxLayout()
        btn_prev = QtWidgets.QPushButton("上一步")
        btn_prev.clicked.connect(lambda: self._stack.setCurrentIndex(0))
        btn_ok = QtWidgets.QPushButton("确认应用")
        btn_ok.clicked.connect(self._on_accept)
        bottom.addWidget(btn_prev)
        bottom.addStretch()
        bottom.addWidget(btn_ok)
        outer_lo.addLayout(bottom)

        return w

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
            triang = tri.Triangulation(pts[:, 0], pts[:, 1])
            z = pts[:, 2]
            z_min, z_max = z.min(), z.max()
            if z_max - z_min < 1.0:
                return
            n_levels = 8 if len(pts) > 3000 else 12
            levels = np.linspace(z_min, z_max, n_levels)
            ax.tricontour(triang, z, levels=levels,
                          colors="gray", linewidths=0.4, alpha=0.4,
                          zorder=0)
        except Exception:
            pass

    def _setup_axes(self):
        self._ax.clear()
        lim = self._xy_limit
        self._ax.set_xlim(-lim, lim)
        self._ax.set_ylim(-lim, lim)
        self._ax.set_aspect("equal")
        self._ax.grid(True, linestyle="--", alpha=0.7)
        self._ax.set_xlabel("X")
        self._ax.set_ylabel("Y")
        self._ax.axhline(0, color="gray", linewidth=0.5)
        self._ax.axvline(0, color="gray", linewidth=0.5)
        self._draw_contours(self._ax)

    # ── Tool management ─────────────────────────────────────

    def _set_tool(self, tool_key):
        self._current_tool = tool_key
        self._pend_pts = []
        self._status_label.setText(f"当前工具: {tool_key}")
        # Uncheck all other buttons, check this one
        for k, btn in self._tool_btns.items():
            btn.setChecked(k == tool_key)
        # Change cursor
        if tool_key == "polygon":
            self._canvas.setCursor(QtGui.QCursor(QtCore.Qt.CrossCursor))
        else:
            self._canvas.setCursor(QtGui.QCursor(QtCore.Qt.CrossCursor))

    # ── Canvas interaction ──────────────────────────────────

    def _on_canvas_click(self, event):
        if event.inaxes != self._ax or self._current_tool is None:
            return
        x, y = event.xdata, event.ydata

        # Right-click closes polygon
        if event.button == 3:
            if self._current_tool == "polygon" and len(self._pend_pts) >= 3:
                self._close_polygon()
            return

        if event.button != 1:
            return

        # Check if clicking on existing shape (for selection)
        if self._current_tool is None:
            self._select_shape_at(x, y)
            return

        if self._current_tool == "polygon":
            self._pend_pts.append([x, y])
            self._redraw_canvas()
            return

        self._pend_pts.append([x, y])
        tool = self._current_tool
        needed = {"rectangle": 2, "square": 2, "triangle": 3, "circle": 2}.get(tool, 2)

        if len(self._pend_pts) >= needed:
            self._finalize_shape()
            self._pend_pts = []

        self._redraw_canvas()

    def _on_canvas_move(self, event):
        if event.inaxes != self._ax or not self._pend_pts:
            return
        # Show preview line + coordinates
        self._redraw_canvas(preview=(event.xdata, event.ydata))
        if event.xdata and event.ydata:
            self._status_label.setText(
                f"当前工具: {self._current_tool}  |  "
                f"坐标: ({event.xdata:.2f}, {event.ydata:.2f})  |  "
                f"已点: {len(self._pend_pts)} 点")

    def _on_canvas_release(self, event):
        pass  # Not needed for this implementation

    def _on_shape_selected(self, row):
        if 0 <= row < len(self._shapes):
            self._selected_shape_idx = row
            self._btn_delete_shape.setEnabled(True)
            shape = self._shapes[row]
            opacity = shape.get("opacity", 1.0)
            self._opacity_slider.blockSignals(True)
            self._opacity_slider.setValue(int(opacity * 100))
            self._opacity_slider.blockSignals(False)
            self._opacity_label.setText(f"{int(opacity * 100)}%")
        else:
            self._selected_shape_idx = -1
            self._btn_delete_shape.setEnabled(False)
        self._redraw_canvas()

    def _on_opacity_changed(self, val):
        if self._selected_shape_idx < 0:
            return
        opacity = val / 100.0
        self._shapes[self._selected_shape_idx]["opacity"] = opacity
        self._opacity_label.setText(f"{val}%")
        self._redraw_canvas()

    def _delete_selected_shape(self):
        if self._selected_shape_idx < 0:
            return
        self._shapes.pop(self._selected_shape_idx)
        self._selected_shape_idx = -1
        self._update_shape_list()
        self._redraw_canvas()

    def _clear_all_shapes(self):
        self._shapes.clear()
        self._selected_shape_idx = -1
        self._pend_pts = []
        self._update_shape_list()
        self._redraw_canvas()

    # ── Shape creation ──────────────────────────────────────

    def _finalize_shape(self):
        pts = np.array(self._pend_pts)
        tool = self._current_tool

        if tool == "rectangle":
            x1, y1 = pts[0]
            x2, y2 = pts[1]
            poly = np.array([[x1, y1], [x2, y1], [x2, y2], [x1, y2]])
        elif tool == "square":
            cx, cy = pts[0]
            ex, ey = pts[1]
            half = max(abs(ex - cx), abs(ey - cy))
            poly = np.array([
                [cx - half, cy - half],
                [cx + half, cy - half],
                [cx + half, cy + half],
                [cx - half, cy + half],
            ])
        elif tool == "triangle":
            poly = pts
        elif tool == "circle":
            cx, cy = pts[0]
            ex, ey = pts[1]
            radius = math.hypot(ex - cx, ey - cy)
            angles = np.linspace(0, 2 * math.pi, 37)[:-1]
            poly = np.column_stack([
                cx + radius * np.cos(angles),
                cy + radius * np.sin(angles),
            ])
        elif tool == "polygon":
            poly = pts
        else:
            return

        shape = {
            "type": tool,
            "xy": poly,
            "opacity": 1.0,
        }
        self._shapes.append(shape)
        self._update_shape_list()
        # Auto-select the new shape
        self._shape_list.setCurrentRow(len(self._shapes) - 1)

    def _close_polygon(self):
        """Close a polygon in progress."""
        if len(self._pend_pts) < 3:
            self._pend_pts = []
            self._redraw_canvas()
            return
        self._current_tool = "polygon"
        self._finalize_shape()
        self._pend_pts = []
        self._redraw_canvas()

    # ── Selection ───────────────────────────────────────────

    def _select_shape_at(self, x, y):
        """Find nearest shape within 10 data units and select it."""
        best = -1
        best_dist = 15.0
        for i, shape in enumerate(self._shapes):
            poly = shape["xy"]
            if shape["type"] == "circle" and "radius" in shape:
                cx, cy = shape["center"]
                d = abs(math.hypot(x - cx, y - cy) - shape["radius"])
            else:
                # Distance to any vertex
                d = min(math.hypot(x - p[0], y - p[1]) for p in poly)
            if d < best_dist:
                best_dist = d
                best = i
        if best >= 0:
            self._shape_list.setCurrentRow(best)

    def _on_accept(self):
        """Auto-close pending polygon and accept."""
        if self._pend_pts and self._current_tool == "polygon":
            if len(self._pend_pts) >= 3:
                self._close_polygon()
            else:
                self._pend_pts = []
        self.accept()

    # ── Rendering ───────────────────────────────────────────

    def _redraw_canvas(self, preview=None):
        self._setup_axes()
        lim = self._xy_limit

        for i, shape in enumerate(self._shapes):
            poly = shape["xy"]
            color = SHAPE_COLORS[i % len(SHAPE_COLORS)]
            is_sel = i == self._selected_shape_idx
            fill_alpha = shape.get("opacity", 1.0) * 0.3
            self._ax.fill(
                poly[:, 0], poly[:, 1],
                color=color, alpha=fill_alpha,
                edgecolor=color, linewidth=3 if is_sel else 1.5,
                linestyle="--" if is_sel else "-",
                zorder=2,
            )
            # Label
            cx, cy = poly.mean(axis=0)
            self._ax.text(cx, cy, str(i + 1), fontsize=10,
                          fontweight="bold", ha="center", va="center",
                          color="white" if is_sel else color,
                          bbox=dict(boxstyle="circle", facecolor=color, alpha=0.7))

        # Draw pending points
        if self._pend_pts:
            pts = np.array(self._pend_pts)
            self._ax.plot(pts[:, 0], pts[:, 1], "o-",
                         color=SHAPE_COLORS[len(self._shapes) % len(SHAPE_COLORS)],
                         markersize=6, linewidth=1.5, zorder=3)

        # Preview cursor position + dashed line from last point
        if preview:
            px, py = preview
            self._ax.plot(px, py, "x", color="red", markersize=8, zorder=4)
            if len(self._pend_pts) > 0:
                last = self._pend_pts[-1]
                self._ax.plot([last[0], px], [last[1], py], "--",
                             color="red", alpha=0.5, linewidth=1, zorder=3)

        self._ax.set_xlim(-lim, lim)
        self._ax.set_ylim(-lim, lim)
        self._canvas.draw_idle()

    def _update_shape_list(self):
        self._shape_list.blockSignals(True)
        self._shape_list.clear()
        for i, shape in enumerate(self._shapes):
            label = f"{i + 1}. {shape['type']}"
            self._shape_list.addItem(label)
        self._shape_list.blockSignals(False)
        self._count_label.setText(f"已绘制: {len(self._shapes)} 个图形")

    # ── Results ──────────────────────────────────────────────

    def get_results(self):
        """Return the dialog results.

        Returns
        -------
        dict with keys:
            layer_visibility : {key: bool, ...}
            layer_shapes : {key: [shape_dict, ...], ...}
                Only for sand/grass/earth layers.
            layer_opacity : {key: float, ...}
        """
        visibility = {}
        for key in self._layer_names:
            if key in self._layer_checks:
                visibility[key] = self._layer_checks[key].isChecked()

        shapes_by_layer = {}
        if self._shapes:
            shape_layers = [k for k in self._selected_layers
                            if k in ("layer_sand", "layer_grass", "layer_earth")]
            for key in shape_layers:
                shapes_by_layer[key] = [
                    {
                        "type": s["type"],
                        "xy": s["xy"].tolist(),
                        "opacity": s.get("opacity", 1.0),
                    }
                    for s in self._shapes
                ]

        opacity = {}
        for key in self._selected_layers:
            opacity[key] = 1.0  # full layers use full opacity

        return {
            "layer_visibility": visibility,
            "layer_shapes": shapes_by_layer,
            "layer_opacity": opacity,
        }


class ClipManagerDialog(QtWidgets.QDialog):
    """Manage clip layers created from shape-based layer extraction.

    Shows all clip layers (``*_clip``) with per-item:
        - visibility checkbox
        - opacity slider
        - delete button
    """

    def __init__(self, parent, scene_objects, plotter_actors,
                 toggle_fn, opacity_fn, remove_fn, rebuild_fn):
        super().__init__(parent)
        self.setWindowTitle("图层管理")
        self.setMinimumSize(650, 400)

        self._scene_objects = scene_objects
        self._plotter_actors = plotter_actors
        self._toggle_fn = toggle_fn
        self._opacity_fn = opacity_fn
        self._remove_fn = remove_fn
        self._rebuild_fn = rebuild_fn

        layout = QtWidgets.QVBoxLayout(self)
        layout.addWidget(QtWidgets.QLabel("管理所有通过图层提取创建的裁剪图层:"))

        scroll = QtWidgets.QScrollArea()
        scroll.setWidgetResizable(True)
        self._list_widget = QtWidgets.QWidget()
        self._list_layout = QtWidgets.QVBoxLayout(self._list_widget)
        self._list_layout.setSpacing(6)
        scroll.setWidget(self._list_widget)
        layout.addWidget(scroll, 1)

        btn_lo = QtWidgets.QHBoxLayout()
        btn_lo.addStretch()
        btn_close = QtWidgets.QPushButton("关闭")
        btn_close.clicked.connect(self.accept)
        btn_lo.addWidget(btn_close)
        layout.addLayout(btn_lo)

        self._rebuild_items()

    def _rebuild_items(self):
        while self._list_layout.count():
            item = self._list_layout.takeAt(0)
            if item.widget():
                item.widget().deleteLater()

        clips = [(n, info) for n, info in self._scene_objects.items()
                 if "_clip" in n and "mesh" in info]
        if not clips:
            lbl = QtWidgets.QLabel("暂无裁剪图层。使用「增加图层」绘制多边形来创建。")
            lbl.setStyleSheet("color: #888; padding: 12px;")
            self._list_layout.addWidget(lbl)
            return

        self._row_widgets = {}
        for name, info in clips:
            row = QtWidgets.QWidget()
            row_lo = QtWidgets.QHBoxLayout(row)
            row_lo.setContentsMargins(4, 2, 4, 2)

            chk = QtWidgets.QCheckBox(name)
            chk.setChecked(info.get("visible", True))
            chk.toggled.connect(lambda checked, n=name: self._toggle_fn(n, checked))
            row_lo.addWidget(chk, 1)

            actor = self._plotter_actors.get(name)
            cur_op = 1.0
            if actor is not None:
                try:
                    cur_op = self._resolve_vtk_opacity(actor)
                except Exception:
                    pass
            slider = QtWidgets.QSlider(QtCore.Qt.Horizontal)
            slider.setRange(0, 100)
            slider.setValue(int(cur_op * 100))
            slider.setFixedWidth(120)
            slider.valueChanged.connect(
                lambda val, n=name: self._opacity_fn(n, val / 100.0)
            )
            row_lo.addWidget(slider)

            op_label = QtWidgets.QLabel(f"{int(cur_op * 100)}%")
            op_label.setFixedWidth(35)
            slider.valueChanged.connect(
                lambda val, lbl=op_label: lbl.setText(f"{val}%")
            )
            row_lo.addWidget(op_label)

            btn_del = QtWidgets.QPushButton("删除")
            btn_del.setFixedWidth(60)
            btn_del.setStyleSheet(
                "QPushButton { color: #c00; } QPushButton:hover { font-weight: bold; }"
            )
            btn_del.clicked.connect(lambda checked, n=name: self._delete_clip(n))
            row_lo.addWidget(btn_del)

            self._list_layout.addWidget(row)
            self._row_widgets[name] = row

        self._list_layout.addStretch()

    def _delete_clip(self, name):
        """Remove a clip layer and delete its row."""
        reply = QtWidgets.QMessageBox.question(
            self, "确认删除",
            f"确定要删除图层「{name}」吗？\n此操作不可撤销。",
            QtWidgets.QMessageBox.Yes | QtWidgets.QMessageBox.No,
        )
        if reply != QtWidgets.QMessageBox.Yes:
            return

        self._remove_fn(name)
        self._scene_objects.pop(name, None)
        self._rebuild_items()

    @staticmethod
    def _resolve_vtk_opacity(actor):
        """Extract opacity from a VTK actor (or assembly by walking children)."""
        if isinstance(actor, vtkActor):
            return actor.GetProperty().GetOpacity()
        if hasattr(actor, "GetParts"):
            it = actor.GetParts()
            it.InitTraversal()
            while True:
                part = it.GetNextPartAsObject()
                if part is None:
                    break
                if isinstance(part, vtkActor):
                    return part.GetProperty().GetOpacity()
        return 1.0
