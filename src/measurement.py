"""
Distance and angle measurement tools.

All graphics (lines + labels) stay on screen after each measurement so the user
can accumulate multiple measurements.  Call ``clear_all()`` or ``reset()`` to
remove them.
"""

import numpy as np
import pyvista as pv


class MeasurementTool:
    """Manages distance / angle measurement graphics in the 3D viewport."""

    def __init__(self, plotter):
        self._plotter = plotter
        self.mode = "distance"       # "distance" | "angle"
        self._pending = []           # points collected for current measurement
        self._actors = []            # all drawn actors (lines + labels)
        self._points = []            # all placed points for redraw
        self._groups = []            # list of lists: each measurement's actor indices

    # ── public API ─────────────────────────────

    def set_mode(self, mode):
        """Switch measurement mode (``"distance"`` or ``"angle"``)."""
        assert mode in ("distance", "angle")
        self.mode = mode
        self._pending.clear()

    on_measurement = None

    def add_point(self, world_pos):
        """Feed a 3D point into the current measurement workflow."""
        self._points.append(np.asarray(world_pos))
        self._pending.append(np.asarray(world_pos))

        if self.mode == "distance" and len(self._pending) >= 2:
            start_idx = len(self._actors)
            self._draw_distance(self._pending[-2], self._pending[-1])
            self._groups.append(list(range(start_idx, len(self._actors))))
            self._pending = []

        elif self.mode == "angle" and len(self._pending) >= 3:
            start_idx = len(self._actors)
            self._draw_angle(self._pending[-3], self._pending[-2],
                             self._pending[-1])
            self._groups.append(list(range(start_idx, len(self._actors))))
            self._pending = []

    def undo_last(self):
        """Remove the most recent measurement from the scene (ID-8)."""
        if not self._groups:
            return
        indices = self._groups.pop()
        # Remove in reverse to keep indices valid
        for idx in reversed(indices):
            if idx < len(self._actors):
                actor = self._actors.pop(idx)
                self._plotter.remove_actor(actor)
        # Rebuild _points from remaining actors (approximate: keep all points)
        # Since we don't know which points belonged to the removed group,
        # just keep _points as-is. The next measurement will still work.
        self._pending.clear()

    def clear_all(self):
        """Remove all measurement graphics from the scene."""
        for a in self._actors:
            self._plotter.remove_actor(a)
        self._actors.clear()
        self._groups.clear()
        self._pending.clear()
        self._points.clear()

    def reset(self):
        """Alias for clear_all()."""
        self.clear_all()

    # ── drawing helpers ────────────────────────

    def _draw_distance(self, p1, p2):
        line = pv.Line(p1, p2)
        actor = self._plotter.add_mesh(line, color="red", line_width=3)
        self._actors.append(actor)

        dist = float(np.linalg.norm(p2 - p1))
        mid = (p1 + p2) * 0.5
        label_actor = self._add_3d_label(mid, f"d = {dist:.2f}", color="red")
        self._actors.append(label_actor)

        pts = pv.PolyData(np.array([p1, p2]))
        pt_actor = self._plotter.add_points(
            pts, color="red", point_size=8, render_points_as_spheres=True,
        )
        self._actors.append(pt_actor)

        if callable(self.on_measurement):
            self.on_measurement(f"测距：{dist:.2f} 米")

    def _draw_angle(self, p1, p2, p3):
        line1 = pv.Line(p2, p1)
        line2 = pv.Line(p2, p3)
        a1 = self._plotter.add_mesh(line1, color="orange", line_width=3)
        a2 = self._plotter.add_mesh(line2, color="orange", line_width=3)
        self._actors.append(a1)
        self._actors.append(a2)

        v1 = p1 - p2
        v2 = p3 - p2
        cosang = np.dot(v1, v2) / (np.linalg.norm(v1) * np.linalg.norm(v2))
        angle_deg = float(np.degrees(np.arccos(np.clip(cosang, -1.0, 1.0))))

        label_actor = self._add_3d_label(p2, f"angle = {angle_deg:.1f}°",
                                         color="orange")
        self._actors.append(label_actor)

        vtx = pv.PolyData(np.array([p2]))
        v_actor = self._plotter.add_points(
            vtx, color="orange", point_size=10, render_points_as_spheres=True,
        )
        self._actors.append(v_actor)

        if callable(self.on_measurement):
            self.on_measurement(f"测角：{angle_deg:.1f}°")

    def _add_3d_label(self, position, text, color="red"):
        """Create a floating 3D text label at *position*.

        Uses PyVista's ``add_point_labels`` internally.  The point itself is
        hidden so only the text + optional leader line is visible.
        """
        pts = pv.PolyData(np.array([position]))
        actor = self._plotter.add_point_labels(
            pts,
            [text],
            show_points=False,
            point_size=0,
            font_size=14,
            text_color=color,
            shape=None,          # no background shape → transparent
            always_visible=True,
        )
        return actor
