"""
Default 3D scene factory — builds terrain, river, vegetation, aircraft,
plus a bird, tree, and second aircraft.

Each object is returned as a plain dict:

    {
        "mesh":       pv.PolyData | pv.StructuredGrid,
        "type":       "mesh" | "points",
        "visible":    bool,
        "params":     dict,              # passed to add_mesh / add_points
        "extra":      dict | None,       # tool-specific metadata
        "name":       str,
    }
"""

import numpy as np
import pyvista as pv


# ═══════════════════════════════════════════════════════════════

def build_default_scene():
    """Return a dict ``{name: scene_object, ...}`` describing the initial scene."""

    actors = {}

    # ──────────────────────────────────────────────
    #  1.  Terrain  (60 × 60 grid, gaussian hills + river channel)
    # ──────────────────────────────────────────────
    res_x, res_y = 60, 60
    xs = np.linspace(-10, 10, res_x)
    ys = np.linspace(-10, 10, res_y)
    X, Y = np.meshgrid(xs, ys)

    Z = (
        5.0 * np.exp(-((X - 4) ** 2 + (Y - 3) ** 2) / 12)
        + 3.5 * np.exp(-((X + 3) ** 2 + (Y - 4) ** 2) / 9)
        + 2.0 * np.exp(-((X - 1) ** 2 + (Y + 2) ** 2) / 15)
    )

    # Carve a river channel
    river_width = 1.2
    river_depth = 0.7
    for i in range(res_x):
        for j in range(res_y):
            dist = abs(X[i, j] - 3.0 * np.sin(Y[i, j] * 0.4))
            if dist < river_width:
                t = dist / river_width
                Z[i, j] = Z[i, j] * (0.05 + 0.15 * t) - river_depth * (1 - t)

    grid = pv.StructuredGrid(X, Y, Z)
    grid["elevation"] = Z.flatten(order="F")

    # ── Base terrain — flat light-yellow foundation ──
    # Always visible. Uses a uniform colour (no elevation scalar mapping).
    # Thresholded overlay layers (sand/grass/earth) sit on top with
    # independent opacity control; when an overlay is transparent the
    # base colour shows through.
    z_min, z_max = float(Z.min()), float(Z.max())
    actors["terrain"] = {
        "mesh": grid,
        "type": "mesh",
        "visible": True,
        "params": {
            "color": "#f2e2a8",
            "smooth_shading": True,
            "opacity": 1.0,
        },
        "extra": {
            "original_z": Z.copy(),
            "X": X,
            "Y": Y,
        },
        "name": "terrain",
    }

    # ── Overlay layers (ID-18): sand/grass/earth on top of base terrain ──
    for name, obj in build_terrain_layer_meshes(grid, Z).items():
        actors[name] = obj

    # ──────────────────────────────────────────────
    #  2.  River  (flat surface following a sine path)
    # ──────────────────────────────────────────────
    n_y, n_w = 100, 12
    river_y = np.linspace(-10, 10, n_y)
    river_w = np.linspace(-0.8, 0.8, n_w)
    Ry, Rw = np.meshgrid(river_y, river_w)
    Rx_center = 3.0 * np.sin(Ry * 0.4)
    Rx = Rx_center + Rw
    Rz_base = np.full_like(Rx, 0.0)

    river_grid = pv.StructuredGrid(Rx, Ry, Rz_base)

    actors["river"] = {
        "mesh": river_grid,
        "type": "mesh",
        "visible": True,
        "params": {
            "color": "#1488cc",
            "opacity": 0.88,
            "smooth_shading": True,
        },
        "extra": {
            "Ry": Ry,
            "Rz_base": Rz_base,
            "phase": 0.0,
        },
        "name": "river",
    }

    # ──────────────────────────────────────────────
    #  3.  Vegetation  (point sprites along both banks)
    # ──────────────────────────────────────────────
    bank_n = 100
    bank_y = np.linspace(-10, 10, bank_n)
    bank_x_left = 3.0 * np.sin(bank_y * 0.4) - 0.88
    bank_x_right = 3.0 * np.sin(bank_y * 0.4) + 0.88
    bank_z = np.full_like(bank_y, -0.15)

    left_pts = pv.PolyData(np.column_stack((bank_x_left, bank_y, bank_z)))
    right_pts = pv.PolyData(np.column_stack((bank_x_right, bank_y, bank_z)))
    merged = left_pts.merge(right_pts)

    actors["vegetation"] = {
        "mesh": merged,
        "type": "points",
        "visible": True,
        "params": {
            "color": "#2d882d",
            "point_size": 10,
            "opacity": 0.8,
        },
        "extra": None,
        "name": "vegetation",
    }

    # ──────────────────────────────────────────────
    #  4.  Aircraft  (grey jet)
    # ──────────────────────────────────────────────
    fuselage = pv.Cylinder(
        center=(0, 0, 0), direction=(1, 0, 0), radius=0.16, height=1.6
    )
    nose = pv.Cone(
        center=(0.8 + 0.25, 0, 0), direction=(1, 0, 0),
        height=0.5, radius=0.16,
    )
    wing_left = pv.Box(bounds=(-0.3, 0.3, -0.9, 0, -0.025, 0.025))
    wing_right = pv.Box(bounds=(-0.3, 0.3, 0, 0.9, -0.025, 0.025))
    wing_left.translate((0, -0.45, 0), inplace=True)
    wing_right.translate((0, 0.45, 0), inplace=True)
    ht_left = pv.Box(bounds=(-0.125, 0.125, -0.4, 0, -0.015, 0.015))
    ht_right = pv.Box(bounds=(-0.125, 0.125, 0, 0.4, -0.015, 0.015))
    ht_left.translate((-0.8 + 0.083, -0.2, 0), inplace=True)
    ht_right.translate((-0.8 + 0.083, 0.2, 0), inplace=True)
    vt = pv.Box(bounds=(-0.15 - 0.8, -0.8, -0.015, 0.015, 0, 0.35))
    nozzle = pv.Cylinder(
        center=(-0.8, 0, 0), direction=(-1, 0, 0), radius=0.14, height=0.03,
    )

    airplane = fuselage.merge([nose, wing_left, wing_right,
                               ht_left, ht_right, vt, nozzle])
    airplane.scale(1.5, inplace=True)
    airplane.translate((-6, -2, 8), inplace=True)

    actors["aircraft"] = {
        "mesh": airplane,
        "type": "mesh",
        "visible": True,
        "params": {
            "color": "#c4c4c4",
            "smooth_shading": True,
            "ambient": 0.35,
            "diffuse": 0.75,
            "specular": 0.6,
            "specular_power": 40,
        },
        "extra": None,
        "name": "aircraft",
    }

    # ──────────────────────────────────────────────
    #  5.  Aircraft 2  (red escort jet)
    # ──────────────────────────────────────────────
    # Reuse the same geometry, different position & colour
    fuselage2 = pv.Cylinder(
        center=(0, 0, 0), direction=(1, 0, 0), radius=0.16, height=1.6
    )
    nose2 = pv.Cone(
        center=(0.8 + 0.25, 0, 0), direction=(1, 0, 0),
        height=0.5, radius=0.16,
    )
    wing_l2 = pv.Box(bounds=(-0.3, 0.3, -0.9, 0, -0.025, 0.025))
    wing_r2 = pv.Box(bounds=(-0.3, 0.3, 0, 0.9, -0.025, 0.025))
    wing_l2.translate((0, -0.45, 0), inplace=True)
    wing_r2.translate((0, 0.45, 0), inplace=True)
    ht_l2 = pv.Box(bounds=(-0.125, 0.125, -0.4, 0, -0.015, 0.015))
    ht_r2 = pv.Box(bounds=(-0.125, 0.125, 0, 0.4, -0.015, 0.015))
    ht_l2.translate((-0.8 + 0.083, -0.2, 0), inplace=True)
    ht_r2.translate((-0.8 + 0.083, 0.2, 0), inplace=True)
    vt2 = pv.Box(bounds=(-0.15 - 0.8, -0.8, -0.015, 0.015, 0, 0.35))
    nozzle2 = pv.Cylinder(
        center=(-0.8, 0, 0), direction=(-1, 0, 0), radius=0.14, height=0.03,
    )

    airplane2 = fuselage2.merge([nose2, wing_l2, wing_r2,
                                 ht_l2, ht_r2, vt2, nozzle2])
    airplane2.scale(1.5, inplace=True)
    airplane2.translate((-6, 3, 7.5), inplace=True)

    actors["aircraft2"] = {
        "mesh": airplane2,
        "type": "mesh",
        "visible": True,
        "params": {
            "color": "#cc3333",
            "smooth_shading": True,
            "ambient": 0.35,
            "diffuse": 0.75,
            "specular": 0.6,
            "specular_power": 40,
        },
        "extra": None,
        "name": "aircraft2",
    }

    # ──────────────────────────────────────────────
    #  6.  Bird  (sphere body + cone wings)
    # ──────────────────────────────────────────────
    body = pv.Sphere(radius=0.12, center=(0, 0, 0))
    head = pv.Sphere(radius=0.06, center=(0.15, 0, 0.04))
    # Wings — thin cones angled slightly
    lw = pv.Cone(center=(0, -0.15, 0.02), direction=(0, -1, 0.2),
                 height=0.18, radius=0.06)
    rw = pv.Cone(center=(0, 0.15, 0.02), direction=(0, 1, 0.2),
                 height=0.18, radius=0.06)
    tail = pv.Cone(center=(-0.12, 0, 0.02), direction=(-1, 0, 0.1),
                   height=0.08, radius=0.04)

    bird = body.merge([head, lw, rw, tail])
    bird.translate((5, -1, 7), inplace=True)

    actors["bird"] = {
        "mesh": bird,
        "type": "mesh",
        "visible": True,
        "params": {
            "color": "#e8c36a",
            "smooth_shading": True,
        },
        "extra": None,
        "name": "bird",
    }

    # ──────────────────────────────────────────────
    #  7.  Tree  (cylinder trunk + cone leaves)
    # ──────────────────────────────────────────────
    trunk = pv.Cylinder(center=(0, 0, -0.25), direction=(0, 0, 1),
                        radius=0.06, height=0.5)
    leaves = pv.Cone(center=(0, 0, 0.15), direction=(0, 0, 1),
                     height=0.5, radius=0.2)
    trunk.points[:, :2] *= 0.5  # flatten trunk cross-section slightly
    tree_mesh = trunk.merge(leaves)
    # Compute terrain Z at (4.5, 2) so the tree rests on the surface
    # Offset by 0.5 so the trunk bottom (local z=-0.5) sits at terrain level
    tree_x, tree_y = 4.5, 2.0
    terrain_z = (
        5.0 * np.exp(-((tree_x - 4) ** 2 + (tree_y - 3) ** 2) / 12)
        + 3.5 * np.exp(-((tree_x + 3) ** 2 + (tree_y - 4) ** 2) / 9)
        + 2.0 * np.exp(-((tree_x - 1) ** 2 + (tree_y + 2) ** 2) / 15)
    )
    tree_mesh.translate((tree_x, tree_y, terrain_z + 0.5), inplace=True)

    actors["tree"] = {
        "mesh": tree_mesh,
        "type": "mesh",
        "visible": True,
        "params": {
            "color": "#5a8f3c",
            "smooth_shading": True,
        },
        "extra": None,
        "name": "tree",
    }

    return actors


# ═══════════════════════════════════════════════════════════════
#  Terrain layer meshes (ID-18)
# ═══════════════════════════════════════════════════════════════

def build_terrain_layer_meshes(grid, Z):
    """Create 3 elevation-thresholded terrain layers (sand/grass/earth).

    Each layer is extracted from *grid* surface by thresholding the
    ``"elevation"`` scalar.  The returned dict has the same shape as a
    ``build_default_scene`` entry and is typically merged into the actors
    dict::

        actors["layer_sand"]   — 低处沙地  (yellow gradient)
        actors["layer_grass"]  — 中部草地  (green gradient)
        actors["layer_earth"]  — 高处土地  (brown gradient)
    """
    surface = grid.extract_surface()
    z_min, z_max = float(Z.min()), float(Z.max())
    elev_range = z_max - z_min

    # Elevation thresholds for 3 bands — mountain tops (top ~25%) have NO overlay
    sand_max = z_min + elev_range * 0.20
    grass_max = z_min + elev_range * 0.45
    earth_max = z_min + elev_range * 0.70

    # Slight overlap to prevent visible gaps at band boundaries
    eps = 0.02

    sand_mesh = surface.threshold(
        [z_min - 1.0, sand_max + eps],
        scalars="elevation",
        preference="point",
    )
    grass_mesh = surface.threshold(
        [sand_max - eps, grass_max + eps],
        scalars="elevation",
        preference="point",
    )
    earth_mesh = surface.threshold(
        [grass_max - eps, earth_max + eps],
        scalars="elevation",
        preference="point",
    )

    return {
        "layer_sand": {
            "mesh": sand_mesh,
            "type": "mesh",
            "visible": False,
            "params": {
                "scalars": "elevation",
                "cmap": ["#f5e6b8", "#e8c76a", "#d4a843"],
                "clim": [z_min, sand_max],
                "smooth_shading": True,
                "opacity": 1.0,
                "show_scalar_bar": False,
            },
            "extra": None,
            "name": "layer_sand",
        },
        "layer_grass": {
            "mesh": grass_mesh,
            "type": "mesh",
            "visible": False,
            "params": {
                "scalars": "elevation",
                "cmap": ["#a8d5a2", "#5a9e4c", "#2d6b28"],
                "clim": [sand_max, grass_max],
                "smooth_shading": True,
                "opacity": 1.0,
                "show_scalar_bar": False,
            },
            "extra": None,
            "name": "layer_grass",
        },
        "layer_earth": {
            "mesh": earth_mesh,
            "type": "mesh",
            "visible": False,
            "params": {
                "scalars": "elevation",
                "cmap": ["#d4b896", "#8b6f47", "#5c4033"],
                "clim": [grass_max, earth_max],
                "smooth_shading": True,
                "opacity": 1.0,
                "show_scalar_bar": False,
            },
            "extra": None,
            "name": "layer_earth",
        },
    }
