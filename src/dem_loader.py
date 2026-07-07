"""
DEM (Digital Elevation Model) loader — import ASTER GDEM / SRTM .img files
into the 3D scene.

File format
-----------
The input is an HFA (ERDAS Imagine) / GeoTIFF .img file containing 16-bit
integer elevation values (metres).  No-data / void pixels are marked with
the value 32767 (ASTER GDEM convention).

Processing pipeline
-------------------
    1. Open with rasterio, read band 1
    2. Filter: replace void (≥ 32767) and outlier (< -50) values with 0
    3. Downsample (decimate) by *step* in both dimensions
    4. Centre the XY grid around the origin so the DEM sits at (0,0)
    5. Build a ``pyvista.StructuredGrid`` with real-world elevation as Z
    6. Optionally apply vertical exaggeration

Usage
-----
::

    from src.dem_loader import load_dem, build_dem_scene

    dem = load_dem("ASTGTM_N46E129H.img", step=2)
    scene_objects = build_dem_scene(dem, vert_exag=2.0, aircraft_scale=500)
"""

import numpy as np
import pyvista as pv

import sys as _sys

try:
    import rasterio
    HAS_RASTERIO = True
except ImportError:
    HAS_RASTERIO = False


# ═══════════════════════════════════════════════════════════════
#  Constants
# ═══════════════════════════════════════════════════════════════

# Default downsampling factor (every Nth pixel)
DEFAULT_STEP = 2

# ASTER GDEM void / no-data sentinel (32767), and lower outlier bound
ASTER_NODATA = 32767
OUTLIER_MIN = -50
OUTLIER_MAX = 9000  # values above this are also suspect

# Aircraft scale factor research:
#   DEM terrain extent ~ 136 km diagonal.  The default aircraft (~2.4 m
#   long at 1.5×) is invisible at this scale.  At 500× the aircraft is
#   ~1.2 km long, which is clearly visible when the full terrain is in
#   view (~11 px at 1296 px viewport width).  Scale factor chosen: 500×
#   — enough to be unmistakably an aircraft shape without dominating the
#   scene.
AIRCRAFT_DEFAULT_SCALE = 2000

# Z exaggeration for visual clarity (mountains visible without absurd peaks)
DEFAULT_VERT_EXAG = 2.0


# ═══════════════════════════════════════════════════════════════
#  Public API
# ═══════════════════════════════════════════════════════════════

def load_dem(filepath, step=DEFAULT_STEP):
    """Load, clean, and decimate a DEM .img file.

    Parameters
    ----------
    filepath : str
        Path to the .img file.
    step : int
        Downsampling stride (default 2 → every 2nd pixel).

    Returns
    -------
    dict with keys:
        ``z``        : 2-D np.ndarray (float32) of elevation values
        ``x``        : 1-D np.ndarray (float32) X coordinates (centred)
        ``y``        : 1-D np.ndarray (float32) Y coordinates (centred)
        ``xx``       : 2-D meshgrid of X
        ``yy``       : 2-D meshgrid of Y
        ``rows``     : int
        ``cols``     : int
        ``pixel_size`` : float  source pixel size in metres × step
        ``crs``      : str | None  source CRS (e.g. "EPSG:32652")

    Raises
    ------
    ImportError
        If ``rasterio`` is not installed.
    FileNotFoundError
        If *filepath* does not exist.
    ValueError
        If the file cannot be read as a valid DEM.
    """
    if not HAS_RASTERIO:
        raise ImportError(
            "rasterio is required to load DEM .img files.\n"
            "Install with:  pip install rasterio"
        )

    with rasterio.open(filepath) as src:
        data = src.read(1).astype(np.float32)
        crs = str(src.crs) if src.crs else None
        pixel_size_raw = abs(src.transform[0])  # pixel width in metres
        # Height (rows) and width (cols) of the full-resolution grid
        full_rows, full_cols = data.shape

    # ── Filter voids & outliers ──────────────────────────
    data[data >= ASTER_NODATA] = 0.0
    data[data < OUTLIER_MIN] = 0.0
    data[data > OUTLIER_MAX] = 0.0

    # ── Downsample ───────────────────────────────────────
    data = data[::step, ::step]
    rows, cols = data.shape
    pixel_size = pixel_size_raw * step

    # ── Build XY grid (centred around origin) ────────────
    x_raw = np.arange(cols, dtype=np.float32) * pixel_size
    y_raw = np.arange(rows, dtype=np.float32) * pixel_size

    # Centre
    x_centre = (x_raw[-1] + x_raw[0]) * 0.5
    y_centre = (y_raw[-1] + y_raw[0]) * 0.5
    x = x_raw - x_centre
    y = y_raw - y_centre

    xx, yy = np.meshgrid(x, y)

    return {
        "z": data,
        "x": x,
        "y": y,
        "xx": xx,
        "yy": yy,
        "rows": rows,
        "cols": cols,
        "pixel_size": pixel_size,
        "crs": crs,
        "full_resolution": (full_rows, full_cols),
        "source": filepath,
    }


def dem_to_mesh(dem_data, vert_exag=DEFAULT_VERT_EXAG):
    """Convert a ``load_dem`` output dict to a ``pyvista.StructuredGrid``.

    Parameters
    ----------
    dem_data : dict
        Output of :func:`load_dem`.
    vert_exag : float
        Vertical exaggeration multiplier (default 2.0).

    Returns
    -------
    pv.StructuredGrid
    """
    z = dem_data["z"] * vert_exag
    xx = dem_data["xx"]
    yy = dem_data["yy"]
    rows = dem_data["rows"]
    cols = dem_data["cols"]

    points = np.column_stack((xx.ravel(), yy.ravel(), z.ravel()))
    grid = pv.StructuredGrid()
    grid.points = points.astype(np.float32)
    grid.dimensions = (cols, rows, 1)
    grid["elevation"] = z.ravel()

    return grid


def build_dem_scene(
    dem_data,
    vert_exag=DEFAULT_VERT_EXAG,
    aircraft_scale=AIRCRAFT_DEFAULT_SCALE,
    aircraft_z=7000.0,
):
    """Build a full scene objects dict from DEM data.

    Returns the same structure as ``scene_builder.build_default_scene()``:
    ``{name: {mesh, type, visible, params, extra, name}, ...}``.

    Objects returned
    - ``"terrain"``       — DEM surface
    - ``"aircraft"``      — grey jet at Z=7000 m, scaled up
    - ``"aircraft2"``     — red escort jet at Z=7000 m, scaled up
    """
    grid = dem_to_mesh(dem_data, vert_exag)

    actors = {}

    # ── 1. Base terrain ──────────────────────────────────
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
            "original_z": dem_data["z"].copy(),
            "X": dem_data["xx"],
            "Y": dem_data["yy"],
        },
        "name": "terrain",
    }

    # ── 2. Aircraft (grey jet) at Z=7000 ─────────────────
    aircraft_z_pos = aircraft_z
    # Place aircraft slightly off-centre so it doesn't obscure the view
    ac_pos = (dem_data["xx"].max() * 0.15,
              dem_data["yy"].max() * 0.15,
              aircraft_z_pos)

    airplane1 = _build_airplane_mesh()
    airplane1.scale(aircraft_scale, inplace=True)
    airplane1.translate(ac_pos, inplace=True)

    actors["aircraft"] = {
        "mesh": airplane1,
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

    # ── 3. Aircraft 2 (red escort) ──────────────────────
    ac2_pos = (dem_data["xx"].max() * 0.15 + aircraft_scale * 0.002,
               dem_data["yy"].max() * 0.15 - aircraft_scale * 0.0015,
               aircraft_z_pos)

    airplane2 = _build_airplane_mesh()
    airplane2.scale(aircraft_scale, inplace=True)
    airplane2.translate(ac2_pos, inplace=True)

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

    return actors


# ═══════════════════════════════════════════════════════════════
#  Helpers
# ═══════════════════════════════════════════════════════════════

def _build_airplane_mesh():
    """Return a merged PolyData jet (same geometry as scene_builder)."""
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
        center=(-0.8, 0, 0), direction=(-1, 0, 0),
        radius=0.14, height=0.03,
    )

    return fuselage.merge([nose, wing_left, wing_right,
                           ht_left, ht_right, vt, nozzle])
