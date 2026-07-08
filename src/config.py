"""
Configuration constants, coordinate system definitions, and config persistence.

All magic numbers live here — no hardcoded values in business logic.
"""

import os
import json

# ═══════════════════════════════════════════════════════════════
#  Magic-number elimination — every hardcoded constant lives here
# ═══════════════════════════════════════════════════════════════

# ── DEM import defaults ──────────────────────────────
DEM_DEFAULT_STEP = 2                 # downsample stride for load_dem()
DEM_DEFAULT_VERT_EXAG = 2.0         # default Z exaggeration
DEM_VERT_EXAG_MIN = 0.1
DEM_VERT_EXAG_MAX = 20.0
DEM_AIRCRAFT_Z = 7000.0             # aircraft altitude in DEM scenes (m)

# ── Default scene ────────────────────────────────────
DEFAULT_SCENE_SIZE = 10.0           # default terrain X/Y half-span
DEFAULT_SCENE_Z_RANGE = 5.0         # default terrain Z range
DEFAULT_GROUND_PLANE_SIZE = 12.0    # default ground plane when no terrain loaded

# ── Aircraft ─────────────────────────────────────────
AIRCRAFT_MIN_SCALE = 0.1
AIRCRAFT_MAX_SCALE = 5.0
AIRCRAFT_DEFAULT_SCALE_DEM = 2000   # scale in DEM scenes (matching dem_loader)
AIRCRAFT_DEFAULT_SCALE_DEFAULT = 1.0  # scale in default scene

# ── Camera defaults ──────────────────────────────────
DEFAULT_CAMERA_POSITION = (18.0, -16.0, 8.0)
DEFAULT_CAMERA_FOCAL = (0.0, 0.0, 1.5)
DEFAULT_CAMERA_UP = (0.0, 0.0, 1.0)

# ── Slider ranges (default scene) ────────────────────
DEFAULT_SLIDER_RANGE_XY = 15.0
DEFAULT_SLIDER_RANGE_Z = 15.0
DEM_SLIDER_Z_MIN = -500.0
DEM_SLIDER_Z_MAX = 10000.0

# ── Flight animation ─────────────────────────────────
FLIGHT_CRUISE_SPEED = 5.0           # units per second
FLIGHT_TIMER_INTERVAL = 50          # ms per tick
FLIGHT_PITCH_FACTOR = 0.35          # speed reduction per radian of pitch
FLIGHT_TURN_FACTOR = 0.006          # speed reduction per deg/s yaw rate
FLIGHT_MIN_SPEED_FACTOR = 0.30      # minimum speed as fraction of cruise

# ── Formation flight ─────────────────────────────────
FORMATION_TRAIL_DISTANCE = 0.003    # trail distance multiplier
FORMATION_TRAIL_DISTANCE_DEM = 0.005  # DEM scene multiplier

# ── Waypoint ─────────────────────────────────────────
WP_SPHERE_RADIUS = 0.15
WP_LABEL_FONT_SIZE = 14

# ── Measurement ──────────────────────────────────────
MEAS_POINT_SIZE = 8
MEAS_LINE_WIDTH = 3
MEAS_FONT_SIZE = 14

# ── Pickers ──────────────────────────────────────────
CELL_PICKER_TOLERANCE = 0.001

# ── Transform log de-bounce ──────────────────────────
TRANSFORM_LOG_DEBOUNCE_MS = 1500

# ── Recording ────────────────────────────────────────
RECORDING_DIR = "screenshots"

# ── Data persistence ─────────────────────────────────
SAVE_DIR_AIRCRAFT = "data/aircraft"
SAVE_DIR_TERRAIN = "data/terrain"
SAVE_DIR_FLIGHT = "data/flight"
LOG_FILENAME = "操作日志"           # exported log filename prefix

# ── UI ───────────────────────────────────────────────
SCENE_SETTINGS_REFRESH_INTERVAL = 1000  # ms for SceneSettingsDialog timer

# ═══════════════════════════════════════════════════════════════
#  Default application configuration
# ═══════════════════════════════════════════════════════════════

DEFAULT_CONFIG = {
    "bg_color": [0.82, 0.90, 1.0],
    "terrain_cmap": "terrain",
    "elevation_scale": DEM_DEFAULT_VERT_EXAG,
    "water_level": 0.0,
    "coordinate_system": "ENU",
    "show_grid": True,
    "show_axes": True,
    "dem_source_path": "",          # last DEM file path
}

# Config file path next to the entry-point script
CFG_FILENAME = "3dscene_config.json"


def _cfg_path():
    """Resolve config file path relative to this source file's location."""
    return os.path.join(
        os.path.dirname(os.path.dirname(os.path.abspath(__file__))),
        CFG_FILENAME,
    )


# ═══════════════════════════════════════════════════════════════
#  Coordinate system definitions
# ═══════════════════════════════════════════════════════════════

COORD_SYSTEMS = {
    "ENU": {
        "label": "东-北-天",
        "axes": ["X: 东 (East)", "Y: 北 (North)", "Z: 天 (Up)"],
    },
    "FLU": {
        "label": "前-左-上",
        "axes": ["X: 前 (Front)", "Y: 左 (Left)", "Z: 上 (Up)"],
    },
    "NED": {
        "label": "北-东-地",
        "axes": ["X: 北 (North)", "Y: 东 (East)", "Z: 地 (Down)"],
    },
    "NWU": {
        "label": "北-西-天",
        "axes": ["X: 北 (North)", "Y: 西 (West)", "Z: 天 (Up)"],
    },
}

COORD_SYSTEM_NAMES = list(COORD_SYSTEMS.keys())


def world_to_coord_str(cs, x, y, z):
    """Format a world-coordinate triplet according to the named coordinate system."""
    labels = COORD_SYSTEMS[cs]["axes"]
    return f"{labels[0]}={x:.2f},  {labels[1]}={y:.2f},  {labels[2]}={z:.2f}"


# ═══════════════════════════════════════════════════════════════
#  Config persistence
# ═══════════════════════════════════════════════════════════════

def load_config():
    """Load config from disk, merging with defaults. Silent on missing / corrupt file."""
    config = DEFAULT_CONFIG.copy()
    path = _cfg_path()
    if os.path.exists(path):
        try:
            with open(path) as f:
                config.update(json.load(f))
        except Exception:
            pass
    return config


def save_config(config):
    """Write config dict to disk (best-effort)."""
    path = _cfg_path()
    try:
        with open(path, "w") as f:
            json.dump(config, f, indent=2)
    except Exception:
        pass
