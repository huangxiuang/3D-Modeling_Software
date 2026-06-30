"""
Configuration constants, coordinate system definitions, and config persistence.
"""

import os
import json

# ──────────────────────────────────────────────
# Default application configuration
# ──────────────────────────────────────────────
DEFAULT_CONFIG = {
    "bg_color": [0.82, 0.90, 1.0],
    "terrain_cmap": "terrain",
    "elevation_scale": 1.0,
    "water_level": 0.0,
    "coordinate_system": "ENU",
    "show_grid": True,
    "show_axes": True,
}

# Config file path next to the entry-point script
CFG_FILENAME = "3dscene_config.json"


def _cfg_path():
    """Resolve config file path relative to this source file's location."""
    return os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), CFG_FILENAME)


# ──────────────────────────────────────────────
# Coordinate system definitions
# ──────────────────────────────────────────────
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


# ──────────────────────────────────────────────
# Config persistence
# ──────────────────────────────────────────────

def load_config():
    """Load config from disk, merging with defaults.  Silent on missing / corrupt file."""
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
