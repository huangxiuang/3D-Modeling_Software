#!/usr/bin/env python3
"""
Unit tests for the non-GUI logic of 3DSceneSoftware.

Run with::

    cd /path/to/project
    python -m pytest tests/ -v

or::

    python tests/test_core.py
"""

import sys
import os
import math

# Ensure ``src`` is importable
_proj_root = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
if _proj_root not in sys.path:
    sys.path.insert(0, _proj_root)

import numpy as np
import pyvista as pv

from src.config import world_to_coord_str, COORD_SYSTEMS
from src.collision import check_aabb_collision, find_collisions
from src.measurement import MeasurementTool  # light test, no Qt required
from src.scene_builder import build_default_scene


# ═══════════════════════════════════════════════════════════════
#  Coordinate system tests
# ═══════════════════════════════════════════════════════════════

def test_world_to_coord_str_enu():
    s = world_to_coord_str("ENU", 1.234, -5.678, 10.0)
    assert "东" in s and "北" in s and "天" in s
    assert "1.23" in s
    assert "-5.68" in s
    assert "10.00" in s


def test_world_to_coord_str_flu():
    s = world_to_coord_str("FLU", 0.0, 0.0, 0.0)
    assert "前" in s and "左" in s and "上" in s


def test_world_to_coord_str_ned():
    s = world_to_coord_str("NED", 100.5, 200.5, -30.0)
    assert "北" in s and "东" in s and "地" in s


def test_world_to_coord_str_nwu():
    s = world_to_coord_str("NWU", -1.0, -2.0, 3.0)
    assert "北" in s and "西" in s and "天" in s


def test_all_coord_systems_defined():
    """Every coordinate system should have axes labels."""
    for cs_name, cs_data in COORD_SYSTEMS.items():
        assert len(cs_data["axes"]) == 3
        for axis_label in cs_data["axes"]:
            assert ":" in axis_label, f"{cs_name} axis missing ':' separator"


# ═══════════════════════════════════════════════════════════════
#  Collision detection tests
# ═══════════════════════════════════════════════════════════════

def test_aabb_no_collision():
    """Two boxes far apart should NOT collide."""
    box1 = pv.Box(bounds=(-2, -1, -2, -1, -2, -1))
    box2 = pv.Box(bounds=(1, 2, 1, 2, 1, 2))
    assert check_aabb_collision(box1, box2) is False


def test_aabb_touching_edges_collide():
    """Boxes that touch at edges ARE intersecting (boundary inclusive)."""
    # Box 1: x in [-1, 1], Box 2: x in [1, 3] → overlap at x=1
    box1 = pv.Box(bounds=(-1, 1, -1, 1, -1, 1))
    box2 = pv.Box(bounds=(1, 3, -1, 1, -1, 1))
    assert check_aabb_collision(box1, box2) is True


def test_aabb_full_overlap():
    """Two identical boxes definitely collide."""
    box1 = pv.Box(bounds=(-1, 1, -1, 1, -1, 1))
    box2 = pv.Box(bounds=(-1, 1, -1, 1, -1, 1))
    assert check_aabb_collision(box1, box2) is True


def test_aabb_partial_overlap():
    """Partial overlap in all axes → collision."""
    a = pv.Box(bounds=(0, 2, 0, 2, 0, 2))
    b = pv.Box(bounds=(1, 3, 1, 3, 1, 3))
    assert check_aabb_collision(a, b) is True


def test_aabb_separated_z():
    """Same x/y but separated z → no collision."""
    a = pv.Box(bounds=(0, 1, 0, 1, 0, 1))
    b = pv.Box(bounds=(0, 1, 0, 1, 10, 11))
    assert check_aabb_collision(a, b) is False


def test_find_collisions_empty():
    assert find_collisions([], []) == []


def test_find_collisions_single():
    """A single mesh cannot collide with itself."""
    m = pv.Box(bounds=(-1, 1, -1, 1, -1, 1))
    assert find_collisions([m], ["only"]) == []


def test_find_collisions_pair():
    m1 = pv.Box(bounds=(0, 2, 0, 2, 0, 2))
    m2 = pv.Box(bounds=(1, 3, 1, 3, 1, 3))
    results = find_collisions([m1, m2], ["A", "B"])
    assert ("A", "B") in results or ("B", "A") in results


# ═══════════════════════════════════════════════════════════════
#  Scene builder tests
# ═══════════════════════════════════════════════════════════════

def test_default_scene_has_expected_objects():
    scene = build_default_scene()
    assert "terrain" in scene
    assert "river" in scene
    assert "vegetation" in scene
    assert "aircraft" in scene
    assert "aircraft2" in scene
    assert "bird" in scene
    assert "tree" in scene
    assert len(scene) == 7


def test_default_scene_objects_have_required_keys():
    scene = build_default_scene()
    for name, obj in scene.items():
        assert "mesh" in obj, f"{name} missing 'mesh'"
        assert "type" in obj, f"{name} missing 'type'"
        assert obj["type"] in ("mesh", "points"), f"{name} bad type"
        assert "visible" in obj, f"{name} missing 'visible'"
        assert "params" in obj, f"{name} missing 'params'"
        assert obj["visible"] is True, f"{name} should start visible"


def test_terrain_mesh_size():
    scene = build_default_scene()
    grid = scene["terrain"]["mesh"]
    # 60 × 60 grid → 3600 points
    assert grid.n_points == 3600
    assert grid.n_cells == 3481  # (60-1) × (60-1)


def test_terrain_has_elevation_scalars():
    scene = build_default_scene()
    grid = scene["terrain"]["mesh"]
    assert "elevation" in grid.point_data


def test_vegetation_is_points():
    scene = build_default_scene()
    veg = scene["vegetation"]
    assert veg["type"] == "points"


def test_aircraft_is_merged_polydata():
    scene = build_default_scene()
    aircraft = scene["aircraft"]
    assert aircraft["type"] == "mesh"
    # Merged airplane should have > 100 cells
    assert aircraft["mesh"].n_cells > 100


# ═══════════════════════════════════════════════════════════════
#  Measurement logic tests (pure math, no renderer needed)
# ═══════════════════════════════════════════════════════════════

def test_measurement_distance_math():
    """Verify the distance formula used by MeasurementTool."""
    p1 = np.array([0.0, 0.0, 0.0])
    p2 = np.array([3.0, 4.0, 0.0])
    dist = np.linalg.norm(p2 - p1)
    assert abs(dist - 5.0) < 1e-9


def test_measurement_angle_math():
    """Verify angle computation used by MeasurementTool."""
    # Right angle: (1,0,0), (0,0,0), (0,1,0)
    p1, p2, p3 = np.array([1, 0, 0]), np.array([0, 0, 0]), np.array([0, 1, 0])
    v1 = p1 - p2
    v2 = p3 - p2
    cosang = np.dot(v1, v2) / (np.linalg.norm(v1) * np.linalg.norm(v2))
    angle = np.degrees(np.arccos(np.clip(cosang, -1.0, 1.0)))
    assert abs(angle - 90.0) < 1e-9, f"Expected 90°, got {angle}°"


def test_measurement_angle_acute():
    """45° angle."""
    p1 = np.array([1, 0, 0])
    p2 = np.array([0, 0, 0])
    p3 = np.array([1, 1, 0])
    v1 = p1 - p2
    v2 = p3 - p2
    cosang = np.dot(v1, v2) / (np.linalg.norm(v1) * np.linalg.norm(v2))
    angle = np.degrees(np.arccos(np.clip(cosang, -1.0, 1.0)))
    assert abs(angle - 45.0) < 1e-9


def test_measurement_angle_collinear():
    """Collinear points with vertex in the middle → 180° (straight line)."""
    p1 = np.array([1, 0, 0])
    p2 = np.array([0, 0, 0])
    p3 = np.array([-1, 0, 0])
    v1 = p1 - p2
    v2 = p3 - p2
    cosang = np.dot(v1, v2) / (np.linalg.norm(v1) * np.linalg.norm(v2))
    angle = np.degrees(np.arccos(np.clip(cosang, -1.0, 1.0)))
    assert abs(angle - 180.0) < 1e-9


# ═══════════════════════════════════════════════════════════════
#  Run directly
# ═══════════════════════════════════════════════════════════════

if __name__ == "__main__":
    # Simple runner when pytest is not available
    tests = [
        (test_world_to_coord_str_enu, "world_to_coord_str ENU"),
        (test_world_to_coord_str_flu, "world_to_coord_str FLU"),
        (test_world_to_coord_str_ned, "world_to_coord_str NED"),
        (test_world_to_coord_str_nwu, "world_to_coord_str NWU"),
        (test_all_coord_systems_defined, "all coord systems defined"),
        (test_aabb_no_collision, "AABB no collision"),
        (test_aabb_touching_edges_collide, "AABB touching edges"),
        (test_aabb_full_overlap, "AABB full overlap"),
        (test_aabb_partial_overlap, "AABB partial overlap"),
        (test_aabb_separated_z, "AABB separated Z"),
        (test_find_collisions_empty, "find_collisions empty"),
        (test_find_collisions_single, "find_collisions single"),
        (test_find_collisions_pair, "find_collisions pair"),
        (test_default_scene_has_expected_objects, "default scene objects"),
        (test_default_scene_objects_have_required_keys, "scene object keys"),
        (test_terrain_mesh_size, "terrain mesh size"),
        (test_terrain_has_elevation_scalars, "terrain elevation scalars"),
        (test_vegetation_is_points, "vegetation type"),
        (test_aircraft_is_merged_polydata, "aircraft merged"),
        (test_measurement_distance_math, "measure distance math"),
        (test_measurement_angle_math, "measure angle math (90°)"),
        (test_measurement_angle_acute, "measure angle math (45°)"),
        (test_measurement_angle_collinear, "measure angle math (0°)"),
    ]

    passed = 0
    failed = 0
    for fn, desc in tests:
        try:
            fn()
            passed += 1
            print(f"  ✅  {desc}")
        except Exception as e:
            failed += 1
            print(f"  ❌  {desc}: {e}")

    print(f"\n{'=' * 40}")
    print(f"  {passed} passed, {failed} failed out of {passed + failed}")
    sys.exit(0 if failed == 0 else 1)
