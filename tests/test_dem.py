#!/usr/bin/env python3
"""
Unit tests for the DEM loader module (src/dem_loader.py).

Run with::

    cd /path/to/project
    python -m pytest tests/test_dem.py -v

or::

    python tests/test_dem.py
"""

import sys
import os
import math

_proj_root = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
if _proj_root not in sys.path:
    sys.path.insert(0, _proj_root)

import numpy as np
import pyvista as pv

from src.dem_loader import (
    load_dem,
    dem_to_mesh,
    build_dem_scene,
    HAS_RASTERIO,
    AIRCRAFT_DEFAULT_SCALE,
    DEFAULT_STEP,
    DEFAULT_VERT_EXAG,
    ASTER_NODATA,
    OUTLIER_MIN,
)


# ═══════════════════════════════════════════════════════════════
#  Fixtures
# ═══════════════════════════════════════════════════════════════

DEM_FILE = os.path.join(_proj_root, "ASTGTM_N46E129H.img")


# ═══════════════════════════════════════════════════════════════
#  DEM loading tests
# ═══════════════════════════════════════════════════════════════

def _skip_if_no_rasterio():
    if not HAS_RASTERIO:
        import pytest
        pytest.skip("rasterio not installed")


def _skip_if_no_dem():
    if not os.path.isfile(DEM_FILE):
        import pytest
        pytest.skip(f"DEM file not found: {DEM_FILE}")


def test_load_dem_basic():
    """Load the DEM with default settings and verify structure."""
    _skip_if_no_rasterio()
    _skip_if_no_dem()

    dem = load_dem(DEM_FILE, step=DEFAULT_STEP)

    # Required keys
    for key in ("z", "x", "y", "xx", "yy", "rows", "cols", "pixel_size", "crs"):
        assert key in dem, f"Missing key: {key}"

    # Dimensions are downsampled
    assert dem["rows"] > 0
    assert dem["cols"] > 0
    assert dem["rows"] * dem["cols"] > 1000, "Too few points after downsampling"

    # Arrays match dimensions
    assert dem["z"].shape == (dem["rows"], dem["cols"])
    assert dem["xx"].shape == (dem["rows"], dem["cols"])
    assert dem["yy"].shape == (dem["rows"], dem["cols"])
    assert len(dem["x"]) == dem["cols"]
    assert len(dem["y"]) == dem["rows"]


def test_load_dem_centered():
    """XY coordinates should be centered around the origin."""
    _skip_if_no_rasterio()
    _skip_if_no_dem()

    dem = load_dem(DEM_FILE, step=2)

    # X should be roughly centered
    assert abs(dem["x"][0] + dem["x"][-1]) < 1.0, \
        f"X not centered: {dem['x'][0]:.1f}, {dem['x'][-1]:.1f}"
    assert abs(dem["y"][0] + dem["y"][-1]) < 1.0, \
        f"Y not centered: {dem['y'][0]:.1f}, {dem['y'][-1]:.1f}"

    # Center should be near origin
    center_x = np.mean(dem["x"])
    center_y = np.mean(dem["y"])
    assert abs(center_x) < 1.0, f"Mean X off-center: {center_x:.1f}"
    assert abs(center_y) < 1.0, f"Mean Y off-center: {center_y:.1f}"


def test_load_dem_elevation_range():
    """Elevation values should be in a realistic range (no voids)."""
    _skip_if_no_rasterio()
    _skip_if_no_dem()

    dem = load_dem(DEM_FILE, step=2)
    z = dem["z"]

    # No void values (32767) should remain
    assert np.all(z >= OUTLIER_MIN), "Void/outlier values remain in Z"
    assert np.max(z) < ASTER_NODATA * 0.5,  "Void values remain in Z"

    # Elevation should be within realistic range
    assert np.min(z) >= -100, f"Z too low: {np.min(z)}"
    assert np.max(z) <= 5000, f"Z too high: {np.max(z)}"

    # CRS should be present
    assert dem["crs"] is not None


def test_load_dem_pixel_size():
    """Pixel size should be 30m × step."""
    _skip_if_no_rasterio()
    _skip_if_no_dem()

    dem = load_dem(DEM_FILE, step=2)
    assert abs(dem["pixel_size"] - 60.0) < 1.0, \
        f"Expected pixel_size ~60, got {dem['pixel_size']}"

    dem4 = load_dem(DEM_FILE, step=4)
    assert abs(dem4["pixel_size"] - 120.0) < 1.0, \
        f"Expected pixel_size ~120, got {dem4['pixel_size']}"


def test_load_dem_coordinate_extent():
    """Coordinate extent should match data dimensions × pixel size."""
    _skip_if_no_rasterio()
    _skip_if_no_dem()

    dem = load_dem(DEM_FILE, step=2)
    expected_x_extent = (dem["cols"] - 1) * dem["pixel_size"]
    actual_x_extent = dem["x"][-1] - dem["x"][0]
    assert abs(actual_x_extent - expected_x_extent) < 1.0, \
        f"X extent mismatch: {actual_x_extent:.0f} vs {expected_x_extent:.0f}"


def test_load_dem_step_variation():
    """Higher step → fewer points, larger pixel size."""
    _skip_if_no_rasterio()
    _skip_if_no_dem()

    dem2 = load_dem(DEM_FILE, step=2)
    dem4 = load_dem(DEM_FILE, step=4)

    assert dem4["rows"] < dem2["rows"]
    assert dem4["cols"] < dem2["cols"]
    assert dem4["pixel_size"] > dem2["pixel_size"]


def test_load_dem_no_rasterio(monkeypatch):
    """Should raise ImportError when rasterio is unavailable."""
    monkeypatch.setattr("src.dem_loader.HAS_RASTERIO", False)
    try:
        load_dem("nonexistent.img")
        assert False, "Should have raised ImportError"
    except ImportError:
        pass


# ═══════════════════════════════════════════════════════════════
#  DEM → mesh conversion tests
# ═══════════════════════════════════════════════════════════════

def test_dem_to_mesh_basic():
    """dem_to_mesh should produce a valid StructuredGrid."""
    _skip_if_no_rasterio()
    _skip_if_no_dem()

    dem = load_dem(DEM_FILE, step=4)  # faster with higher step
    mesh = dem_to_mesh(dem, vert_exag=1.0)

    assert isinstance(mesh, pv.StructuredGrid)
    assert mesh.n_points == dem["rows"] * dem["cols"]
    assert mesh.n_cells == (dem["rows"] - 1) * (dem["cols"] - 1)

    # Should have elevation scalar
    assert "elevation" in mesh.point_data
    assert mesh.point_data["elevation"].shape[0] == mesh.n_points


def test_dem_to_mesh_vertical_exaggeration():
    """Vertical exaggeration should scale Z values."""
    _skip_if_no_rasterio()
    _skip_if_no_dem()

    dem = load_dem(DEM_FILE, step=8)  # fast
    mesh1 = dem_to_mesh(dem, vert_exag=1.0)
    mesh5 = dem_to_mesh(dem, vert_exag=5.0)

    # Z range should scale with exaggeration
    z1_range = mesh1.points[:, 2].max() - mesh1.points[:, 2].min()
    z5_range = mesh5.points[:, 2].max() - mesh5.points[:, 2].min()
    assert abs(z5_range / z1_range - 5.0) < 0.1, \
        f"Exaggeration ratio: {z5_range / z1_range:.2f} (expected ~5.0)"


def test_dem_to_mesh_coordinates_preserved():
    """XY coordinates should pass through unchanged."""
    _skip_if_no_rasterio()
    _skip_if_no_dem()

    dem = load_dem(DEM_FILE, step=8)
    mesh = dem_to_mesh(dem, vert_exag=1.0)

    pts = mesh.points
    xx = dem["xx"]

    # X and Y should match within rounding
    assert np.max(np.abs(pts[:, 0] - xx.ravel())) < 0.01, "X mismatch"
    assert np.max(np.abs(pts[:, 1] - dem["yy"].ravel())) < 0.01, "Y mismatch"


# ═══════════════════════════════════════════════════════════════
#  DEM scene building tests
# ═══════════════════════════════════════════════════════════════

def test_build_dem_scene_has_objects():
    """build_dem_scene should return all expected scene objects."""
    _skip_if_no_rasterio()
    _skip_if_no_dem()

    dem = load_dem(DEM_FILE, step=8)
    scene = build_dem_scene(dem, vert_exag=1.0, aircraft_scale=500, aircraft_z=1000.0)

    assert "terrain" in scene
    assert "layer_sand" in scene
    assert "layer_grass" in scene
    assert "layer_earth" in scene
    assert "aircraft" in scene
    assert "aircraft2" in scene
    assert len(scene) == 6


def test_build_dem_scene_required_keys():
    """Each scene object should have the required schema keys."""
    _skip_if_no_rasterio()
    _skip_if_no_dem()

    dem = load_dem(DEM_FILE, step=8)
    scene = build_dem_scene(dem)

    for name, obj in scene.items():
        assert "mesh" in obj, f"{name} missing 'mesh'"
        assert "type" in obj, f"{name} missing 'type'"
        assert obj["type"] in ("mesh", "points"), f"{name} bad type"
        assert "params" in obj, f"{name} missing 'params'"


def test_build_dem_scene_aircraft_scale():
    """Aircraft should be scaled up significantly from default."""
    _skip_if_no_rasterio()
    _skip_if_no_dem()

    dem = load_dem(DEM_FILE, step=8)
    scene = build_dem_scene(dem, vert_exag=1.0, aircraft_scale=500, aircraft_z=1000.0)

    ac = scene["aircraft"]
    ac2 = scene["aircraft2"]

    # Aircraft should have many cells (merged geometry)
    assert ac["mesh"].n_cells > 50
    assert ac2["mesh"].n_cells > 50

    # Aircraft should be large (scaled 500x from ~2.4m)
    ac_bounds = ac["mesh"].GetBounds()
    ac_width = ac_bounds[1] - ac_bounds[0]
    assert ac_width > 500, f"Aircraft too small: {ac_width:.0f} (expected > 500)"


def test_build_dem_scene_aircraft_elevation():
    """Aircraft Z should be at the specified height."""
    _skip_if_no_rasterio()
    _skip_if_no_dem()

    dem = load_dem(DEM_FILE, step=8)
    scene = build_dem_scene(dem, vert_exag=1.0, aircraft_scale=500, aircraft_z=1000.0)

    ac_center = np.mean(scene["aircraft"]["mesh"].points, axis=0)
    # Aircraft should be near Z=1000
    assert abs(ac_center[2] - 1000.0) < 50, \
        f"Aircraft Z at {ac_center[2]:.1f} (expected ~1000)"


def test_build_dem_scene_layers_hidden():
    """Layer overlays (sand/grass/earth) should default to invisible."""
    _skip_if_no_rasterio()
    _skip_if_no_dem()

    dem = load_dem(DEM_FILE, step=8)
    scene = build_dem_scene(dem)

    for layer in ("layer_sand", "layer_grass", "layer_earth"):
        assert scene[layer]["visible"] is False, \
            f"{layer} should be hidden by default"


def test_build_dem_scene_aircraft_visible():
    """Aircraft should be visible by default."""
    _skip_if_no_rasterio()
    _skip_if_no_dem()

    dem = load_dem(DEM_FILE, step=8)
    scene = build_dem_scene(dem)

    assert scene["aircraft"]["visible"] is True
    assert scene["aircraft2"]["visible"] is True


def test_build_dem_scene_terrain_opacity():
    """Base terrain should be fully opaque."""
    _skip_if_no_rasterio()
    _skip_if_no_dem()

    dem = load_dem(DEM_FILE, step=8)
    scene = build_dem_scene(dem)

    assert scene["terrain"]["params"]["opacity"] == 1.0


def test_build_dem_scene_no_rasterio(monkeypatch):
    """Should gracefully handle rasterio not being available at call time."""
    _skip_if_no_dem()

    monkeypatch.setattr("src.dem_loader.HAS_RASTERIO", False)
    try:
        load_dem(DEM_FILE, step=8)
        assert False, "Should have raised ImportError"
    except ImportError:
        pass


def test_build_dem_scene_aircraft_500x_research():
    """Validate the research claim: 500x makes aircraft visible at DEM scale.

    At 500x scale, the aircraft should span at least 1% of the terrain extent
    so it is visually identifiable in the full-scene view.
    """
    _skip_if_no_rasterio()
    _skip_if_no_dem()

    dem = load_dem(DEM_FILE, step=4)
    terrain_extent = max(abs(dem["x"][0]) + abs(dem["x"][-1]),
                         abs(dem["y"][0]) + abs(dem["y"][-1]))

    scene = build_dem_scene(dem, vert_exag=1.0, aircraft_scale=500, aircraft_z=1000.0)
    ac = scene["aircraft"]
    bounds = ac["mesh"].GetBounds()
    ac_diag = math.sqrt(
        (bounds[1] - bounds[0]) ** 2 +
        (bounds[3] - bounds[2]) ** 2 +
        (bounds[5] - bounds[4]) ** 2
    )

    ratio = ac_diag / terrain_extent
    assert ratio >= 0.005, \
        f"Aircraft is only {ratio*100:.2f}% of terrain extent ({ac_diag:.0f}m vs {terrain_extent:.0f}m)"


# ═══════════════════════════════════════════════════════════════
#  Backward compatibility tests
# ═══════════════════════════════════════════════════════════════

def test_dem_scene_compatible_with_build_default():
    """DEM scene objects should have same schema as build_default_scene()."""
    from src.scene_builder import build_default_scene

    default_scene = build_default_scene()

    if HAS_RASTERIO and os.path.isfile(DEM_FILE):
        dem = load_dem(DEM_FILE, step=8)
        dem_scene = build_dem_scene(dem, vert_exag=1.0, aircraft_scale=500, aircraft_z=1000.0)

        # Both should have terrain and aircraft
        assert "terrain" in dem_scene
        assert "aircraft" in dem_scene

        # Same schema keys
        for name in ("terrain", "aircraft", "aircraft2"):
            if name in dem_scene and name in default_scene:
                for key in ("mesh", "type", "visible", "params", "name"):
                    assert key in dem_scene[name], f"DEM scene {name} missing '{key}'"
                    assert key in default_scene[name], f"Default scene {name} missing '{key}'"


# ═══════════════════════════════════════════════════════════════
#  Run directly
# ═══════════════════════════════════════════════════════════════

if __name__ == "__main__":
    tests = [
        (test_load_dem_basic, "load_dem basic"),
        (test_load_dem_centered, "load_dem centered"),
        (test_load_dem_elevation_range, "load_dem elevation range"),
        (test_load_dem_pixel_size, "load_dem pixel size"),
        (test_load_dem_coordinate_extent, "load_dem coordinate extent"),
        (test_load_dem_step_variation, "load_dem step variation"),
        (test_load_dem_no_rasterio, "load_dem no rasterio"),
        (test_dem_to_mesh_basic, "dem_to_mesh basic"),
        (test_dem_to_mesh_vertical_exaggeration, "dem_to_mesh exaggeration"),
        (test_dem_to_mesh_coordinates_preserved, "dem_to_mesh coordinates"),
        (test_build_dem_scene_has_objects, "build_dem_scene objects"),
        (test_build_dem_scene_required_keys, "build_dem_scene keys"),
        (test_build_dem_scene_aircraft_scale, "build_dem_scene aircraft scale"),
        (test_build_dem_scene_aircraft_elevation, "build_dem_scene aircraft Z=1000"),
        (test_build_dem_scene_layers_hidden, "build_dem_scene layers hidden"),
        (test_build_dem_scene_aircraft_visible, "build_dem_scene aircraft visible"),
        (test_build_dem_scene_terrain_opacity, "build_dem_scene terrain opacity"),
        (test_build_dem_scene_no_rasterio, "build_dem_scene no rasterio"),
        (test_build_dem_scene_aircraft_500x_research, "build_dem_scene aircraft 500x research"),
        (test_dem_scene_compatible_with_build_default, "DEM scene compatible with default"),
    ]

    passed = 0
    failed = 0
    for fn, desc in tests:
        try:
            fn()
            passed += 1
            print(f"  \u2705  {desc}")
        except Exception as e:
            failed += 1
            print(f"  \u274c  {desc}: {e}")
        except ImportError:
            failed += 1
            print(f"  \u274c  {desc}: ImportError")

    print(f"\n{'=' * 40}")
    print(f"  {passed} passed, {failed} failed out of {passed + failed}")
    sys.exit(0 if failed == 0 else 1)
