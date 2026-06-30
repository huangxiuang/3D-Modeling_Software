"""
AABB (axis-aligned bounding box) collision detection.

Used by the collision-detection tool to report all intersecting object pairs
in the scene.
"""

import pyvista as pv


def check_aabb_collision(mesh1, mesh2):
    """Return ``True`` if the axis-aligned bounding boxes of two meshes
    intersect.

    ``GetBounds()`` returns ``(xmin, xmax, ymin, ymax, zmin, zmax)``.
    Boxes intersect if they overlap in all three axes.
    """
    b1 = mesh1.GetBounds()
    b2 = mesh2.GetBounds()

    return (
        b1[0] <= b2[1] and b1[1] >= b2[0]  # x overlap
        and b1[2] <= b2[3] and b1[3] >= b2[2]  # y overlap
        and b1[4] <= b2[5] and b1[5] >= b2[4]   # z overlap
    )


# ──────────────────────────────────────────────
# Higher-level batch check
# ──────────────────────────────────────────────

def find_collisions(visible_meshes, mesh_names):
    """Return a list of ``(name_i, name_j)`` pairs for every intersecting
    pair of bounding boxes in *visible_meshes*.

    Parameters
    ----------
    visible_meshes : list of pv.PolyData
    mesh_names : list of str
        Parallel list of user-visible names.
    """
    results = []
    n = len(visible_meshes)
    for i in range(n):
        for j in range(i + 1, n):
            if check_aabb_collision(visible_meshes[i], visible_meshes[j]):
                results.append((mesh_names[i], mesh_names[j]))
    return results
