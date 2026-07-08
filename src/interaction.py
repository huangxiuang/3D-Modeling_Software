"""
Interaction mode enum — application-level state for 3D click routing.

Mouse event handling moved to ``ClickablePlotter`` in *main_window.py*
to work around PyVista's VTK observer pipeline bug (observers registered
via ``add_observer`` never fire on ``RenderWindowInteractor``).
"""

from enum import Enum, auto


class InteractionMode(Enum):
    """Application-level interaction mode (exactly one active at a time)."""
    NORMAL = auto()             # Orbit / pan / zoom + object selection
    MEASURE_DISTANCE = auto()   # Left-click places distance-measurement points
    MEASURE_ANGLE = auto()      # Left-click places angle-measurement points
    WAYPOINT = auto()           # Left-click places 3D waypoints on terrain
