"""
Scene tree model — hierarchical node types and factory for the scene browser.

This module is decoupled from Qt widgets: it defines the data model (node
type constants + ``NodeData``) and a factory that builds a ``QTreeWidget``
tree from a nested tuple structure.  The factory is invoked by
``MainWindow._setup_docks()`` in *main_window.py*.
"""

import dataclasses
from typing import List, Optional

from PyQt5 import QtWidgets, QtGui
from PyQt5.QtCore import Qt


# ---------------------------------------------------------------------------
# 1.  Node-type constants
# ---------------------------------------------------------------------------

class SceneNodeType:
    """Canonical string constants used as NodeData.node_type values."""

    ROOT            = "root"
    SCENE_SETTINGS  = "scene_settings"
    FLIGHT_PLATFORM = "flight_platform"
    PATH_PLANNING   = "path_planning"
    ANIMATION_TASK  = "animation_task"
    AIRCRAFT        = "aircraft"
    WAYPOINT        = "waypoint"
    GLOBAL_TOOL     = "global_tool"
    PATH_ACTION     = "path_action"
    ANIM_ACTION     = "anim_action"


# ---------------------------------------------------------------------------
# 2.  NodeData — payload carried by every tree item
# ---------------------------------------------------------------------------

@dataclasses.dataclass
class NodeData:
    """Payload carried by every QTreeWidgetItem in the scene tree.

    Fields
    ------
    node_type:
        One of the SceneNodeType constants -- drives routing / icon
        selection in the view layer.
    label:
        Display string shown as the item's text.
    scene_obj_name:
        Optional key into the master scene_objects dict (meshes/actors).
    waypoint_index:
        Optional index into a parent aircraft's waypoints list.
    aircraft_name:
        Parent aircraft identifier (used primarily by waypoint nodes).
    parent_node_type:
        NodeType of the immediate parent -- allows routing decisions
        without walking the tree.
    tooltip:
        Hover tooltip string.
    icon_name:
        Logical icon identifier (resolved to a QIcon by the view layer).
    is_editable:
        Whether the user may rename this item in-place.
    is_deletable:
        Whether the user may delete this item.
    slot_name:
        Name of the MainWindow handler method to invoke when the item is
        activated / double-clicked (empty string -> no handler).
    """

    node_type:        str
    label:            str
    scene_obj_name:   Optional[str] = None
    waypoint_index:   Optional[int] = None
    aircraft_name:    Optional[str] = None
    parent_node_type: Optional[str] = None
    tooltip:          str            = ""
    icon_name:        str            = ""
    is_editable:      bool           = False
    is_deletable:     bool           = False
    slot_name:        str            = ""


# ---------------------------------------------------------------------------
# 3.  SceneTreeFactory — builds the QTreeWidget tree
# ---------------------------------------------------------------------------

class SceneTreeFactory:
    """Factory that populates a ``QTreeWidget`` with a static tree skeleton.

    Usage::

        tree = QtWidgets.QTreeWidget()
        items = SceneTreeFactory.build_tree(tree)
        # items is a dict:  node-type-string -> QTreeWidgetItem
    """

    # ── Icon name → unicode symbol map ──────────────
    _ICON_MAP = {
        SceneNodeType.ROOT:            "\U0001F30D",  # globe
        SceneNodeType.SCENE_SETTINGS:  "\u2699\ufe0f",  # gear
        SceneNodeType.FLIGHT_PLATFORM: "\u2708\ufe0f",  # airplane
        SceneNodeType.PATH_PLANNING:   "\U0001F9F0",    # target
        SceneNodeType.ANIMATION_TASK:  "\u25b6\ufe0f",  # play
        SceneNodeType.AIRCRAFT:        "\U0001F680",    # rocket (child aircraft)
        SceneNodeType.WAYPOINT:        "\u2b50",        # star
        SceneNodeType.GLOBAL_TOOL:     "\U0001F6E0\ufe0f",  # tool
        SceneNodeType.PATH_ACTION:     "\u2795",        # plus
        SceneNodeType.ANIM_ACTION:     "\u2795",        # plus
    }

    _DEFAULT_FONT_SIZE = 11

    @classmethod
    def build_tree(cls, tree_widget: QtWidgets.QTreeWidget):
        """Create the static tree skeleton and return a ``{node_type: item}`` map."""

        font = tree_widget.font()
        font.setPointSize(cls._DEFAULT_FONT_SIZE)

        # Top level:  4 main categories
        items: dict = {}

        def _icon(symbol: str) -> QtGui.QIcon:
            """Return a transparent QIcon with the given symbol."""
            pm = QtGui.QPixmap(24, 24)
            pm.fill(Qt.transparent)
            # We cannot trivially paint a unicode char onto a pixmap without
            # a QPainter in scope, so we fall back to a plain text label for
            # the icon column.  For now just return an empty icon.
            return QtGui.QIcon()

        def _make_item(parent, node_type, label, tooltip="",
                       icon_name="", slot_name="",
                       is_editable=False, is_deletable=False,
                       scene_obj_name=None, waypoint_index=None,
                       aircraft_name=None):
            nd = NodeData(
                node_type=node_type,
                label=label,
                scene_obj_name=scene_obj_name,
                waypoint_index=waypoint_index,
                aircraft_name=aircraft_name,
                parent_node_type=parent.data(0, Qt.UserRole).node_type
                    if isinstance(parent, QtWidgets.QTreeWidgetItem) else None,
                tooltip=tooltip or label,
                icon_name=icon_name or node_type,
                is_editable=is_editable,
                is_deletable=is_deletable,
                slot_name=slot_name,
            )
            item = QtWidgets.QTreeWidgetItem(parent)
            item.setText(0, label)
            item.setData(0, Qt.UserRole, nd)
            item.setToolTip(0, nd.tooltip)
            item.setFont(0, font)
            items[node_type] = item
            return item

        # ── 3 top-level root nodes ──
        root_flight   = _make_item(
            tree_widget, SceneNodeType.FLIGHT_PLATFORM, "飞行平台")
        root_path     = _make_item(
            tree_widget, SceneNodeType.PATH_PLANNING, "路径规划")
        root_anim     = _make_item(
            tree_widget, SceneNodeType.ANIMATION_TASK, "动画与任务")

        # -- Flight children are added dynamically by _populate_aircraft_nodes
        #    (no static children here)

        # -- Path Planning children --------------------------------------
        path_children: List[tuple] = [
            ("添加路径点 (点击场景)",  SceneNodeType.PATH_ACTION, "_on_wp_button"),
            ("精准添加路径",           SceneNodeType.PATH_ACTION, "_open_precise_wp_dialog"),
            ("清除所有路径点",         SceneNodeType.PATH_ACTION, "_clear_waypoints"),
        ]

        for label, nt, slot in path_children:
            item = cls._create_item(
                root_path,
                NodeData(
                    node_type=nt,
                    label=label,
                    parent_node_type=SceneNodeType.PATH_PLANNING,
                    tooltip=label,
                    icon_name=nt,
                    slot_name=slot,
                ),
            )
            items[f"{SceneNodeType.PATH_PLANNING}.{label}"] = item

        # -- Animation / Task children -----------------------------------
        anim_children: List[tuple] = [
            ("开始飞行 / 停止飞行", SceneNodeType.ANIM_ACTION, "_toggle_flight"),
            ("编队飞行",            SceneNodeType.ANIM_ACTION, "_on_formation_toggled"),
            ("保存飞行数据",        SceneNodeType.ANIM_ACTION, "_save_flight_data"),
            ("载入飞行数据",        SceneNodeType.ANIM_ACTION, "_load_flight_data"),
        ]

        for label, nt, slot in anim_children:
            item = cls._create_item(
                root_anim,
                NodeData(
                    node_type=nt,
                    label=label,
                    parent_node_type=SceneNodeType.ANIMATION_TASK,
                    tooltip=label,
                    icon_name=nt,
                    slot_name=slot,
                ),
            )
            items[f"{SceneNodeType.ANIMATION_TASK}.{label}"] = item

        return items

    @staticmethod
    def _create_item(parent, data: NodeData) -> QtWidgets.QTreeWidgetItem:
        """Create a single tree item from *data* and attach it to *parent*."""
        item = QtWidgets.QTreeWidgetItem(parent)
        item.setText(0, data.label)
        item.setData(0, Qt.UserRole, data)
        item.setToolTip(0, data.tooltip)
        item.setFont(0, parent.font(0) if isinstance(parent, QtWidgets.QTreeWidgetItem)
                     else QtWidgets.QTreeWidget().font())
        return item
