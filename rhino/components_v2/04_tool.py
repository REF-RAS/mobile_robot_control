"""Build a tool from a mesh on disk.

COMPAS FAB v2.0.1

Inputs : tool (int, item), show (bool, item), data_path (str, item, optional)
Outputs: out, tool, M, ee_plane

compas_fab.robots.Tool was removed in 2.x. compas_robots.ToolModel takes the
same constructor arguments and is the direct replacement -- it *is* the model
rather than wrapping one, so the frame lives at tool.frame.

The tool is not attached here. Wire this into `attach tool`, which registers it
on the RobotCell and updates the RobotCellState.

WHY data_path IS NOW OPTIONAL
-----------------------------
The v1 component took a single `data_path` and joined 'tool_geometry' onto it.
That cannot work here, because the two tools live in different repositories:

    measurement_toolx3.stl  ->  mobile_robot_control/data/tool_geometry/
    gripper-basic.stl       ->  workshop_qut_mobile_robots/data/tool_geometry/

and the path it was fed was hardcoded to another machine
('C:\\Users\\heywoodk\\...'), so it broke as soon as anyone else opened the file.

Each package is located via importlib without importing it, so the roots follow
whatever python-3.pth points at and stay correct on any machine. Set
`data_path` only to override with an explicit folder.
"""

import importlib.util
import os

from compas.datastructures import Mesh
from compas.geometry import Frame
from compas_rhino.conversions import frame_to_rhino_plane, mesh_to_rhino
from compas_robots import ToolModel

# Packages whose <repo>/data/tool_geometry folder is searched, in order.
# marker_tracking is the handle onto workshop_qut_mobile_robots.
SEARCH_PACKAGES = ["mobile_robot_control", "workshop_qut_mobile_robots",
                   "marker_tracking", "ur_fabrication_control"]

TOOLS = {
    0: ("measurement_toolx3.stl", Frame([0, 0, 0.109], [1, 0, 0], [0, 1, 0]), "measurement_tool"),
    1: ("gripper-basic.stl", Frame([0, -0.06185557, 0.37374513], [0, 1, 0], [-1, 0, 0]), "gripper_basic"),
}

M = None
ee_plane = None


def data_roots():
    """<repo>/data/tool_geometry for each locatable package, plus any override."""
    roots = []
    if data_path:  # noqa: F821
        roots.append(os.path.join(data_path, "tool_geometry"))  # noqa: F821
        roots.append(data_path)  # noqa: F821
    for name in SEARCH_PACKAGES:
        try:
            spec = importlib.util.find_spec(name)
        except Exception:
            continue
        if not spec or not spec.submodule_search_locations:
            continue
        pkg_dir = list(spec.submodule_search_locations)[0]
        root = os.path.abspath(os.path.join(pkg_dir, "..", "..", "data", "tool_geometry"))
        if root not in roots:
            roots.append(root)
    return roots


def find_mesh(filename):
    roots = data_roots()
    for root in roots:
        candidate = os.path.join(root, filename)
        if os.path.isfile(candidate):
            return candidate

    # Fail with the information needed to fix it, rather than just a path.
    print("Could not find '%s'. Searched:" % filename)
    for root in roots:
        print("    [%s] %s" % ("ok " if os.path.isdir(root) else "no ", root))
    available = set()
    for root in roots:
        if os.path.isdir(root):
            available.update(f for f in os.listdir(root) if f.lower().endswith((".stl", ".obj")))
    if available:
        print("\nAvailable geometry:")
        for f in sorted(available):
            print("    %s" % f)
    raise IOError("Tool geometry '%s' not found. See the report above." % filename)


if tool in TOOLS:  # noqa: F821
    filename, ee_frame, tool_name = TOOLS[tool]  # noqa: F821
    ee_mesh = Mesh.from_stl(find_mesh(filename))

    if show:  # noqa: F821
        M = mesh_to_rhino(ee_mesh)

    # `name` becomes the id the tool is registered under in the RobotCell, so
    # give it a stable one rather than letting ToolModel default to
    # 'attached_tool' -- two tools sharing a name overwrite each other.
    tool = ToolModel(ee_mesh, ee_frame, name=tool_name)
    ee_plane = frame_to_rhino_plane(ee_frame)
    print("Loaded '%s' as tool id '%s'." % (filename, tool_name))
else:
    print("No tool for index %r. Valid: %s" % (tool, sorted(TOOLS)))  # noqa: F821
    tool = None
