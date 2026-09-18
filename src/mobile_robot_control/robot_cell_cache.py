"""Disk cache for a loaded RobotCell.

COMPAS FAB v2.0.1

WHY
---
Loading the cell from ROS takes ~16 seconds: it fetches the URDF and SRDF off
topics and then pulls ~30 meshes over HTTP. That cost is paid again on every
Rhino restart, every `req_reload`, and every time the mesh server is restarted
-- and it cannot be paid at all when the robot is off.

``RobotCell.__data__`` serialises the robot model, semantics, tool models and
rigid bodies, meshes included, so the whole cell round-trips through
``compas.json_dump``/``json_load``. Caching it removes the network from the
common path entirely.

WHAT THIS IS NOT
----------------
This caches the robot's *description* -- geometry and kinematics. Those are
static facts about the machine: the URDF does not change while the robot drives
around.

It does NOT cache anything measured -- joint angles, lift height, base pose.
Those come from the robot every time or not at all, which is the whole point of
the strictly-online policy in rhino/components_v2/README.md. A cached cell with
no ROS connection gives you the robot's shape and nothing about its state, so
`plan motion`, `inverse` and `analytic inverse` still refuse exactly as before
-- they need a planner and live joint states, neither of which a cache provides.

The distinction matters: a stale mesh draws the robot slightly wrong, which is
visible. A stale joint angle gets executed.

CACHE LOCATION
--------------
``~/.mobile_robot_control/robot_cells``. Deliberately outside the repository:
the cell JSON runs to tens of megabytes, and the workspace lives in a synced
OneDrive folder that should not be carrying it.

STALENESS
---------
A cached cell is only as good as the robot description it came from. If the
robot is reflashed, the arm swapped, or a URDF edited, the cache is wrong and
nothing here can detect that -- the robot would have to be online to compare,
and if it were online you would not be reading the cache.

So the cache records when it was written and reports its own age. Treat a cell
older than the last robot-side change as suspect, and press `load` while
connected to refresh it.
"""

import os
import time

import compas

__all__ = [
    "cache_dir",
    "cache_path",
    "save",
    "load",
    "info",
    "clear",
]

#: Default cache directory. Outside the repo and outside OneDrive.
DEFAULT_DIR = os.path.join(os.path.expanduser("~"), ".mobile_robot_control", "robot_cells")


def cache_dir(directory=None):
    """The cache directory, created if needed."""
    directory = directory or DEFAULT_DIR
    if not os.path.exists(directory):
        os.makedirs(directory)
    return directory


def cache_path(name, directory=None):
    """Path of the cell JSON for ``name``."""
    return os.path.join(cache_dir(directory), "%s.json" % name)


def _meta_path(name, directory=None):
    return os.path.join(cache_dir(directory), "%s.meta.json" % name)


def save(robot_cell, name, directory=None, source=None):
    """Write ``robot_cell`` to the cache under ``name``.

    The write goes to a temporary file and is then moved into place, so an
    interrupted save cannot leave a half-written cell that would later fail to
    parse -- and fail at the moment the robot is unreachable, which is the one
    moment the cache is needed.

    Parameters
    ----------
    robot_cell : :class:`compas_fab.robots.RobotCell`
    name : str
        Cache key. Use one per robot -- 'robot_AB' -- so two robots do not
        overwrite each other.
    directory : str, optional
    source : str, optional
        Where the cell came from, recorded for the report. e.g. the mesh
        server URL.

    Returns
    -------
    dict
        The metadata written, including ``path`` and ``bytes``.
    """
    path = cache_path(name, directory)
    tmp = path + ".tmp"

    compas.json_dump(robot_cell, tmp)
    if os.path.exists(path):
        os.remove(path)
    os.rename(tmp, path)

    model = robot_cell.robot_model
    meta = {
        "name": name,
        "path": path,
        "bytes": os.path.getsize(path),
        "saved_at": time.time(),
        "saved_at_text": time.strftime("%Y-%m-%d %H:%M:%S"),
        "robot_name": getattr(model, "name", None),
        "links": len(model.links) if model else 0,
        "joints": len(model.joints) if model else 0,
        "groups": list(robot_cell.group_names) if robot_cell.robot_semantics else [],
        "tools": list(robot_cell.tool_models.keys()),
        "source": source,
    }
    compas.json_dump(meta, _meta_path(name, directory))
    return meta


def info(name, directory=None):
    """Metadata for a cached cell, without loading it.

    Returns ``None`` if there is no cache. Reading the sidecar rather than the
    cell itself keeps this cheap enough to call on every solve -- the cell JSON
    is tens of megabytes and parsing it to answer "is there a cache?" would
    stall the canvas.
    """
    path = cache_path(name, directory)
    if not os.path.exists(path):
        return None

    meta = {}
    meta_path = _meta_path(name, directory)
    if os.path.exists(meta_path):
        try:
            meta = compas.json_load(meta_path)
        except Exception:
            meta = {}

    meta.setdefault("name", name)
    meta.setdefault("path", path)
    meta["bytes"] = os.path.getsize(path)
    meta.setdefault("saved_at", os.path.getmtime(path))
    meta.setdefault("saved_at_text", time.strftime("%Y-%m-%d %H:%M:%S",
                                                   time.localtime(meta["saved_at"])))
    meta["age_days"] = (time.time() - meta["saved_at"]) / 86400.0
    return meta


def load(name, directory=None):
    """Read a cached cell.

    Returns
    -------
    tuple
        ``(robot_cell, meta)``, or ``(None, None)`` if nothing is cached.

    Raises
    ------
    ValueError
        If the file exists but cannot be parsed. This is raised rather than
        returned as None so the caller cannot mistake a corrupt cache for an
        absent one: the first is worth reporting and deleting, the second is
        just a machine that has not connected yet.
    """
    path = cache_path(name, directory)
    if not os.path.exists(path):
        return None, None
    try:
        robot_cell = compas.json_load(path)
    except Exception as e:
        raise ValueError(
            "Cached cell at %s could not be read (%s: %s).\n"
            "Delete it and press `load` while connected to rebuild it."
            % (path, type(e).__name__, e)
        )
    return robot_cell, info(name, directory)


def clear(name=None, directory=None):
    """Delete one cached cell, or all of them. Returns the paths removed."""
    directory = cache_dir(directory)
    removed = []
    if name:
        candidates = [cache_path(name, directory), _meta_path(name, directory)]
    else:
        candidates = [os.path.join(directory, f) for f in os.listdir(directory)
                      if f.endswith(".json")]
    for path in candidates:
        if os.path.exists(path):
            os.remove(path)
            removed.append(path)
    return removed


def describe(name, directory=None):
    """Human-readable lines about the cache state, for a component report."""
    meta = info(name, directory)
    if meta is None:
        return ["no cached cell (press `load` while connected to create one)"]

    age = meta["age_days"]
    age_text = "%.1f days old" % age if age >= 1 else "%.0f minutes old" % (age * 1440)
    lines = [
        "cached cell : '%s', %s" % (meta.get("robot_name") or "?", age_text),
        "              written %s, %.1f MB"
        % (meta.get("saved_at_text", "?"), meta["bytes"] / 1048576.0),
    ]
    if meta.get("links"):
        lines.append("              %d links, groups: %s"
                     % (meta["links"], ", ".join(meta.get("groups") or []) or "-"))
    return lines
