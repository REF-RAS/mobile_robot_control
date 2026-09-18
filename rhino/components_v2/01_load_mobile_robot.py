"""Load the mobile robot cell from ROS 2, or from the disk cache when offline.

COMPAS FAB v2.0.1

Every successful online load writes the cell to
``~/.mobile_robot_control/robot_cells``. If the robot cannot be reached, the
cached cell is loaded instead and the component says so in bold. That makes the
16-second mesh download a once-per-robot-change cost rather than a
once-per-restart one, and lets the geometry-only parts of the definition run
with the robot switched off.

The cache holds the robot's *description*, never its state -- see the long note
above the load block. Offline there is no client and no planner, so everything
that needs a measurement still refuses.

Inputs
------
ros_client  : a connected compas_fab.backends.RosClient
load        : Button -- fetch the cell (from ROS if reachable, else the cache)
prefix      : str, ROS namespace -- 'robot' on this machine
file_server : str, HTTP mesh server, e.g. 'http://192.168.0.200:9190'

Outputs
-------
mobile_robot : MobileRobot
robot_cell   : compas_fab.robots.RobotCell
cell_state   : compas_fab.robots.RobotCellState
meshes       : the drawn robot geometry

WHY FAILURES USED TO VANISH
---------------------------
`load` is a Button, so it is True for exactly one solve. If load_robot_cell
raised, the component went red for that solve -- then Grasshopper re-solved
with load False, skipped the whole block, raised nothing, and cleared `out`.
The component flickered red and went quiet with no diagnosis anywhere.

Now the load is wrapped, the outcome is classified into something actionable,
and the last status is kept in sticky so it survives the button releasing.
`out` always says what state the component is in.

Editing mobile_robot_control requires a Rhino restart (or the reload
component) to take effect: Python caches the module in sys.modules, and the
MobileRobot held in sticky is an instance of the class loaded at the time.
"""

import time
from urllib.error import HTTPError, URLError
from urllib.request import urlopen

import Rhino
import scriptcontext
from scriptcontext import sticky as st

from compas.scene import SceneObject
from compas_fab.ghpython.scene import RobotCellObject

from mobile_robot_control import robot_cell_cache
from mobile_robot_control.mobile_robot import MobileRobot
from mobile_robot_control.mobile_robot_client import MobileRobotClient

scriptcontext.doc = Rhino.RhinoDoc.ActiveDoc
scriptcontext.doc = ghdoc  # noqa: F821

# Namespaces the sticky cache per robot, so a definition pointed at AA and one
# pointed at AB do not share (and overwrite) each other's cached cell.
key = "robot_AB"
planner_key = key + "_planner"
status_key = key + "_status"
mobile_client_key = key + "_mobile_client"

prefix = prefix or ""  # noqa: F821
file_server = file_server or None  # noqa: F821

# Namespace for MoveIt's SERVICES, which is not the same as the topic prefix.
#
# On this robot the topics are namespaced (/robot/robot_description) but
# move_group's services are not (/plan_kinematic_path, verified with
# `ros2 service list`). compas_fab hardcodes the unnamespaced names, so an
# empty value here is correct and no rewriting happens.
#
# Set this to 'robot' (or whatever `ros2 service list | grep plan_kinematic_path`
# shows) if a future bringup launches move_group inside a namespace -- otherwise
# every planning call waits on a service nobody provides and times out.
MOVEIT_NAMESPACE = ""


def base_url():
    """The URL load_robot_cell will use, including the client's own default."""
    if file_server:  # noqa: F821
        return file_server.rstrip("/")  # noqa: F821
    host = getattr(ros_client, "host", "localhost")  # noqa: F821
    return "http://{}:9190".format(host)


def check_file_server(url):
    """Is anything listening? Returns (reachable, detail).

    A 404 from the root still means the server is up -- a static server may
    simply not list directories. Only a transport-level failure means nothing
    is there, and that is the case worth calling out, because it is the one
    that looks identical to 'the robot is off'.
    """
    try:
        urlopen(url + "/", timeout=5)
        return True, "responding"
    except HTTPError as e:
        return True, "responding (HTTP %s at root, which is fine)" % e.code
    except URLError as e:
        return False, str(e.reason)
    except Exception as e:
        return False, "%s: %s" % (type(e).__name__, e)


def diagnose(exc, url):
    """Turn an exception into the thing to go and fix."""
    text = str(exc)
    # HTTPError subclasses URLError, so it has to be tested first or a 404
    # gets reported as "nothing is listening", sending you to fix the wrong
    # thing entirely.
    if isinstance(exc, HTTPError) or "404" in text:
        return ("MESH FILE SERVER IS UP BUT THE PATH IS WRONG", [
            "%s answered, but not for the mesh path requested." % url,
            "Its document root must have package names as direct children,",
            "e.g. <root>/ur_description/meshes/...",
            "Run the file server diagnostic for the exact failing URL.",
        ])
    if isinstance(exc, URLError) or "10061" in text or "refused" in text.lower():
        return ("MESH FILE SERVER NOT RUNNING", [
            "Nothing is listening at %s" % url,
            "Start it on the robot, serving the ROS share tree:",
            "    python3 -m http.server 9190 --directory ~/mesh_root",
            "Then check from here:  curl -I %s/ur_description/meshes/ur20/visual/base.dae" % url,
        ])
    if isinstance(exc, TimeoutError) or "Timeout waiting for message" in text:
        return ("URDF/SRDF TOPIC DID NOT ARRIVE", [
            "Asked for '%s/robot_description'." % prefix,  # noqa: F821
            "prefix is currently %r -- it should be 'robot' on this machine." % prefix,  # noqa: F821
            "Otherwise robot_state_publisher or move_group may not be running.",
        ])
    return ("LOAD FAILED", [text])


def build(robot_cell, source):
    """Wrap a cell in a MobileRobot and put it in sticky.

    `source` is recorded on the instance so every downstream component can tell
    a cell fetched from the robot from one read off disk. Nothing else in the
    definition needs to care -- but if a mesh ever looks wrong, the first
    question is which of the two you are looking at.
    """
    mobile_robot = MobileRobot(
        robot_cell,
        robot_cell_state=robot_cell.default_cell_state(),
        scene_object=SceneObject(item=robot_cell, sceneobject_type=RobotCellObject),
    )
    mobile_robot.attributes["cell_source"] = source
    st[key] = mobile_robot
    st.pop(planner_key, None)  # a new cell must be re-uploaded to MoveIt
    return mobile_robot


# --------------------------------------------------------------------------
# Load the cell. From ROS when the robot is reachable, otherwise from the
# disk cache.
#
# WHY A CACHE IS NOT A POLICY VIOLATION
# -------------------------------------
# This definition is strictly online (see README.md): nothing the robot can
# measure may be supplied by hand. The cell is not a measurement. It is the
# robot's *description* -- link geometry and kinematics, which do not change
# while the robot drives around. Caching it removes a 16-second download; it
# invents nothing.
#
# What the cache deliberately does NOT provide is a client or a planner. With
# no ROS connection there are no joint states, no lift height and no MoveIt,
# so `plan motion`, `inverse` and `analytic inverse` refuse exactly as they
# did before. Offline you get the robot's shape, and nothing about its state.
# --------------------------------------------------------------------------
if load:  # noqa: F821
    url = base_url()
    cache_note = None
    try:
        if not (ros_client and ros_client.is_connected):  # noqa: F821
            raise URLError("not connected to ROS")

        reachable, detail = check_file_server(url)
        if not reachable:
            raise URLError(detail)

        robot_cell = ros_client.load_robot_cell(  # noqa: F821
            load_geometry=True,
            urdf_param_name="{}/robot_description".format(prefix),
            srdf_param_name="{}/robot_description_semantic".format(prefix),
            http_file_server_base_url=file_server,
        )
        build(robot_cell, "ros")

        # Refresh the cache on every successful online load, so what is on
        # disk is always the last cell the robot actually published. A write
        # failure must not fail the load -- the cell in hand is good.
        try:
            meta = robot_cell_cache.save(robot_cell, key, source=url)
            cache_note = "cached %.1f MB to %s" % (meta["bytes"] / 1048576.0, meta["path"])
        except Exception as e:
            cache_note = "CACHE WRITE FAILED (%s: %s) -- load itself was fine" % (
                type(e).__name__, e)

        st[status_key] = {
            "ok": True, "when": time.strftime("%H:%M:%S"),
            "headline": "LOADED '%s' FROM THE ROBOT" % robot_cell.robot_model.name,
            "detail": [
                "%d links, groups: %s"
                % (len(robot_cell.robot_model.links), ", ".join(robot_cell.group_names)),
                "mesh server: %s (%s)" % (url, detail),
                cache_note,
            ],
        }

    except Exception as e:
        headline, detail_lines = diagnose(e, url)

        # Fall back to disk. Reported as its own outcome, never folded into a
        # success: the robot was not reached, and that has to stay visible.
        try:
            cached_cell, meta = robot_cell_cache.load(key)
        except ValueError as cache_error:
            cached_cell, meta = None, None
            detail_lines = detail_lines + ["", "Cache unusable: %s" % cache_error]

        if cached_cell is not None:
            build(cached_cell, "cache")
            st[status_key] = {
                "ok": True, "when": time.strftime("%H:%M:%S"),
                "headline": "OFFLINE -- LOADED '%s' FROM CACHE"
                            % cached_cell.robot_model.name,
                "detail": robot_cell_cache.describe(key) + [
                    "",
                    "The robot was NOT reached: %s" % headline,
                    "Geometry only. No joint states, no lift height, no MoveIt,",
                    "so planning and IK will refuse. Drawing and frame algebra work.",
                ],
                "exc": "%s: %s" % (type(e).__name__, e),
            }
        else:
            st[status_key] = {
                "ok": False, "when": time.strftime("%H:%M:%S"),
                "headline": headline,
                "detail": detail_lines + [
                    "",
                    "No cached cell to fall back on either. Connect to the robot",
                    "once and press `load`; the cell is then kept on disk and",
                    "later offline loads work without it.",
                ],
                "exc": "%s: %s" % (type(e).__name__, e),
            }

mobile_robot = st.get(key, None)

# --------------------------------------------------------------------------
# Attach the client and planner. A client can be restarted without reloading
# geometry, so this is kept separate from the load above.
# --------------------------------------------------------------------------
planner_note = "no planner (needs a connected client)"
if mobile_robot and ros_client and ros_client.is_connected:  # noqa: F821
    mobile_robot.client = ros_client  # noqa: F821

    # CACHED, not rebuilt. A MobileRobotClient holds the live topic
    # subscriptions and the accumulated current_joint_values. Constructing a
    # fresh one each solve threw both away: joint states filled up in one
    # instance and get_current_configuration() was then called on the next,
    # empty one, so the arm always read back as all zeros. With a recompute
    # timer running, the client was replaced faster than any value could be
    # read. The orphaned instances also kept their subscriptions alive,
    # leaking one more on every solve.
    mobile_client = st.get(mobile_client_key)
    if mobile_client is None or mobile_client.ros_client is not ros_client:  # noqa: F821
        mobile_client = MobileRobotClient(ros_client)  # noqa: F821
        st[mobile_client_key] = mobile_client
    mobile_robot.mobile_client = mobile_client

    planner = st.get(planner_key)
    if planner is not None and planner.client is not ros_client:  # noqa: F821
        planner = None  # client was reconnected
    try:
        if planner is None:
            # attach_planner builds a MoveItPlanner and uploads the cell. Its
            # constructor resets MoveIt's planning scene, so this must not run
            # on every solve.
            #
            # MOVEIT_NAMESPACE, not `prefix` -- see the note at the top. The
            # topics are namespaced on this robot but the services are not.
            planner = mobile_robot.attach_planner(  # noqa: F821
                ros_client, namespace=MOVEIT_NAMESPACE  # noqa: F821
            )
            st[planner_key] = planner
            planner_note = "planner attached (MoveIt services under '%s'), cell uploaded" % (
                "/" + MOVEIT_NAMESPACE.strip("/") if MOVEIT_NAMESPACE else "/"
            )
        else:
            mobile_robot.planner = planner
            planner_note = "planner reused from cache"
    except Exception as e:
        planner_note = "PLANNER FAILED: %s: %s (is move_group running?)" % (type(e).__name__, e)

# --------------------------------------------------------------------------
# Report. Always says something, whether or not the button is down.
# --------------------------------------------------------------------------
status = st.get(status_key)
if not status:
    print("No load attempted yet. Press `load`.")
else:
    print("[%s] %s" % (status["when"], status["headline"]))
    for line in status["detail"]:
        print("    %s" % line)
    if not status["ok"] and status.get("exc"):
        print("    (%s)" % status["exc"])

print("")
# "sticky", not "cache" -- there are now two and conflating them sent you
# looking in the wrong place. Sticky holds the live object for this Rhino
# session; the disk cache survives restarts.
print("robot in sticky: %s" % bool(mobile_robot))
print("%s" % planner_note)
if mobile_robot:
    source = mobile_robot.attributes.get("cell_source", "unknown")
    if source == "cache":
        print("cell source    : DISK CACHE -- the robot was not reached")
    else:
        print("cell source    : %s" % source)
    tools = ", ".join(mobile_robot.robot_cell.tool_ids) or "none"
    print("tools attached : %s" % tools)

    mc = mobile_robot.mobile_client
    if mc is None:
        print("joint states   : no mobile_client")
    else:
        values = mc.current_joint_values
        subscribed = [t for t in mc.topics if "joint_state" in t]
        if values:
            print("joint states   : %d joints held, e.g. %s"
                  % (len(values), ", ".join(sorted(values)[:3])))
        elif subscribed:
            print("joint states   : subscribed to %s but NOTHING received yet"
                  % ", ".join(subscribed))
        else:
            print("joint states   : not subscribed (press subscribe on `get joint states`)")

print("")
for line in robot_cell_cache.describe(key):
    print(line if line.startswith(" ") else "disk %s" % line)

# --------------------------------------------------------------------------
# Outputs
# --------------------------------------------------------------------------
robot_cell = mobile_robot.robot_cell if mobile_robot else None
cell_state = mobile_robot.robot_cell_state if mobile_robot else None

meshes = None
if mobile_robot and mobile_robot.scene_object:
    # display_cell_state puts robot_base_frame at the BCF so the mobile base is
    # drawn where it actually is; the planning state keeps it at worldXY.
    meshes = mobile_robot.scene_object.draw(mobile_robot.display_cell_state())
