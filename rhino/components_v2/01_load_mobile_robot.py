"""Load the mobile robot cell directly from ROS 2.

COMPAS FAB v2.0.1

Inputs
------
ros_client  : a connected compas_fab.backends.RosClient
load        : Button -- re-fetch the cell from ROS (slow: downloads meshes)
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


# --------------------------------------------------------------------------
# Load the cell from ROS. Cached: this is the slow, network-bound step.
# --------------------------------------------------------------------------
if load:  # noqa: F821
    url = base_url()
    if not (ros_client and ros_client.is_connected):  # noqa: F821
        st[status_key] = {
            "ok": False, "when": time.strftime("%H:%M:%S"),
            "headline": "NOT CONNECTED TO ROS",
            "detail": ["Connect the RosClient component first."],
        }
    else:
        reachable, detail = check_file_server(url)
        try:
            if not reachable:
                raise URLError(detail)

            robot_cell = ros_client.load_robot_cell(  # noqa: F821
                load_geometry=True,
                urdf_param_name="{}/robot_description".format(prefix),
                srdf_param_name="{}/robot_description_semantic".format(prefix),
                http_file_server_base_url=file_server,
            )
            mobile_robot = MobileRobot(
                robot_cell,
                robot_cell_state=robot_cell.default_cell_state(),
                scene_object=SceneObject(item=robot_cell, sceneobject_type=RobotCellObject),
            )
            st[key] = mobile_robot
            st.pop(planner_key, None)  # a new cell must be re-uploaded to MoveIt

            links = len(robot_cell.robot_model.links)
            st[status_key] = {
                "ok": True, "when": time.strftime("%H:%M:%S"),
                "headline": "LOADED '%s'" % robot_cell.robot_model.name,
                "detail": [
                    "%d links, groups: %s" % (links, ", ".join(robot_cell.group_names)),
                    "mesh server: %s (%s)" % (url, detail),
                ],
            }
        except Exception as e:
            headline, detail_lines = diagnose(e, url)
            st[status_key] = {
                "ok": False, "when": time.strftime("%H:%M:%S"),
                "headline": headline, "detail": detail_lines,
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
            # `namespace` matters: compas_fab hardcodes unnamespaced MoveIt
            # service names (/plan_kinematic_path and six others). move_group
            # runs under /robot here, so without the prefix every call waits on
            # a service nobody provides and times out -- which reads as
            # "move_group is down" rather than "wrong service name".
            planner = mobile_robot.attach_planner(ros_client, namespace=prefix)  # noqa: F821
            st[planner_key] = planner
            planner_note = "planner attached, services under '%s', cell uploaded" % (
                "/" + prefix.strip("/") if prefix else "(no namespace)"
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
print("robot in cache : %s" % bool(mobile_robot))
print("%s" % planner_note)
if mobile_robot:
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
