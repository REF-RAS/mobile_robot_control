"""Load the mobile robot cell directly from ROS 2.

COMPAS FAB v2.0.1

Grasshopper component script.

Inputs
------
ros_client  : a connected compas_fab.backends.RosClient
load        : bool, re-fetch the cell from ROS (slow: downloads meshes)
prefix      : str, ROS namespace -- 'robot' on this machine
file_server : str, HTTP mesh server, e.g. 'http://192.168.0.200:9190'

Outputs
-------
mobile_robot : MobileRobot
robot_cell   : compas_fab.robots.RobotCell
cell_state   : compas_fab.robots.RobotCellState
meshes       : the drawn robot geometry

Note
----
Editing mobile_robot_control requires a Rhino restart to take effect: Python
caches the module in sys.modules, and the MobileRobot held in sticky is an
instance of the class object that was loaded at the time. Restarting clears
both.
"""

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
# pointed at AB do not share (and overwrite) each other's cached cell. Carried
# over from the v1 script as 'robot_AA'; this workspace targets AB.
key = "robot_AB"
planner_key = key + "_planner"

prefix = prefix or ""  # noqa: F821
file_server = file_server or None  # noqa: F821

# --------------------------------------------------------------------------
# Load the cell from ROS. Cached: this is the slow, network-bound step.
# --------------------------------------------------------------------------
if ros_client and ros_client.is_connected and load:  # noqa: F821
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

mobile_robot = st.get(key, None)

# --------------------------------------------------------------------------
# Attach the client and planner. A client can be restarted without reloading
# geometry, so this is kept separate from the load above.
# --------------------------------------------------------------------------
if mobile_robot and ros_client and ros_client.is_connected:  # noqa: F821
    mobile_robot.client = ros_client  # noqa: F821
    mobile_robot.mobile_client = MobileRobotClient(ros_client)  # noqa: F821

    planner = st.get(planner_key)
    if planner is not None and planner.client is not ros_client:  # noqa: F821
        planner = None  # client was reconnected
    if planner is None:
        # attach_planner builds a MoveItPlanner and uploads the cell. The
        # MoveItPlanner constructor resets MoveIt's planning scene, so this
        # must not run on every solve.
        planner = mobile_robot.attach_planner(ros_client)  # noqa: F821
        st[planner_key] = planner
    else:
        mobile_robot.planner = planner

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
