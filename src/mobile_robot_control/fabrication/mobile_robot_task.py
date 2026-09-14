"""Fabrication tasks for the mobile robot.

COMPAS FAB v2.0.1

Planning moved off the robot and onto a planner in compas_fab 2.x. The goal
constraints these tasks used to build by hand (``constraints_from_configuration``
/ ``constraints_from_frame``) are now ``Target`` objects, and the call goes
through ``MobileRobot.plan_motion_to_*``, which wraps
``MoveItPlanner.plan_motion``. The ``robot`` passed to these tasks must
therefore have a planner attached -- see ``MobileRobot.attach_planner``.
"""

from fabrication_manager.task import Task

from ur_fabrication_control.direct_control.fabrication import URTask
from ur_fabrication_control.direct_control.mixins import URScript, URScript_AreaGrip

from compas_rhino.conversions import frame_to_rhino_plane
from compas_robots import Configuration
from compas.geometry import Frame, Transformation, Point, Vector
import json

import time
import math

# Joint order the Vogui expects in the UR script and on the lift controller.
#
# Sourced from MobileRobotClient so there is one definition of these names. The
# lift joint was 'robot_ewellix_lift_top_joint' under robotnik_description; the
# robot now uses ewellix_description, where the lift is two joints with
# different names. A stale name here does not merely fail -- if it reaches
# MoveIt's /apply_planning_scene it aborts move_group.
from mobile_robot_control.mobile_robot_client import MobileRobotClient  # noqa: E402

MOBILE_ROBOT_JOINT_NAMES = (
    list(MobileRobotClient.LIFT_JOINT_NAMES) + list(MobileRobotClient.ARM_JOINT_NAMES)
)


def _reorder_configuration(robot, trajectory_point, trajectory, group):
    """Expand a group trajectory point to the mobile robot's joint order.

    Joints absent from the merged configuration are skipped rather than
    raising. ``list.index`` throws ValueError on a missing name, which turned a
    joint-naming mismatch into an unrelated-looking error deep in the task
    loop; the names come from the robot's URDF and have changed once already.
    """
    config = robot.full_configuration(trajectory_point, trajectory.start_configuration)

    names, values, types = [], [], []
    for joint_name in MOBILE_ROBOT_JOINT_NAMES:
        if joint_name not in config.joint_names:
            continue
        index = config.joint_names.index(joint_name)
        names.append(joint_name)
        values.append(config.joint_values[index])
        types.append(config.joint_types[index])

    return Configuration(values, types, names)

__all__ = [
    "MoveJointsTask",
    "MoveLinearTask",
    "MotionPlanConfigurationTask",
    "MotionPlanFrameTask",
    "InverseKinematicsTask",
    "GetConfigurationTask",
    "ExecuteMotionTask",
    "SearchAndSaveMarkersTask",
    "GetMarkerPoseTask",
    "FixRobotToMarkerTask",
]

### Motion plan tasks ###

class MotionPlanConfigurationTask(Task):
    def __init__(
        self,
        robot,
        target_configuration,
        start_configuration,
        group="ur20",
        tolerance_above=[math.radians(1)] * 6,
        tolerance_below=[math.radians(1)] * 6,
        path_constraints=None,
        planner_id="RRTConnect",
        validation=True,
        key=None,
    ):
        super(MotionPlanConfigurationTask, self).__init__(key)
        self.robot = robot
        self.group = group
        self.target_configuration = target_configuration
        self.start_configuration = start_configuration

        self.tolerance_above = tolerance_above
        self.tolerance_below = tolerance_below

        self.path_constraints = path_constraints
        self.planner_id = planner_id

        self.trajectory = None
        self.results = {
            "configurations": [],
            "planes": [],
            "positions": [],
            "velocities": [],
            "accelerations": [],
        }

        self.validation = validation
        self.replan = False
        self.approved = False

    def run(self, stop_thread):
        self.log("Planning trajectory...")

        while not stop_thread():
            # Clean trajectory.
            self.replan = False
            self.trajectory = None
            self.results = {
                "configurations": [],
                "planes": [],
                "positions": [],
                "velocities": [],
                "accelerations": [],
                }

            self.trajectory = self.robot.plan_motion_to_configuration(
                self.target_configuration,
                start_configuration=self.start_configuration,
                group=self.group,
                tolerance_above=self.tolerance_above,
                tolerance_below=self.tolerance_below,
                options=dict(
                    path_constraints=self.path_constraints,
                    planner_id=self.planner_id,
                ),
            )

            while not stop_thread():
                if self.trajectory is not None:
                    break
                time.sleep(0.1)

            self.log("Trajectory found at {}.".format(self.trajectory))

            for c in self.trajectory.points:
                self.results["configurations"].append(
                    _reorder_configuration(self.robot, c, self.trajectory, self.group)
                )

                frame_t = self.robot.forward_kinematics(c, self.group)
                self.results["planes"].append(
                    frame_to_rhino_plane(self.robot.from_BCF_to_WCF(frame_t))
                )
                self.results["positions"].append(c.positions)
                self.results["velocities"].append(c.velocities)
                self.results["accelerations"].append(c.accelerations)

            if self.validation:
                # Wait until trajectory is approved or replan is requested.
                while not stop_thread():
                    time.sleep(0.1)
                    if self.approved == True or self.replan == True:
                        break
                # Break if trajectory is approved.
                if self.approved == True:
                    break
                time.sleep(0.1)
            else:
                break
                
        self.is_completed = True
        return True

class MotionPlanFrameTask(Task):
    def __init__(
        self,
        robot,
        frame_WCF,
        start_configuration,
        group="ur20",
        tolerance_position=0.001,
        tolerance_orientation=1.0,
        path_constraints=None,
        planner_id="RRTConnect",
        validation=True,
        key=None,
    ):
        super(MotionPlanFrameTask, self).__init__(key)
        self.robot = robot
        self.group = group
        self.frame_WCF = frame_WCF
        self.start_configuration = start_configuration

        self.tolerance_position = tolerance_position
        # v1 took one tolerance per axis; FrameTarget takes a single orientation
        # tolerance that the planner applies to all three axes.
        self.tolerance_orientation = math.radians(tolerance_orientation)

        self.path_constraints = path_constraints
        self.planner_id = planner_id

        self.trajectory = None
        self.results = {
            "configurations": [],
            "planes": [],
            "positions": [],
            "velocities": [],
            "accelerations": [],
        }

        self.validation = validation
        self.replan = False
        self.approved = False

    def run(self, stop_thread):
        self.log("Planning trajectory...")

        while not stop_thread():
            # Clean trajectory.
            self.replan = False
            self.trajectory = None
            self.results = {
                "configurations": [],
                "planes": [],
                "positions": [],
                "velocities": [],
                "accelerations": [],
                }

            # The WCF -> BCF conversion happens inside plan_motion_to_frame.
            self.trajectory = self.robot.plan_motion_to_frame(
                self.frame_WCF,
                start_configuration=self.start_configuration,
                group=self.group,
                tolerance_position=self.tolerance_position,
                tolerance_orientation=self.tolerance_orientation,
                options=dict(
                    path_constraints=self.path_constraints,
                    planner_id=self.planner_id,
                ),
            )

            while not stop_thread():
                if self.trajectory is not None:
                    break
                time.sleep(0.1)

            self.log("Trajectory found at {}.".format(self.trajectory))

            for c in self.trajectory.points:
                self.results["configurations"].append(
                    _reorder_configuration(self.robot, c, self.trajectory, self.group)
                )

                frame_t = self.robot.forward_kinematics(c, self.group)
                self.results["planes"].append(
                    frame_to_rhino_plane(self.robot.from_BCF_to_WCF(frame_t))
                )
                self.results["positions"].append(c.positions)
                self.results["velocities"].append(c.velocities)
                self.results["accelerations"].append(c.accelerations)

            if self.validation:
                # Wait until trajectory is approved or replan is requested.
                while not stop_thread():
                    time.sleep(0.1)
                    if self.approved == True or self.replan == True:
                        break
                # Break if trajectory is approved.
                if self.approved == True:
                    break
                time.sleep(0.1)
            else:
                break
                
        self.is_completed = True
        return True

class InverseKinematicsTask(Task):
    def __init__(
        self,
        robot,
        frame_WCF,
        start_configuration,
        group="ur10e",
        json_path=None,
        key=None,
    ):
        super(InverseKinematicsTask, self).__init__(key)
        self.robot = robot
        self.frame_WCF = frame_WCF
        self.start_configuration = start_configuration
        self.group = group
        self.configuration = None
        self.path = json_path

    def run(self, stop_thread):
        self.log("Computing inverse kinematics...")
        # The WCF -> BCF conversion happens inside MobileRobot.inverse_kinematics.
        self.configuration = self.robot.inverse_kinematics(
            self.frame_WCF, self.start_configuration, self.group
        )

        while not stop_thread():
            if self.configuration is not None:
                break
            time.sleep(0.1)

        self.log("Configuration found at {}.".format(self.configuration))
        filename = "Task_{}.json".format(self.key)
        filepath = self.path / filename
        # compas 2.x: Data.to_data() was replaced by the __data__ property.
        json_data = json.dumps(self.configuration.__data__)

        with open(filepath, "w") as f:
            f.write(json_data)

        self.is_completed = True
        return True

class GetConfigurationTask(Task):
    def __init__(self, robot, key=None):
        super(GetConfigurationTask, self).__init__(key)
        self.robot = robot
        self.configuration = None

    def run(self, stop_thread):
        self.log("Waiting for current configuration...")
        current_joint_values = self.robot.mobile_client.current_joint_values

        # NOTE: this used to spell the lift joint "robot_ewellix_top_lift_joint",
        # which never matched the joint state and so always read back as 0.0.
        joint_values_ordered = [
            current_joint_values.get(joint_name, 0.00000)
            for joint_name in MOBILE_ROBOT_JOINT_NAMES
        ]
        joint_types_ordered = [2, 0, 0, 0, 0, 0, 0]
        self.configuration = Configuration(
            joint_values_ordered, joint_types_ordered, MOBILE_ROBOT_JOINT_NAMES
        )

        self.log("Current configuration is: {}".format(self.configuration))

        self.is_completed = True
        return True

class ExecuteMotionTask(URTask):
    def __init__(self, robot, robot_address, fabrication, motiontask_key=0, reverse=False, velocity=0.06, radius=0.01, payload=0.0, CoG=[0,0,0], key=None):
        super(ExecuteMotionTask, self).__init__(robot, robot_address, key)
        self.robot = robot
        self.robot_address = robot_address
        self.fabrication = fabrication
        self.motiontask_key = motiontask_key
        self.velocity = velocity
        self.radius = radius
        self.configurations = None
        self.reverse = reverse
        self.payload = payload
        self.CoG = CoG
                
    def create_urscript(self):
        self.log("Executing the planned motion!")
        # Get the motion plan from 1 task before.
        motionplan_task = self.fabrication.get_task_by_key(self.motiontask_key)
        configurations = motionplan_task.results.get("configurations")
        frame_WCF = motionplan_task.frame_WCF
        if self.reverse:
            self.configurations = configurations[::-1]
        else:
            self.configurations = configurations

        self.urscript.set_payload(self.payload, self.CoG)
        self.urscript.add_line("textmsg(\">> TASK {}.\")".format(self.key))

        for config in self.configurations:
            joint_configuration = Configuration.from_revolute_values(config.revolute_values)
            self.urscript.move_joint(joint_configuration, self.velocity, self.radius)

        # Go to the target frame with radius 0.
        self.urscript.add_line("\tsleep({})".format(1.0))
        self.urscript.move_linear(frame=self.robot.from_WCF_to_RCF(frame_WCF), velocity=self.velocity/2, radius=0.0)
        #self.log(self.urscript.commands)

### UR direct tasks ###

class MoveJointsTask(URTask):
    def __init__(
        self,
        robot,
        robot_address,
        configuration,
        velocity=0.10,
        radius=0.1,
        payload=0.0,
        CoG=[0.0, 0.0, 0.0],
        key=None,
    ):
        super(MoveJointsTask, self).__init__(robot, robot_address, key)
        self.configuration = configuration
        self.velocity = velocity
        self.radius = radius
        self.payload = payload
        self.CoG = CoG

    def create_urscript(self):
        self.urscript.set_payload(self.payload, self.CoG)
        self.urscript.add_line('textmsg(">> TASK{}.")'.format(self.key))

        joint_configuration = Configuration.from_revolute_values(self.configuration.revolute_values)
        
        self.urscript.move_joint(joint_configuration, self.velocity, self.radius)
        self.log("Going to set configuration {}.".format(self.configuration))

class MoveLinearTask(URTask):
    def __init__(
        self,
        robot,
        robot_address,
        frame,
        in_RCF=True,
        velocity=0.10,
        radius=0.0,
        payload=0.0,
        CoG=[0.0, 0.0, 0.0],
        key=None,
    ):
        super(MoveLinearTask, self).__init__(robot, robot_address, key)
        self.robot = robot
        self.robot_address = robot_address
        self.frame = frame
        self.in_RCF = in_RCF
        self.velocity = velocity
        self.radius = radius
        self.payload = payload
        self.CoG = CoG

    def create_urscript(self):
        if not self.in_RCF:
            frame_RCF = self.frame.transformed(self.robot.transformation_WCF_RCF())
        else:
            frame_RCF = self.frame

        self.urscript.set_payload(self.payload, self.CoG)
        self.urscript.add_line('textmsg(">> TASK{}.")'.format(self.key))

        self.urscript.move_linear(frame_RCF, self.velocity, self.radius)

        self.log("Going to frame {}.".format(self.frame))

### Marker related tasks ###

class SearchAndSaveMarkersTask(Task):
    def __init__(
        self, robot, robot_address, fabrication, duration=10, update=True, key=None
    ):
        super(SearchAndSaveMarkersTask, self).__init__(key)
        self.robot = robot
        self.robot_address = robot_address
        self.fabrication = fabrication
        self.duration = duration
        self.update = update
        self.marker_ids = []

    def receive_marker_ids(self, message):
        msg = message.get("transforms")[0]
        if msg.get("header").get("frame_id") == "camera_color_optical_frame":
            marker_id = msg.get("child_frame_id")
            if marker_id not in self.marker_ids:
                self.log("Found marker with ID: {}".format(marker_id))
                if (self.update) or (
                    not self.update
                    and not self.robot.mobile_client.marker_frames.get(marker_id)
                ):
                    self.marker_ids.append(marker_id)
                else:
                    self.log(
                        "Ignoring {}, as it is already recorded in the marker dictionary and update is set to False.".format(
                            marker_id
                        )
                    )

    def run(self, stop_thread):
        self.marker_ids = []
        # Get the marker ids in the scene
        self.robot.mobile_client.topic_subscribe(
            "/tf", "tf2_msgs/TFMessage", self.receive_marker_ids
        )
        t0 = time.time()
        while time.time() - t0 < self.duration and not stop_thread():
            time.sleep(0.1)
        self.robot.mobile_client.topic_unsubscribe("/tf")
        self.log("Got all the visible marker ids.")
        time.sleep(1)
        self.log("Length of the list is {}.".format(len(self.marker_ids)))

        # Iterate the marker ids.
        if len(self.marker_ids) > 0:
            for marker_id in self.marker_ids:
                next_key = self.fabrication.get_next_task_key()
                task = GetMarkerPoseTask(
                    self.robot,
                    marker_id=marker_id,
                    reference_frame_id="robot_arm_base",
                    key=next_key,
                )
                self.fabrication.add_task(task, key=next_key)
        else:
            self.log("No more markers are visible.")

        self.is_completed = True
        return True

class SearchAndSaveRobotPoseInMarkerTask(Task):
    def __init__(
        self, robot, robot_address, fabrication, duration=10, update=True, key=None
    ):
        super(SearchAndSaveRobotPoseInMarkerTask, self).__init__(key)
        self.robot = robot
        self.robot_address = robot_address
        self.fabrication = fabrication
        self.duration = duration
        self.update = update
        self.marker_ids = []

    def receive_marker_ids(self, message):
        msg = message.get("transforms")[0]
        if msg.get("header").get("frame_id") == "camera_color_optical_frame":
            marker_id = msg.get("child_frame_id")
            if marker_id not in self.marker_ids:
                self.log("Found marker with ID: {}".format(marker_id))
                if (self.update) or (
                    not self.update
                    and not self.robot.mobile_client.marker_frames.get(marker_id)
                ):
                    self.marker_ids.append(marker_id)
                else:
                    self.log(
                        "Ignoring {}, as it is already recorded in the marker dictionary and update is set to False.".format(
                            marker_id
                        )
                    )

    def run(self, stop_thread):
        self.marker_ids = []
        # Get the marker ids in the scene
        self.robot.mobile_client.topic_subscribe(
            "/tf", "tf2_msgs/TFMessage", self.receive_marker_ids
        )
        t0 = time.time()
        while time.time() - t0 < self.duration and not stop_thread():
            time.sleep(0.1)
        self.robot.mobile_client.topic_unsubscribe("/tf")
        self.log("Got all the visible marker ids.")
        time.sleep(1)
        self.log("Length of the list is {}.".format(len(self.marker_ids)))

        # Iterate the marker ids.
        if len(self.marker_ids) > 0:
            for marker_id in self.marker_ids:
                next_key = self.fabrication.get_next_task_key()
                task = GetRobotPoseInMarkerPoseTask(
                    self.robot,
                    marker_id=marker_id,
                    reference_frame_id="robot_arm_base",
                    key=next_key,
                )
                self.fabrication.add_task(task, key=next_key)
        else:
            self.log("No more markers are visible.")

        self.is_completed = True
        return True

class GetRobotPoseInMarkerPoseTask(Task):
    def __init__(
        self, robot, marker_id="marker_0", reference_frame_id="robot_arm_base", key=None
    ):
        super(GetRobotPoseInMarkerPoseTask, self).__init__(key)
        self.robot = robot
        self.marker_id = marker_id
        self.reference_frame_id = reference_frame_id

    def run(self, stop_thread):
        self.robot.mobile_client.clean_tf_frame()
        self.robot.mobile_client.tf_subscribe(self.marker_id, self.reference_frame_id)
        t0 = time.time()
        while (
            time.time() - t0 < 20 and not stop_thread()
        ):  # can be used for live subscription when time limit is removed.
            time.sleep(0.1)
            if self.robot.mobile_client.tf_frame is not None:
                MCF_in_RCF = Frame(
                    self.robot.mobile_client.tf_frame.point,
                    self.robot.mobile_client.tf_frame.zaxis,
                    -self.robot.mobile_client.tf_frame.yaxis,
                )
                MCF_in_BCF = MCF_in_RCF.transformed(self.robot.transformation_RCF_BCF())
                BCF_in_MCF = Frame.from_transformation(
                    Transformation.from_frame(MCF_in_BCF).inverted()
                )  # Invert
                self.log(
                    "Robot base frame in reference to {} is {}.".format(
                        self.marker_id, BCF_in_MCF
                    )
                )

                # Marker frames are added to the dict in WCF.
                self.robot.mobile_client.marker_frames[self.marker_id] = BCF_in_MCF
                break
        if self.robot.mobile_client.tf_frame is None:
            self.log("For {}, could not get the frame.".format(self.marker_id))
        self.robot.mobile_client.tf_unsubscribe(self.marker_id, self.reference_frame_id)
        self.is_completed = True
        return True

class GetMarkerPoseTask(Task):
    def __init__(
        self, robot, marker_id="marker_0", reference_frame_id="robot_arm_base", key=None
    ):
        super(GetMarkerPoseTask, self).__init__(key)
        self.robot = robot
        self.marker_id = marker_id
        self.reference_frame_id = reference_frame_id

    def run(self, stop_thread):
        self.robot.mobile_client.clean_tf_frame()
        self.robot.mobile_client.tf_subscribe(self.marker_id, self.reference_frame_id)
        t0 = time.time()
        while (
            time.time() - t0 < 20 and not stop_thread()
        ):  # can be used for live subscription when time limit is removed.
            time.sleep(0.1)

            if self.robot.mobile_client.tf_frame is not None:
                MCF_in_RCF = Frame(
                    self.robot.mobile_client.tf_frame.point,
                    self.robot.mobile_client.tf_frame.zaxis,
                    -self.robot.mobile_client.tf_frame.yaxis,
                )
                MCF_in_BCF = MCF_in_RCF.transformed(self.robot.transformation_RCF_BCF())
                self.log(
                    "{} pose in reference to robot base frame is {}.".format(
                        self.marker_id, MCF_in_BCF
                    )
                )
                # Marker frames are added to the dict in WCF.
                self.robot.mobile_client.marker_frames[self.marker_id] = MCF_in_BCF
                break
        if self.robot.mobile_client.tf_frame is None:
            self.log("For {}, could not get the frame.".format(self.marker_id))
        self.robot.mobile_client.tf_unsubscribe(self.marker_id, self.reference_frame_id)
        self.is_completed = True
        return True

class FixRobotToMarkerTask(Task):
    def __init__(self, robot, fixed_marker_id="marker_0", key=None):
        super(FixRobotToMarkerTask, self).__init__(key)
        self.robot = robot
        self.fixed_marker_id = fixed_marker_id
        self.marker_pose = None

    def run(self, stop_thread):
        # Get the frame of the fixed marker id.
        self.robot.mobile_client.clean_tf_frame()
        self.robot.mobile_client.tf_subscribe(self.fixed_marker_id, "robot_arm_base")
        t0 = time.time()
        while (
            time.time() - t0 < 20 and not stop_thread()
        ):  # can be used for live subscription when time limit is removed.
            time.sleep(0.1)
            if self.robot.mobile_client.tf_frame is not None:
                self.log(
                    "For {}, got the frame: {}".format(
                        self.fixed_marker_id, self.robot.mobile_client.tf_frame
                    )
                )
                self.marker_pose = self.robot.mobile_client.tf_frame
                break
        if self.robot.mobile_client.tf_frame is None:
            self.log("For {}, could not get the frame.".format(self.fixed_marker_id))
        self.robot.mobile_client.tf_unsubscribe(self.fixed_marker_id, "robot_arm_base")

        # Fix the robot to the marker pose
        if self.marker_pose is not None:
            MCF_in_RCF = self.marker_pose  # marker frame in RCF
            MCF_in_BCF = MCF_in_RCF.transformed(
                self.robot.transformation_RCF_BCF()
            )  # marker frame in BCF
            BCF_in_MCF = Frame.from_transformation(
                Transformation.from_frame(MCF_in_BCF).inverted()
            )  # BCF in measured MCF
            MCF_in_WCF = self.robot.mobile_client.marker_frames[
                self.fixed_marker_id
            ]  # marker frame in WCF
            from_MCF_to_WCF = Transformation.from_change_of_basis(
                MCF_in_WCF, Frame.worldXY()
            )  # T from fixed MCF to WCF

            BCF_in_WCF = BCF_in_MCF.transformed(from_MCF_to_WCF)  # BCF in WCF

            self.robot.BCF = BCF_in_WCF
            self.log("Robot is fixed to {}.".format(self.fixed_marker_id))
        else:
            self.log("Fixed marker frame is not retrieved.")

        self.is_completed = True
        return True


if __name__ == "__main__":
    pass
