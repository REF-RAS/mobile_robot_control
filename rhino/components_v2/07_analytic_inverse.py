"""Analytic inverse kinematics for the UR20, in the arm's own frame.

COMPAS FAB v2.0.1

Inputs : robot, plane, idx (int, item)
Outputs: out, configuration, plane_RCF, plane_tool0_RCF

Local UR kinematics -- no MoveIt, no ROS. Solves the closed-form solutions for
the UR20 and picks one by index.

THERE IS NO `lift` INPUT, DELIBERATELY
--------------------------------------
The lift position is a measured fact about the robot, not a choice. It comes
from `robot_lift_lower_joint` on /robot/joint_states, and nothing else is a
legitimate source.

v1 took it as an input, wired on this canvas to a slider in the forward
kinematics group. That slider states an intent; it does not constrain the
robot. The returned configuration would claim a lift height the robot was not
at, and execution could not correct it -- URScript's movej drives the six arm
joints only, so the lift simply stays where it is while the arm solves as
though it were somewhere else.

With no input there is no way to express a lift height that is not true. If the
measurement is absent, no configuration is produced and the reason is stated.
A missing encoder reading is a fault to surface, not a gap to fill with a
plausible number.

THE LIFT WAS ALSO COUNTED TWICE
-------------------------------
v1 did:

    frame_RCF = robot.from_WCF_to_RCF(frame_WCF)
    frame_RCF.transform(Translation.from_vector([0, 0, -lift]))

robot.RCF is the arm base frame and already sits at the real lift height --
it is URDF forward kinematics over the live joint states, so it rises and falls
with the lift. Subtracting `lift` again placed the IK target that far below the
intended one: at a lift of 0.26, a pose 260 mm low. The subtraction is gone.

NOT YET VERIFIED ON THE ROBOT. Confirm `plane_RCF` against a known target at a
known lift height before executing anything from this.

ALSO: v1 used from_prismatic_and_revolute_values(), which returns a
configuration with no joint NAMES. MobileRobot.full_configuration merges by
name, so an unnamed configuration cannot merge into a full robot state. The
joints are named here.
"""

from compas_robots import Configuration
from compas_ghpython.drawing import draw_frame
from compas_rhino.conversions import plane_to_compas_frame
from ur_fabrication_control.kinematics.ur_kinematics import inverse_kinematics
from ur_fabrication_control.kinematics.ur_params import ur_params

from mobile_robot_control.mobile_robot_client import MobileRobotClient

configuration = None
plane_RCF = None
plane_tool0_RCF = None

LIFT_JOINT = MobileRobotClient.LIFT_JOINT_NAMES[0]

if not robot:  # noqa: F821
    print("No robot wired.")

elif not plane:  # noqa: F821
    print("No plane wired.")

elif robot.RCF is None:  # noqa: F821
    print("robot.RCF is unavailable -- the arm base frame cannot be resolved.")
    print("   It is computed from the URDF using live joint states; check the")
    print("   robot is loaded and ARM_BASE_LINK matches the model.")

else:
    # The target frame does not depend on the lift, so these are always
    # produced -- useful for checking the transform even when the encoder
    # reading is missing.
    frame_WCF = plane_to_compas_frame(plane)  # noqa: F821
    frame_RCF = robot.from_WCF_to_RCF(frame_WCF)  # noqa: F821

    if robot.attached_tool:  # noqa: F821
        tool0_RCF = robot.from_tcf_to_t0cf([frame_RCF])[0]  # noqa: F821
    else:
        tool0_RCF = frame_RCF

    plane_RCF = draw_frame(frame_RCF)
    plane_tool0_RCF = draw_frame(tool0_RCF)

    lift_value = None
    if robot.mobile_client:  # noqa: F821
        lift_value = robot.mobile_client.current_joint_values.get(LIFT_JOINT)  # noqa: F821

    if lift_value is None:
        print("NO LIFT MEASUREMENT -- no configuration produced.")
        print("")
        print("   '%s' has not arrived on /robot/joint_states." % LIFT_JOINT)
        print("   Press `subscribe` on `get joint states`, and check the liftkit")
        print("   is publishing: ros2 topic echo /robot/joint_states --once")
        print("")
        print("   There is no input to override this. The lift position is a")
        print("   measured fact, and a configuration built around an assumed")
        print("   one would be executed as though it were true.")
        print("")
        print("   plane_RCF and plane_tool0_RCF are still output, so the frame")
        print("   transform can be checked without it.")
    else:
        solutions = inverse_kinematics(tool0_RCF, ur_params["ur20"])

        if not solutions:
            print("NO IK SOLUTION for this plane -- out of reach, or the target")
            print("   lies in a singular region. Configuration not produced.")
            print("   lift was %.4f m (measured)." % lift_value)
        else:
            index = int(idx) if idx is not None else 0  # noqa: F821
            index = max(0, min(index, len(solutions) - 1))
            joint_values = solutions[index]

            # Named, because full_configuration merges by name.
            names = list(MobileRobotClient.LIFT_JOINT_NAMES) + list(MobileRobotClient.ARM_JOINT_NAMES)
            configuration = Configuration(
                [lift_value] + list(joint_values),
                [2] + [0] * len(joint_values),
                names,
            )

            print("solution %d of %d" % (index, len(solutions)))
            print("lift   : %.4f m  (measured from %s)" % (lift_value, LIFT_JOINT))
            print("joints : %s" % [round(v, 3) for v in joint_values])
