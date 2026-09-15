"""Analytic inverse kinematics for the UR20, in the arm's own frame.

COMPAS FAB v2.0.1

Inputs : robot, plane, idx (int, item), lift (float, item, optional)
Outputs: out, configuration, plane_RCF, plane_tool0_RCF

Local UR kinematics -- no MoveIt, no ROS. Solves the 8 closed-form solutions
for the UR20 and picks one by index.

TWO FIXES OVER THE V1 SCRIPT
----------------------------

1. THE LIFT WAS COUNTED TWICE

   v1 did:

       frame_RCF = robot.from_WCF_to_RCF(frame_WCF)
       frame_RCF.transform(Translation.from_vector([0, 0, -lift]))

   robot.RCF is the arm base frame, and it already sits at the real lift
   height -- it comes from URDF forward kinematics using the live joint
   states, so it rises and falls with the lift. Subtracting `lift` again put
   the IK target that far below where it should be. At a lift of 0.26 the arm
   would solve for a pose 260 mm low.

   The subtraction is gone. RCF alone places the target correctly.

2. THE LIFT VALUE WAS A FREE SLIDER

   `lift` feeds the prismatic slot of the returned configuration, so it has to
   be the lift's ACTUAL position. A slider is a statement of intent, and
   nothing made it agree with the robot -- the configuration would claim a
   lift height the robot was not at, and URScript execution would not correct
   it, because movej drives only the six arm joints.

   It is now read from live joint states when available, with the `lift` input
   as an explicit offline fallback. The output says which was used.

ALSO: v1 built the configuration with from_prismatic_and_revolute_values(),
which produces no joint NAMES. MobileRobot.full_configuration merges by name,
so an unnamed configuration cannot be merged into a full robot state. The
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

if not robot:  # noqa: F821
    print("No robot wired.")
elif not plane:  # noqa: F821
    print("No plane wired.")
elif robot.RCF is None:  # noqa: F821
    print("robot.RCF is unavailable -- the arm base frame cannot be resolved.")
    print("   It is computed from the URDF using live joint states; check the")
    print("   robot is loaded and ARM_BASE_LINK matches the model.")
else:
    # ---------------------------------------------------------------- lift
    lift_name = MobileRobotClient.LIFT_JOINT_NAMES[0]
    measured = None
    if robot.mobile_client:  # noqa: F821
        measured = robot.mobile_client.current_joint_values.get(lift_name)  # noqa: F821

    if measured is not None:
        lift_value, lift_source = measured, "measured (%s)" % lift_name
    elif lift is not None:  # noqa: F821
        lift_value, lift_source = float(lift), "wired `lift` input (NOT measured)"  # noqa: F821
    else:
        lift_value, lift_source = 0.0, "defaulted to 0.0 -- no measurement, nothing wired"

    # ------------------------------------------------------------- solve
    # No -lift translation here: robot.RCF already sits at the real lift
    # height, so subtracting it again would double-count.
    frame_WCF = plane_to_compas_frame(plane)  # noqa: F821
    frame_RCF = robot.from_WCF_to_RCF(frame_WCF)  # noqa: F821

    if robot.attached_tool:  # noqa: F821
        tool0_RCF = robot.from_tcf_to_t0cf([frame_RCF])[0]  # noqa: F821
    else:
        tool0_RCF = frame_RCF

    plane_RCF = draw_frame(frame_RCF)
    plane_tool0_RCF = draw_frame(tool0_RCF)

    solutions = inverse_kinematics(tool0_RCF, ur_params["ur20"])

    if not solutions:
        print("NO IK SOLUTION for this plane -- out of reach, or the target is")
        print("   inside the arm's singular region. Configuration not produced.")
    else:
        index = int(idx) if idx is not None else 0  # noqa: F821
        index = max(0, min(index, len(solutions) - 1))
        joint_values = solutions[index]

        # Name the joints. from_prismatic_and_revolute_values() leaves them
        # unnamed, and full_configuration merges by name -- an unnamed
        # configuration silently fails to merge into a full robot state.
        names = list(MobileRobotClient.LIFT_JOINT_NAMES) + list(MobileRobotClient.ARM_JOINT_NAMES)
        values = [lift_value] + list(joint_values)
        types = [2] + [0] * len(joint_values)
        configuration = Configuration(values, types, names)

        print("solution %d of %d" % (index, len(solutions)))
        print("lift  : %.4f  (%s)" % (lift_value, lift_source))
        print("joints: %s" % [round(v, 3) for v in joint_values])
        if measured is None:
            print("")
            print("WARNING: the lift value is not measured. The returned")
            print("   configuration claims a lift height the robot may not be at,")
            print("   and movej will not correct it -- URScript drives only the")
            print("   six arm joints. Subscribe to joint states before executing.")
