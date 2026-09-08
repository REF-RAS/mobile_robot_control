"""Set the robot's base frame (BCF).

COMPAS FAB v2.0.1

Inputs : robot, set_frame_to, target_BCF, target_RCF
Outputs: out, bang

`BCF` is a local, not an output -- the component has no such parameter. Add one
named BCF if you want the resolved frame downstream.

Unchanged in substance: MobileRobot keeps the BCF/RCF algebra it always had.
The v1 version imported CollisionMesh and draw_frame without using either, and
CollisionMesh no longer exists in 2.x, so the dead imports had to go.

TWO GUARDS THE V1 VERSION LACKED
--------------------------------
`set_to_RCF` calls robot.transform_frame_from_RCF_to_BCF before anything has
checked that `robot` exists, so an unwired robot input raised AttributeError on
a line that reads as if it were guarded.

That call also depends on robot.RCF, which is populated by a TF subscription
through mobile_client -- it is None until the robot is connected and the
transform has arrived. Reaching it too early gave a Transformation error from
inside compas rather than anything pointing at the cause, so it is now checked
and reported.
"""

from compas.geometry import Frame
from compas_rhino.conversions import plane_to_compas_frame

BCF = None
bang = None

if not robot:  # noqa: F821
    print("No robot wired.")

elif set_frame_to == "set_to_BCF":  # noqa: F821
    if target_BCF:  # noqa: F821
        BCF = plane_to_compas_frame(target_BCF)  # noqa: F821
    else:
        print("set_to_BCF selected but target_BCF is empty.")

elif set_frame_to == "set_to_RCF":  # noqa: F821
    if not target_RCF:  # noqa: F821
        print("set_to_RCF selected but target_RCF is empty.")
    elif robot.RCF is None:  # noqa: F821
        # RCF arrives over TF via mobile_client; it is None until the robot is
        # connected and the first transform has landed.
        print("robot.RCF is not available yet -- is the client connected?")
        print("It is populated by a TF subscription (robot_arm_base ->")
        print("robot_base_footprint), so it needs mobile_client and a live ROS link.")
    else:
        RCF = plane_to_compas_frame(target_RCF)  # noqa: F821
        BCF = robot.transform_frame_from_RCF_to_BCF(RCF)  # noqa: F821

else:
    BCF = Frame.worldXY()

if robot and BCF is not None:  # noqa: F821
    robot.BCF = BCF  # noqa: F821
    print("BCF set to %s" % BCF)
    # Recompute trigger for the Visualise component.
    bang = True
