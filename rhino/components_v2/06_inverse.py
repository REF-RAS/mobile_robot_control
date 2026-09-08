"""Calculate the robot's inverse kinematics for a given plane.

COMPAS FAB v2.0.1

Inputs : robot, plane, start_configuration, group, compute
Outputs: out, configuration

In 2.x IK runs on the planner, not the robot, so the guard checks
robot.planner as well as the client. MobileRobot.inverse_kinematics wraps the
target construction and the WCF -> BCF conversion.

The Rhino plane is taken as world coordinates, so it is converted to the
robot's base frame internally -- which matters once the mobile base is
somewhere other than the world origin.
"""

from compas_rhino.conversions import plane_to_compas_frame
from scriptcontext import sticky as st

key = "inverse_solution"

if robot and robot.planner and plane and compute:  # noqa: F821
    if robot.client and robot.client.is_connected:  # noqa: F821
        frame_WCF = plane_to_compas_frame(plane)  # noqa: F821
        st[key] = robot.inverse_kinematics(  # noqa: F821
            frame_WCF,
            start_configuration=start_configuration,  # noqa: F821
            group=group,  # noqa: F821
        )

configuration = st.get(key, None)
