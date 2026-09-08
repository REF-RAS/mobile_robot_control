"""Create a FrameTarget from a plane, for the group's end-effector link.

COMPAS FAB v2.0.1

Was: 'constraints from plane'
Inputs : robot, plane, group, tolerance_position, tolerance_orientation
Outputs: out, target

CONTRACT CHANGE
---------------
v1 emitted a list of PositionConstraint/OrientationConstraint built by
robot.constraints_from_frame(). 2.x has no such method: a goal is expressed as
a single Target object, and the planner converts it to constraints internally.

So this component now outputs `target` rather than `goal_constraints`. Rename
the output parameter and rewire the `plan motion` component to match.

v1 took one orientation tolerance per axis (x/y/z); FrameTarget takes a single
tolerance_orientation that the planner applies to all three. Drop the three old
inputs for one.

The frame is converted from world coordinates to the robot's base frame here,
because the MoveIt backend constrains against the robot's root link. Do not
convert again downstream.
"""

import math

from compas_fab.robots import FrameTarget, TargetMode
from compas_rhino.conversions import plane_to_compas_frame

target = None

if robot and plane:  # noqa: F821
    tolerance_position = tolerance_position or 0.001  # noqa: F821
    tolerance_orientation = math.radians(tolerance_orientation or 1.0)  # noqa: F821

    frame_WCF = plane_to_compas_frame(plane)  # noqa: F821
    frame_BCF = robot.from_WCF_to_BCF(frame_WCF)  # noqa: F821

    # TargetMode.ROBOT targets the planning group's tip link. Use
    # TargetMode.TOOL to target the TCF of an attached tool instead.
    target = FrameTarget(
        frame_BCF,
        TargetMode.ROBOT,
        tolerance_position=tolerance_position,
        tolerance_orientation=tolerance_orientation,
    )
