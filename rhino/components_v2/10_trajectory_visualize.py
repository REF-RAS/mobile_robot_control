"""Visualise a trajectory.

COMPAS FAB v2.0.1

Inputs : robot, group, trajectory
Outputs: out, start_configuration, configurations, fraction, time, planes,
         P, V, A

Changes from v1:
- merge_group_with_full_configuration()  -> robot.full_configuration()
- forward_kinematics(c, group, options=dict(solver="model"))
      -> robot.forward_kinematics(c, group), which is model-based and needs no
         backend round trip -- important here, since this runs per point.
- trajectory.time_from_start no longer exists on JointTrajectory in 2.x. It
  lives on each JointTrajectoryPoint, so the trajectory duration is the last
  point's value.

FK returns a frame in the robot's base frame; from_BCF_to_WCF puts it back in
world coordinates so the planes land where the mobile base actually is.
"""

from compas_ghpython.drawing import draw_frame
from compas_ghpython.sets import list_to_ghtree

start_configuration = None
configurations = []
fraction = 0.0
time = 0.0

planes = []
positions = []
velocities = []
accelerations = []

if robot and trajectory:  # noqa: F821
    group = group or robot.main_group_name  # noqa: F821

    for c in trajectory.points:  # noqa: F821
        configurations.append(
            robot.full_configuration(c, trajectory.start_configuration)  # noqa: F821
        )
        frame_BCF = robot.forward_kinematics(c, group)  # noqa: F821
        planes.append(draw_frame(robot.from_BCF_to_WCF(frame_BCF)))  # noqa: F821
        positions.append(c.positions)
        velocities.append(c.velocities)
        accelerations.append(c.accelerations)

    start_configuration = trajectory.start_configuration  # noqa: F821
    fraction = trajectory.fraction  # noqa: F821
    if trajectory.points:  # noqa: F821
        time = trajectory.points[-1].time_from_start.seconds  # noqa: F821

P = list_to_ghtree(list(zip(*positions)))
V = list_to_ghtree(list(zip(*velocities)))
A = list_to_ghtree(list(zip(*accelerations)))

# Report, so `out` is not blank when something downstream misbehaves.
if not robot:  # noqa: F821
    print("No robot wired.")
elif not trajectory:  # noqa: F821
    print("No trajectory wired -- has plan motion produced one?")
else:
    print("%d configurations, %d planes" % (len(configurations), len(planes)))
    print("group    : %s" % group)
    print("duration : %.2fs   fraction: %s" % (time, fraction))
    if configurations:
        first = configurations[0]
        print("each configuration has %d joints:" % len(first.joint_names))
        print("   %s" % ", ".join(first.joint_names))
        # Configuration is iterable over joint NAMES, so any downstream
        # component that calls list() on one turns it into strings. Anything
        # consuming `configurations` must take the list, not each item.
        print("")
        print("NOTE: a Configuration iterates as joint names, so a downstream")
        print("      component with Item access on this output will expand one")
        print("      into strings rather than treating it as a single object.")
