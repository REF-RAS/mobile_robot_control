"""Calculate a motion path.

COMPAS FAB v2.0.1

Inputs : robot, target, start_configuration, group, path_constraints,
         planner_id, compute
Outputs: out, trajectory

CONTRACT CHANGES
----------------
`goal_constraints` -> `target`: rename the input and wire it to the frame
target component. The planner builds the constraints itself.

`attached_collision_meshes` is gone. Attachments are facts about the
RobotCellState now -- set them with the `attach tool` component, which uploads
the cell to MoveIt. Delete that input parameter.

Planning runs on robot.planner. The start state is built from the cell state
with `start_configuration` merged in, so any attached tool travels with it.
"""

from scriptcontext import sticky as st

key = "trajectory"

path_constraints = list(path_constraints) if path_constraints else None  # noqa: F821
planner_id = str(planner_id) if planner_id else "RRTConnect"  # noqa: F821

if (
    robot  # noqa: F821
    and robot.planner  # noqa: F821
    and robot.client  # noqa: F821
    and robot.client.is_connected  # noqa: F821
    and start_configuration  # noqa: F821
    and target  # noqa: F821
    and compute  # noqa: F821
):
    group_name = group or robot.main_group_name  # noqa: F821
    start_state = robot.cell_state_at(start_configuration, group_name)  # noqa: F821

    st[key] = robot.planner.plan_motion(  # noqa: F821
        target,  # noqa: F821
        start_state,
        group_name,
        options=dict(
            path_constraints=path_constraints,
            planner_id=planner_id,
        ),
    )

trajectory = st.get(key, None)
