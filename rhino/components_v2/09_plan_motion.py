"""Calculate a motion path.

COMPAS FAB v2.0.1

Inputs : robot, target, start_configuration, group, use_live_start (bool),
         path_constraints, planner_id, compute (Button)
Outputs: out, trajectory, start_used

CONTRACT CHANGES FROM V1
------------------------
`goal_constraints` -> `target`: rename the input and wire it to the frame
target component. v1 built a list of PositionConstraint/OrientationConstraint;
2.x expresses the same thing as one Target and the planner converts it.

`attached_collision_meshes` is gone. Attachments are facts about the
RobotCellState now, set by `attach tool`, which uploads the cell to MoveIt.
Delete that input parameter.

WHY THE START STATE MATTERS HERE
--------------------------------
The v1 definition wired start_configuration from `analytic inverse` -- a
*computed* pose, not a measured one. Since the arm is executed open-loop over
URScript, nothing downstream ever checks that the robot was actually there. If
it was not, the whole trajectory is planned from a false premise and the first
move jumps.

So by default this takes the start state from the robot's live joint states and
plans from where the arm actually is. The wired start_configuration is used
only as a fallback, or when use_live_start is False (for planning offline).

The refusal to plan when use_live_start is set but no joint states have arrived
is deliberate. get_current_configuration() returns all zeros in that case,
which looks like a legitimate configuration -- planning from it would produce a
confident, wrong trajectory rather than an error.

NOTE ON OPEN-LOOP EXECUTION
---------------------------
MoveIt validates this trajectory against the planning scene at plan time, but
URScript execution has no runtime monitoring: nothing aborts if the world
changes or the arm deviates. movej blend radius also cuts corners between the
planned configurations, so the executed path is not exactly the planned one --
keep the radius small near obstacles.
"""

from scriptcontext import sticky as st

key = "trajectory"

path_constraints = list(path_constraints) if path_constraints else None  # noqa: F821
planner_id = str(planner_id) if planner_id else "RRTConnect"  # noqa: F821
use_live_start = True if use_live_start is None else use_live_start  # noqa: F821

start_used = None
blocked = None

if robot:  # noqa: F821
    live = robot.current_configuration(require_live=True)  # noqa: F821

    if use_live_start:
        if live is not None:
            start_used = live
            source = "live joint states"
        elif start_configuration:  # noqa: F821
            blocked = (
                "use_live_start is on but no joint states have arrived.\n"
                "    Refusing to fall back to the wired start_configuration,\n"
                "    because planning from the wrong pose is worse than not\n"
                "    planning. Press `subscribe` on `get joint states`, or set\n"
                "    use_live_start to False to plan offline deliberately."
            )
        else:
            blocked = "No joint states and no start_configuration wired."
    else:
        if start_configuration:  # noqa: F821
            start_used = start_configuration  # noqa: F821
            source = "wired start_configuration (offline)"
        else:
            blocked = "use_live_start is off but no start_configuration is wired."

if blocked:
    print(blocked)

if (
    robot  # noqa: F821
    and robot.planner  # noqa: F821
    and robot.client  # noqa: F821
    and robot.client.is_connected  # noqa: F821
    and start_used is not None
    and target  # noqa: F821
    and compute  # noqa: F821
):
    group_name = group or robot.main_group_name  # noqa: F821
    start_state = robot.cell_state_at(start_used, group_name)  # noqa: F821

    try:
        st[key] = robot.planner.plan_motion(  # noqa: F821
            target,  # noqa: F821
            start_state,
            group_name,
            options=dict(
                path_constraints=path_constraints,
                planner_id=planner_id,
            ),
        )
        traj = st[key]
        print("Planned %d points from %s." % (len(traj.points), source))
        print("  group  : %s" % group_name)
        print("  start  : %s" % [round(v, 3) for v in start_used.joint_values])
    except Exception as e:
        print("PLANNING FAILED: %s: %s" % (type(e).__name__, e))
        print("  Is move_group running? Is the target reachable and collision free?")

elif not blocked:
    if not robot:  # noqa: F821
        print("No robot wired.")
    elif not robot.planner:  # noqa: F821
        print("No planner attached -- press `load` on the mobile robot component.")
    elif not target:  # noqa: F821
        print("No target wired (use the frame target component).")
    elif not compute:  # noqa: F821
        print("Ready. Start: %s. Press compute." % source)

trajectory = st.get(key, None)
