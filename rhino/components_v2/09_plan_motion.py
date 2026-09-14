"""Calculate a motion path.

COMPAS FAB v2.0.1

Inputs : robot, target, start_configuration, group, use_live_start (bool),
        path_constraints, planner_id, compute (Button)
Outputs: out, trajectory, start_used

CONTRACT CHANGES FROM V1
------------------------
`goal_constraints` -> `target`: wire it to the ported `constraints from plane`
component, which now emits one FrameTarget instead of a constraint list.

`attached_collision_meshes` is gone -- attachments are facts about the
RobotCellState, set by `attach tool`. Delete that input parameter.

WHY THE START STATE MATTERS
---------------------------
v1 wired start_configuration from `analytic inverse` -- a computed pose. The
arm is executed open-loop over URScript, so nothing downstream ever verifies
the robot was there. A wrong start yields a confident, wrong trajectory whose
first move jumps.

So the start state comes from live joint states by default. The wired
start_configuration is the explicit offline fallback via use_live_start.

Refusing to plan when use_live_start is on but no joint states have arrived is
deliberate: get_current_configuration() returns all zeros in that case, which
is indistinguishable from a real pose.

WHY THE REPORT IS LATCHED
-------------------------
`compute` is a Button -- True for exactly one solve. Grasshopper then re-solves
with it False and whatever the component prints on that pass replaces the
result. The plan outcome flashed up and was gone before it could be read.

The last outcome is kept in sticky and reprinted every solve, with its
timestamp, above a separate line describing the current state. Both stay
visible, and a stale result is obvious from its clock.

NOTE ON OPEN-LOOP EXECUTION
---------------------------
MoveIt validates this trajectory against the planning scene at plan time, but
URScript execution has no runtime monitoring. movej blend radius also cuts
corners between the planned configurations, so the executed path is not
exactly the planned one -- keep the radius small near obstacles.
"""

import time

from scriptcontext import sticky as st

key = "trajectory"
report_key = "plan_motion_report"

path_constraints = list(path_constraints) if path_constraints else None  # noqa: F821
planner_id = str(planner_id) if planner_id else "RRTConnect"  # noqa: F821
use_live_start = True if use_live_start is None else use_live_start  # noqa: F821

start_used = None
source = None
blocked = None


def as_single(value):
    """Unwrap a one-item list into the item itself.

    v1 declared `goal_constraints` with List access, because it passed a list
    of constraints. Renaming the parameter to `target` does not change its
    access mode, so a single FrameTarget arrives wrapped as
    System.Collections.Generic.List[Object] and compas_fab rejects it with
    "Target type ... not supported by ROS planning backend" -- an error about
    the type, which reads nothing like a Grasshopper access-mode setting.

    Setting the input to Item access is the real fix. This keeps the component
    working either way. Strings are excluded because they are iterable but are
    never the container meant here, and anything carrying `target_mode` is
    already a Target and passes through untouched.
    """
    if value is None or isinstance(value, str) or hasattr(value, "target_mode"):
        return value
    if not hasattr(value, "__len__"):
        return value
    try:
        unwrapped = list(value)
    except TypeError:
        return value
    if len(unwrapped) == 1:
        return unwrapped[0]
    if not unwrapped:
        return None
    return value


target = as_single(target)  # noqa: F821

# --------------------------------------------------------------------------
# Decide the start state
# --------------------------------------------------------------------------
if not robot:  # noqa: F821
    blocked = "No robot wired."
else:
    live = robot.current_configuration(require_live=True)  # noqa: F821

    if use_live_start:
        if live is not None:
            start_used, source = live, "live joint states"
        elif start_configuration:  # noqa: F821
            blocked = (
                "use_live_start is on but no joint states have arrived.\n"
                "    Not falling back to the wired start_configuration -- planning\n"
                "    from the wrong pose is worse than not planning. Press\n"
                "    `subscribe` on `get joint states`, or set use_live_start False."
            )
        else:
            blocked = (
                "No joint states and no start_configuration wired.\n"
                "    Press `subscribe` on `get joint states`."
            )
    elif start_configuration:  # noqa: F821
        start_used, source = start_configuration, "wired start_configuration (offline)"  # noqa: F821
    else:
        blocked = "use_live_start is off but no start_configuration is wired."

# --------------------------------------------------------------------------
# Plan, and latch whatever happened
# --------------------------------------------------------------------------
if compute:  # noqa: F821
    lines = []
    if blocked:
        lines.append("DID NOT PLAN")
        lines.append(blocked)
    elif not robot.planner:  # noqa: F821
        lines.append("DID NOT PLAN")
        lines.append("No planner attached -- press `load` on the mobile robot component.")
    elif not (robot.client and robot.client.is_connected):  # noqa: F821
        lines.append("DID NOT PLAN")
        lines.append("ROS client is not connected.")
    elif not target:  # noqa: F821
        lines.append("DID NOT PLAN")
        lines.append("No target wired (from the ported `constraints from plane`).")
    else:
        group_name = group or robot.main_group_name  # noqa: F821
        start_state = robot.cell_state_at(start_used, group_name)  # noqa: F821
        try:
            st[key] = robot.planner.plan_motion(  # noqa: F821
                target,  # noqa: F821
                start_state,
                group_name,
                options=dict(path_constraints=path_constraints, planner_id=planner_id),
            )
            traj = st[key]
            lines.append("PLANNED %d points" % len(traj.points))
            lines.append("  group  : %s" % group_name)
            lines.append("  start  : %s  (%s)"
                        % ([round(v, 3) for v in start_used.joint_values], source))
            if traj.points:
                lines.append("  end    : %s"
                            % [round(v, 3) for v in traj.points[-1].joint_values])
                lines.append("  time   : %.2fs" % traj.points[-1].time_from_start.seconds)
        except Exception as e:
            lines.append("PLANNING FAILED: %s: %s" % (type(e).__name__, e))
            lines.append("  Is move_group running? Is the target reachable and")
            lines.append("  collision free from this start pose?")

    st[report_key] = {"when": time.strftime("%H:%M:%S"), "lines": lines}

# --------------------------------------------------------------------------
# Report: the latched outcome, then the current state
# --------------------------------------------------------------------------
report = st.get(report_key)
if report:
    print("=== last compute at %s ===" % report["when"])
    for line in report["lines"]:
        print(line)
    print("")

print("=== now ===")
if blocked:
    print(blocked)
elif not robot.planner:  # noqa: F821
    print("Start ready (%s), but no planner attached." % source)
elif not target:  # noqa: F821
    print("Start ready (%s), but no target wired." % source)
else:
    print("Ready to plan from %s. Press compute." % source)

trajectory = st.get(key, None)
