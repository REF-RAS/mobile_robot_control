# Grasshopper components ported to COMPAS FAB v2.0.1

Ported from `Grasshopper/05_robot_moveit_planning_and_control-NEW.ghx`, in the
order they flow downstream of the `mobile robot` loader.

Paste each file into the matching script component. Input and output parameter
names are unchanged unless noted, so existing wiring survives.

## Needs no change

These call v1 `Robot` methods that now exist as delegates on `MobileRobot`,
so the original source runs untouched:

| Component | Why it still works |
|---|---|
| `zero` | `robot.zero_configuration(group)` delegates to `RobotCell` |
| `forward kinematics` | `get_configurable_joint_types/names/joints` delegate |
| `info` | `robot.info()` delegates to `RobotCell.print_info()` |
| `attach tool` | `get_group_names_from_link_name`, `attach_tool`, `detach_tool` |
| `analytic inverse` (x4) | `attached_tool`, `from_tcf_to_t0cf`, `get_configurable_joints` |

`attach_tool` now also re-uploads the cell to MoveIt, which v1 did separately.

## Ported

| File | Was | Change |
|---|---|---|
| `03_set_frame.py` | `set frame` | drop dead v1 imports |
| `04_tool.py` | `tool` | `Tool` -> `ToolModel` |
| `06_inverse.py` | `inverse` | guard on `robot.planner` |
| `08_frame_target.py` | `constraints from plane` | outputs a `FrameTarget`, not constraints |
| `09_plan_motion.py` | `plan motion` | goes through the planner; takes a target |
| `10_trajectory_visualize.py` | `trajectory visualize` | config merge + FK + `time_from_start` |
| `11_visualise.py` | `Visualise` | scene object replaces artist; see notes in file |

## Contract changes worth knowing

**`constraints from plane` -> `frame target`.** v1 emitted a list of
`PositionConstraint`/`OrientationConstraint`; 2.x emits one `FrameTarget` that
the planner converts internally. Rename the output from `goal_constraints` to
`target` and rewire `plan motion`. The three per-axis orientation tolerances
collapse into one `tolerance_orientation`.

**`plan motion` lost `attached_collision_meshes`.** Attachments are now facts
about the `RobotCellState`, set via `attach_tool` and uploaded with the cell.
Delete that input.

**`Visualise` collision outputs changed meaning.** v1 read the live MoveIt
planning scene back via `to_collision_meshes()`, which no longer exists. The
outputs now draw the rigid bodies and tools held in the RobotCell -- what was
*sent* to MoveIt rather than what MoveIt reports holding. In normal use these
agree; they diverge if something else modifies the planning scene.
