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
| `07_analytic_inverse.py` | `analytic inverse` x4 | lift double-count removed; `lift` input deleted, read from joint states |
| `08_frame_target.py` | `constraints from plane` | outputs a `FrameTarget`, not constraints |
| `09_plan_motion.py` | `plan motion` | goes through the planner; takes a target |
| `10_trajectory_visualize.py` | `trajectory visualize` | config merge + FK + `time_from_start` |
| `11_visualise.py` | `Visualise` | scene object replaces artist; see notes in file |
| `12_select.py` | `select` | guard against a null list |
| `98_diagnose_joint_states.py` | new | which joint-state topic and type deliver |
| `99_diagnose_tf.py` | new | whether TF reaches GH, by both routes |
| `00_reload_modules.py` | new | reload the libraries without restarting Rhino |
| `01_load_mobile_robot.py` | `mobile robot` | classifies load failures instead of losing them |

## This definition is strictly ONLINE

It assumes a live robot. Anything the robot can measure is read from the robot,
never taken as an input:

- lift position -> `robot_lift_lower_joint` on /robot/joint_states
- arm pose      -> the same topic
- arm base (RCF)-> URDF forward kinematics over those joint states

Where a measurement is missing, components **refuse to produce a result and say
why**, rather than substituting a plausible number. A value that looks
authoritative but was never measured is the more dangerous outcome: it is
executed as though it were true, and nothing downstream corrects it. URScript's
movej drives the six arm joints only, so an assumed lift height simply stays
wrong.

An offline equivalent can be developed separately if bench work needs it. The
feasible subset -- with a locally cached robot cell -- would be `Visualise`,
`forward kinematics`, `zero`, `set frame`, `tool`, `attach tool`, analytic IK
with an assumed lift, and the whole assembly cluster. `plan motion`, `inverse`
and anything reading joint states genuinely need the robot.

Do not add offline affordances to components in this file. Keeping the two
apart is what stops an assumed value reaching execution.

One inherited exception: `plan motion` has a `use_live_start` toggle that
allows a wired start configuration. It is deliberate and loud -- off by default,
and it refuses rather than silently falling back -- but it predates this policy
and would belong in the offline file.

## Access modes matter

Renaming a parameter does not change its access mode. `plan motion`'s
`goal_constraints` was **List** access in v1, since v1 passed a list of
constraints. Renamed to `target` it still arrived as
`System.Collections.Generic.List[Object]`, and compas_fab rejected the
container type with an error that says nothing about Grasshopper.

Set `target` to **Item** access. The script also unwraps a one-item list, so
it works either way. Checked the rest: `Visualise.bang` is the only other
list-access input among the ported components, and nothing reads it.


## A Configuration is iterable, and yields joint names

`compas_robots.Configuration` implements `__iter__`, `__len__` and
`__getitem__`, iterating over its **joint names**:

```python
list(configuration)
# ['robot_lift_lower_joint', 'robot_arm_shoulder_pan_joint', ...]
```

So any Grasshopper component that calls `list()` on one silently turns a
configuration into a list of strings. Index into that and you get a joint name
where a configuration was expected -- clamped to the end of a full 15-joint
state it surfaces as `robot_front_right_wheel_joint`, which reads like a
naming bug somewhere else entirely.

It also means an input with **Item** access on `trajectory visualize`'s
`configurations` output receives one Configuration at a time and may expand
each. Use **List** access on anything consuming that output.

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
