# System workflow review — power on to task complete

Vogui AB · UR20 · Ewellix liftkit · COMPAS FAB 2.0.1 · ROS 2 Jazzy

Written after `plan motion` failed with `FAILURE, 99999` on a target 3.25 m from
the arm base. That was not a bug. It was the first time the workflow was asked a
question it has no mechanism to answer: **where is the robot?**

This document traces the whole chain and marks, at each link, whether a value is
**measured**, **commanded**, or **assumed**.

---

## 1. The finding, in one paragraph

Three of the four things that determine whether a plan is valid are measured
from the robot: arm joint angles, lift height, and the arm base frame derived
from them. The fourth — **the pose of the mobile base in the world** — is a
plane you drag in Rhino. Nothing reads it back from the robot, and nothing
checks it against the robot. Every target frame passes through
`from_WCF_to_BCF()`, so that one assumed value offsets *every* plan, IK solve
and reachability judgement in the definition. When the assumption is wrong by
metres, MoveIt is handed an unreachable goal and correctly declines.

---

## 2. The frame chain

```
  WCF        world / Rhino origin
   |
   |  BCF                                  <-- ASSUMED (dragged plane)
   v
  robot root link
   |
   |  RCF = FK(root -> robot_arm_base)     <-- MEASURED (URDF + live lift)
   v
  arm base
   |
   |  6 arm joints                         <-- MEASURED (/robot/joint_states)
   v
  tool0
   |
   |  tool attachment frame                <-- MODELLED (tool definition)
   v
  TCF
```

Every link is sound except the first.

### Where each value comes from

| Value | Source | Status |
|---|---|---|
| Arm joint angles | `/robot/joint_states` | **measured** |
| Lift height | `robot_lift_lower_joint`, same topic | **measured** |
| Arm base (RCF) | URDF FK over the above, `compute_RCF_from_model()` | **measured** (derived) |
| Tool geometry | `ToolModel` in the cell | modelled |
| Robot cell / URDF | `/robot/robot_description` + mesh server | **measured** |
| **Base pose (BCF)** | **a Rhino plane in `set frame`** | **assumed** |
| Target frames | assembly design, in WCF | designed |
| Arm trajectory | MoveIt, planned against all of the above | derived |
| Executed arm motion | URScript `movej` over TCP 30002 | **commanded, unverified at runtime** |
| Executed base motion | `cmd_vel` published for a timed duration | **commanded, never verified** |

---

## 3. Where the chain breaks

### 3.1 The base pose is disconnected at both ends

`MobileRobot.BCF` is written in exactly one place — `03_set_frame.py`, from the
`target_BCF` plane:

```python
BCF = robot.base_frame_for_link_at(
    plane_to_compas_frame(target_BCF), "robot_base_footprint")
```

Nothing else writes it. In particular, **nothing on the robot writes it.**

The client does keep a dead-reckoned estimate. `move_forward`, `move_radial`
and `rotate_in_place` each publish `cmd_vel` for a computed duration and then
apply the *intended* transform to `self.robot_frame`
(`mobile_robot_client.py:412, 432, 553`). But:

- it integrates the command, not the motion — no odometry, no slip, no
  correction;
- `condition_odometry()` raises `NotImplementedError`, `echo_robot_odom()` is
  `pass`;
- and `self.robot_frame` is read only by `move_to_frame()`. It **never reaches
  `MobileRobot.BCF`**.

So even the dead reckoning is orphaned. Drive the base by any means — cmd_vel,
joystick, pushing it — and Grasshopper's belief about where it stands does not
change.

### 3.2 Unreachability is only discovered by failing

There is no reachability test anywhere in the definition. A target 3 m away is
built, converted, wrapped in a `FrameTarget` and sent to MoveIt exactly like a
valid one. The first indication of trouble is a planner timeout and a catch-all
error code, which is the most expensive possible way to learn that a number is
too big.

The arithmetic is trivial and local: the UR20 reaches 1.75 m from its base, and
`robot.RCF` is already known.

### 3.3 There is no notion of a base station

The whole premise of a mobile manipulator is that the work is larger than the
workspace, so you cover it in stations. Nothing in the definition expresses
that. There is no grouping of targets by base pose, no ordering of stations, no
record of which targets have been done. Each plan is an isolated act with no
memory of the one before.

### 3.4 Execution is not verified

The arm is driven open-loop by URScript, which was a deliberate decision and is
documented. The gap is the *other* half of that decision: after a motion,
nothing compares where the arm ended up against where the plan said it would.
The data to do it is already arriving on `/robot/joint_states` — the comparison
is simply not made.

Base motion is worse: there is no feedback path at all.

---

## 4. The workflow as it actually needs to run

Stages marked **[GAP]** have no implementation today.

### Stage 0 — Physical bringup

Robot powered, e-stops released, drives enabled, arm powered and in remote
control, liftkit homed.

*The exact sequence for this robot is not recorded anywhere in the repo. It
should be, and it should be written down at the machine rather than inferred.*

### Stage 1 — ROS 2 stack

Four services, per §1 of `OPERATING_GUIDE.md`: rosbridge (9090), mesh HTTP
server (9190), `robot_state_publisher`, `move_group`. Plus, once §5.1 below is
implemented, whatever publishes the base pose.

### Stage 2 — Connect and load

`ROS.connect` → `mobile robot.load` (~16 s) → `get joint states.subscribe` →
Trigger running. At this point arm, lift and RCF are live.

### Stage 3 — Establish where the robot is **[GAP]**

Read the base pose from the robot and set the BCF from it.
Today: drag a plane and hope.

### Stage 4 — Tool

`tool` → `attach tool`. Confirm `uploaded to MoveIt: True`.

### Stage 5 — Task geometry

The assembly cluster produces target frames in WCF. Independent of the robot,
correctly so.

### Stage 6 — Partition targets into base stations **[GAP]**

For each target, is it within 1.75 m of the arm base for some candidate base
pose? Group them, order the groups, pick the next one.
Today: implicit, in the operator's head.

### Stage 7 — Drive the base to the station

Whatever the means, this ends with the robot somewhere new.

### Stage 8 — Re-establish the base pose **[GAP]**

Stage 3 again. Without it, everything after Stage 7 is planned against a stale
position — which is precisely today's failure.

### Stage 9 — Check reachability **[GAP]**

Before planning, not after failing.

### Stage 10 — Plan

`frame target` → `plan motion`. Working.

### Stage 11 — Execute

URScript `movej` over TCP 30002. Working, untested on the UR20.

### Stage 12 — Verify and advance **[GAP]**

Compare the measured arm configuration against the planned endpoint. On
mismatch, stop. On success, advance to the next target, or back to Stage 6 for
the next station.

---

## 5. Recommendations, most valuable first

### 5.1 Close the base-pose loop

The single change that fixes the whole class of failure seen today.

The RB-Vogui publishes odometry, and its navigation stack publishes a map-frame
pose. Either is a plain ROS topic, so it arrives over rosbridge without needing
`tf2_web_republisher`. Confirm what exists:

```bash
ros2 topic list | grep -Ei "odom|amcl|pose"
ros2 topic info /robot/robotnik_base_control/odom
```

Then add to `MobileRobotClient` a subscription storing the latest base pose, and
to `MobileRobot` a `measured_BCF` property built from it. Give `set frame` a
third mode — `set_to_measured` — alongside the existing two.

The two frames are not interchangeable. `odom` drifts without bound but is
always available; `map` is drift-free but exists only when localisation is
running, and can jump when it relocalises. For planning over minutes at a single
station, `odom` is sufficient and simpler. Reconciling the Rhino world origin
with the ROS map origin is a separate, one-off alignment.

### 5.2 Gate on reachability before planning

Cheap and local. In `08_frame_target.py`, with `robot.RCF` already known:

```python
reach = (robot.from_BCF_to_WCF(robot.RCF).point - frame_WCF.point).length
if reach > 1.75:
    print("OUT OF REACH: %.2f m from the arm base (UR20 limit 1.75 m)." % reach)
    print("   Move the base closer, or pick a different station.")
```

Worth exposing the distance as an output so the canvas can colour targets by
reachability. The whole task then becomes readable at a glance, and Stage 6
stops being guesswork.

### 5.3 Make base stations explicit

A component taking all task targets plus a candidate base pose, returning
reachable / unreachable. Iterate candidates until every target is covered. This
turns §3.3 from an unwritten procedure into canvas state, and gives a natural
place to track progress.

### 5.4 Verify execution

After `movej` completes, read `/robot/joint_states` and compare against the
trajectory's final point. Refuse to proceed past a mismatch beyond tolerance.
This is the cheap 80% of what closed-loop execution buys, without needing the
MoveIt execution path.

### 5.5 Decide what the cmd_vel helpers are for

As written they are jog commands wrapped in a `move_to_frame` that implies a
positioning accuracy they do not have. Either close them against odometry, or
rename and document them as jogging only.

---

## 6. What is already sound

Worth stating, because the list above is all deficits:

- The lift is measured, with no override — `07_analytic_inverse.py` refuses to
  produce a configuration without a real reading. This is the pattern the base
  pose should follow.
- RCF is derived from the URDF rather than TF, removing a dependency on a ROS 1
  package with no Jazzy build.
- Unknown joint names are rejected client-side before they can abort
  `move_group`.
- The strictly-online policy means no component quietly substitutes a plausible
  number for a missing measurement.
- Frame algebra, tool attachment, planning and visualisation all work.

The architecture is right. It is missing one measurement, and the checks that
measurement makes possible.
