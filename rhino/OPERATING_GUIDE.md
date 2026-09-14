# Operating guide — Vogui AB from Grasshopper

COMPAS FAB v2.0.1 · ROS 2 Jazzy · Rhino 8 (Python 3.9)

Every button press needed to drive the system, and what has to be running on the
robot for each to work.

Robot AB is at **192.168.0.200**, reached over its onboard wifi. All ROS topics
are namespaced under **`robot`**.

---

## 1. On-robot dependencies

Four services, each required by a different part of the workflow. Nothing here
starts automatically — check all four before a session.

| Service | Port | Required for | Check |
|---|---|---|---|
| **rosbridge** | 9090 | everything | `ros2 node list \| grep rosbridge` |
| **HTTP mesh server** | 9190 | loading robot geometry | `curl -I http://192.168.0.200:9190/` |
| **robot_state_publisher** | — | URDF/SRDF, joint states | `ros2 topic echo /robot/robot_description --once` |
| **move_group** (MoveIt) | — | IK and motion planning | `ros2 node list \| grep move_group` |

### Namespacing: topics yes, services no

This robot namespaces its **topics** under `robot` (`/robot/robot_description`,
`/robot/joint_states`) but move_group's **services** are not namespaced --
`/plan_kinematic_path`, not `/robot/plan_kinematic_path`. Verified with
`ros2 service list | grep plan_kinematic_path`.

So the loader's `prefix` input (topics) and its `MOVEIT_NAMESPACE` constant
(services) are deliberately different. compas_fab hardcodes unnamespaced
service names, so `MOVEIT_NAMESPACE = ""` is correct here and no rewriting
happens. If a future bringup launches move_group inside a namespace, set it --
otherwise every planning call waits on a service nobody provides and times out.

Note that a missing move_group looks identical to a wrong service name: both
give `Timeout exceeded while waiting for service response`. Check
`ros2 node list | grep move_group` first.


### The mesh server

There is no ROS 2 equivalent of the ROS 1 `file_server` package, so this is a
plain static HTTP server you run yourself. Its document root must have **package
names as direct children**, because a URDF reference
`package://ur_description/meshes/ur20/visual/base.dae` is fetched from
`<root>/ur_description/meshes/ur20/visual/base.dae`.

Packages are scattered across a colcon install tree, so symlink them into one
directory:

```bash
mkdir -p ~/mesh_root
for p in robotnik_description ewellix_description ur_description robotnik_sensors; do
  ln -sfn "$(ros2 pkg prefix --share $p)" ~/mesh_root/$p
done
cd ~/mesh_root && python3 -m http.server 9190
```

`ros2 pkg prefix --share` resolves each package wherever it lives, so this works
for both `/opt/ros` and workspace packages. Run it under tmux or systemd —
it dies with your ssh session.

Verify from the dev PC before touching Grasshopper:

```
curl -I http://192.168.0.200:9190/ur_description/meshes/ur20/visual/base.dae
```

`200` is good. Connection refused means it isn't running; `404` means the
document root is wrong.

### NOT required: tf2_web_republisher

`roslibpy`'s TFClient needs the `/republish_tfs` service from
`tf2_web_republisher`, a ROS 1 package with no supported Jazzy build. It is not
running on this robot and **does not need to be**: `MobileRobot.RCF` falls back
to computing the arm base from the URDF, which gives the same answer.

It becomes relevant again only for **marker tracking**, where transforms to
ArUco frames cannot come from the URDF. The intended ROS 2 route there is the
`/aruco_markers` topic instead — see `marker_tracking/marker_utilities.py` in
the workshop repo.

---

## 2. Dev PC setup

Done once; verify if imports fail.

**Package paths** — `C:\Users\esplinj\.rhinocode\python-3.pth` must list the
`src` folder of every workspace package:

```
...\workspace\mobile_robot_control\src
...\workspace\fabrication_manager\src
...\workspace\ur_fabrication_control\src
...\workspace\assembly_information_model\src
...\workspace\workshop_qut_mobile_robots\src
```

Edits to these packages are live on next import — no pip reinstall ever needed.

**Network** — the dev PC must be on the robot's onboard wifi. `ping
192.168.0.200` before anything else.

---

## 3. Startup sequence

Order matters. Each step depends on the one before.

| # | Component | Input | Action | Expect |
|---|---|---|---|---|
| 1 | `ROS` | `connect` | toggle **True** | `is_connected` True, `ros_distro` `jazzy` |
| 2 | `mobile robot` | `load` | press **once** | ~16s, then `LOADED 'rbvogui_xl_plus'` |
| 3 | `set frame` | `set_frame_to` | `set_to_BCF` | `BCF set to Frame(...)` |
| 4 | `tool` | `tool` | `0` or `1` | `Loaded '...' as tool id '...'` |
| 5 | `attach tool` | `attach` | press | `Attached '...' to group '...'` |
| 6 | `get joint states` | `subscribe` | press | loader shows `N joints held` |
| 7 | `Trigger` | play | start | live pose updates every 2s |

**Step 2 takes ~16 seconds** — it downloads 30 meshes. The cell is then cached
in sticky; don't press `load` again unless the geometry changed.

**Step 5 uploads the cell to MoveIt.** Check the output says
`uploaded to MoveIt: True`. If it says `False` the planner isn't attached and
only your local model changed — planning will ignore the tool.

**Step 7's Trigger is not optional for live values.** Grasshopper only re-solves
when the canvas changes; the ROS callback fills a dict on a background thread
that GH knows nothing about. Without the Trigger, `draw` reports whatever it
read at the last solve and then sits frozen.

---

## 4. Task sequences

### Move the robot base (simulation)

1. `set_frame_to` → `set_to_BCF`
2. Drag the plane feeding `target_BCF`
3. Robot redraws at the new position

The base is anchored on **`robot_base_footprint`**, so the plane drives the
ground footprint. `robot_base_frame` natively positions the URDF *root*, which
on this robot sits at arm-mount height — anchoring is what keeps the wheels on
the floor.

`set_to_RCF` does the inverse: you place the *arm base* and it solves for where
the mobile base must be.

### Show the real robot pose

1. Complete startup steps 1–2
2. Press `subscribe` on `get joint states`
3. Start the Trigger
4. Set the configuration Stream Filter to **`5 real_configuration`**

Joint states arrive on `/robot/joint_states` as `sensor_msgs/msg/JointState`.
The per-controller topics (`/robot/arm/arm_joint_states_unused`) are remapped
aside and carry nothing.

### Plan a motion

1. Startup steps 1–2, and 4–5 if using a tool
2. Set the target plane on `frame target`
3. Press `compute` on `plan motion`
4. Wire `trajectory visualize` to see the path

Requires **move_group** running. `plan motion` takes a `target` (a `FrameTarget`),
not the v1 `goal_constraints` list.

### Attach or change tools

1. `tool` → pick index
2. `attach tool` → press `attach`
3. To swap: pick a new index, press `attach` again — the previous tool is
   detached automatically
4. `remove` detaches without attaching another

A tool can only be attached to **one planning group** in compas_fab 2.x.

---

## 5. After editing Python

Library edits do not take effect until the module is re-imported. Python caches
it in `sys.modules`, and the `MobileRobot` in sticky is an instance of the class
that was loaded at the time.

1. Press **`req_reload`**
2. Press **`load`** on the mobile robot component (~16s — mesh server must be up)
3. Press **`subscribe`** again if you were reading joint states
4. Recompute

A Rhino restart does the same thing and clears sticky too.

---

## 6. Troubleshooting

Most failures in this stack are **silent**: subscribing or publishing to a
topic that doesn't exist raises nothing.

| Symptom | Cause | Fix |
|---|---|---|
| `mobile_robot` output null | sticky cleared by restart | press `load` |
| Load fails, `out` names the mesh server | server not running | start it (§1) |
| Load fails with 404 | wrong document root | check the symlinks |
| `Timeout waiting for message on topic` | wrong `prefix` | must be `robot` |
| `AttributeError: 'NoneType' has no attribute ...` | upstream `robot` is null | check the loader; wire display is Hidden on many `robot` inputs — hover to see |
| `AttributeError: 'MobileRobot' object has no attribute ...` | stale module | §5 |
| Arm stuck at zero pose | joint states not arriving | check loader's `joint states:` line, then run `98_diagnose_joint_states` |
| Robot draws at double offset | `Orient` still in the preview chain | wire `visual_meshes` straight to Custom Preview |
| Robot drawn with wheels below z=0 | base anchored on the URDF root | use `base_frame_for_link_at` (already in `03_set_frame`) |
| Tool attached but not drawn | scene object cached before the attach | fixed via `structural_signature`; if it recurs, press `req_reload` |
| `robot.RCF` is None | no TF and link missing from model | check `ARM_BASE_LINK` matches the URDF |
| Component flickers red, `out` empty | Button-driven error erased by the next solve | the loader caches status; for others, read `out` immediately |

### Diagnostics

| Component | Answers |
|---|---|
| `98_diagnose_joint_states` | which joint-state topic and message type actually deliver, and whether the 7 expected joint names are present |
| `99_diagnose_tf` | whether TF reaches GH, via raw `/tf` and via TFClient separately |
| `gh_diagnose_fileserver` | the exact mesh URLs and their HTTP status |

All three use a Button and cache their report in sticky, so the result survives
the button releasing.

---

## 7. Display and memory

### Too many plane previews

`trajectory visualize.planes` emits one plane per trajectory point, and
`Visualise.joint_planes` one per link (16 on this robot). Rhino draws a plane as
a rectangular grid, so previewing either fills the viewport with red rectangles
and pushes memory up fast.

Planes are the wrong primitive at this count. Three options, best first:

1. **Turn preview off** on the plane outputs — right-click the component or the
   param, untick Preview. The data still flows; it just stops drawing.
2. **Use the axis drawer already on the canvas.** The unlabelled `Py3` component
   in the Visualisation group takes planes and emits points, vectors and colours
   for `Vector Display Ex` — short coloured axes instead of grids, far lighter
   and far easier to read. Its `scale` slider sets the length.
3. **Show the path, not the frames.** The `VISUALISE MOTION PATH` group takes the
   plane origins into a `PolyLine`, giving the tool path as one curve. Usually
   what you want from a trajectory; per-point orientation only matters when
   checking wrist flips.

### Memory

Two leaks were found and fixed, both in components rather than the library:

- The loader built a fresh `MobileRobotClient` every solve, each with live topic
  subscriptions never torn down.
- `Visualise` cached a `RobotCellObject` per cell structural signature and never
  evicted the old one, so every tool attach or detach stranded a complete set of
  robot meshes in sticky for the session.

If memory still climbs, suspect plane previews first. Failing that, restart
Rhino — sticky holds the cell, the scene objects and the planner for the life of
the session by design.

---

## 8. Known gaps

- **Direct motion commands are untested on ROS 2.** `move_forward`,
  `rotate_in_place`, `arm_move_joint`, `set_lift_height` have had their topics
  corrected but none has been run against the robot. Be near the e-stop.
- **`cmd_vel` variant unconfirmed.** `CMD_VEL_TOPIC` is set to
  `/robot/robotnik_base_control/cmd_vel_unstamped`, matching the bare Twist this
  client builds. Verify with
  `ros2 topic info /robot/robotnik_base_control/cmd_vel` before driving.
- **Marker tracking not ported.** The three marker tasks still use TF, which
  needs the absent republisher.
- **`Orient` components elsewhere** expect to share a base plane with
  `set frame`; they will disagree about the base position until rewired.
