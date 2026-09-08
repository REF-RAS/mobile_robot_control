"""Visualise the robot.

COMPAS FAB v2.0.1

Inputs : robot, configuration, show_frames, show_visual_meshes,
         show_robot_collision_meshes, show_collision_meshes,
         show_attached_collision_meshes, show_BCF, show_RCF, show_EEF, bang
Outputs: out, joint_planes, visual_meshes, robot_collision_meshes,
         collision_meshes, attached_meshes, BCF, RCF, EEF

Changes from v1
---------------
`robot.update(configuration)` and `robot.draw_visual()/draw_collision()` were
Robot methods driving an artist. In 2.x drawing goes through a scene object
over the whole cell, updated from a RobotCellState.

Visual vs collision is fixed when a RobotCellObject is constructed, not per
draw call, so two cached scene objects are kept -- one of each.

The scene object positions the robot from `robot_base_frame`, so it is handed
`display_cell_state()`, which sets that to the BCF. v1 did the same job by
hand with ghcomp.Orient after drawing; that is no longer needed.

CONTRACT CHANGE
---------------
`collision_meshes` and `attached_meshes` previously read the live MoveIt
planning scene back through `to_collision_meshes()`, which does not exist in
2.x. They now draw the rigid bodies and tools held in the RobotCell -- what was
*sent* to MoveIt rather than what MoveIt reports holding. In normal use those
agree; they diverge if something else edits the planning scene.
"""

from compas.scene import SceneObject
from compas_ghpython import create_id
from compas_rhino.conversions import frame_to_rhino
from compas_fab.ghpython.scene import RobotCellObject
from scriptcontext import sticky as st

joint_planes = None
visual_meshes = None
robot_collision_meshes = None
collision_meshes = None
attached_meshes = None
BCF = None
RCF = None
EEF = None


def cell_object(robot, visual, collision):
    """Cached RobotCellObject. Meshes are built once, then only transformed.

    The cell's structural signature is part of the key. A RobotCellObject
    builds its child scene objects from the cell it was constructed with, so
    one cached before a tool was attached has no scene object for that tool and
    would never draw it, no matter how many times update() is called. The
    signature is a hash of the robot name plus the tool and rigid-body ids, so
    attaching, detaching or renaming anything yields a new key and a rebuild.
    """
    signature = robot.robot_cell.structural_signature()
    key = create_id(  # noqa: F821
        ghenv.Component,  # noqa: F821
        "cell_%s_%s_%s" % (visual, collision, signature),
    )
    scene_object = st.get(key)
    if scene_object is None:
        scene_object = SceneObject(
            item=robot.robot_cell,
            sceneobject_type=RobotCellObject,
            draw_visual=visual,
            draw_collision=collision,
        )
        st[key] = scene_object
    return scene_object


if robot:  # noqa: F821
    # robot_base_frame = BCF, so everything draws where the base actually is.
    state = robot.display_cell_state(configuration)  # noqa: F821

    if show_frames:  # noqa: F821
        joint_frames = robot.model.transformed_frames(state.robot_configuration)  # noqa: F821
        joint_planes = [frame_to_rhino(robot.from_BCF_to_WCF(f)) for f in joint_frames]  # noqa: F821

    if show_BCF:  # noqa: F821
        BCF = frame_to_rhino(robot.BCF)  # noqa: F821

    if show_RCF and robot.RCF:  # noqa: F821
        RCF = frame_to_rhino(robot.from_BCF_to_WCF(robot.RCF))  # noqa: F821

    if show_EEF:  # noqa: F821
        EEF = frame_to_rhino(robot.forward_kinematics(configuration, in_wcf=True))  # noqa: F821

    if show_visual_meshes:  # noqa: F821
        visual_meshes = cell_object(robot, True, False).draw(state)  # noqa: F821

    if show_robot_collision_meshes:  # noqa: F821
        robot_collision_meshes = cell_object(robot, False, True).draw(state)  # noqa: F821

    # Rigid bodies and attached tools, drawn from the cell rather than read
    # back from MoveIt. Pulled off the child scene objects so each output can
    # be wired separately.
    if show_collision_meshes or show_attached_collision_meshes:  # noqa: F821
        so = cell_object(robot, True, False)
        so.update(state)

        if show_collision_meshes:  # noqa: F821
            collision_meshes = []
            for rb in so._rigid_body_scene_objects.values():
                collision_meshes.extend(rb.draw())

        if show_attached_collision_meshes:  # noqa: F821
            attached_meshes = []
            for tool_so in so._tool_scene_objects.values():
                attached_meshes.extend(tool_so.draw())
