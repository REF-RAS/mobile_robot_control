"""Attach or detach a tool on the robot.

COMPAS FAB v2.0.1

Inputs : robot, tool, attach, remove
Outputs: out, tool, bang

The original also had `M` and `ee_plane` outputs that the script never
assigned -- leftovers from copying the `tool` component. They always emitted
null and can be deleted.

WHAT CHANGED
------------
v1 attached a Tool to the Robot. In 2.x the tool model belongs to the
RobotCell and the attachment is a fact about the RobotCellState, so
MobileRobot.attach_tool updates both, then re-uploads the cell to MoveIt.
Until that upload happens the planner knows nothing about the tool, so
attaching without a planner attached changes only the local model.

ONE TOOL, ONE GROUP
-------------------
v1 looped over every planning group containing the flange and called
attach_tool for each. That does not carry over: ToolState.attached_to_group is
a single field, so attaching the same tool to a second group silently moves it
off the first, and the loop would leave it on whichever group happened to come
last.

This attaches to one group and prints every candidate, so a robot whose flange
sits in more than one group is visible rather than silently resolved. To have a
tool on two groups in 2.x, register it twice under different ids.

TOOL IDS
--------
The tool is registered under `tool.name`, so give tools distinct names --
`04_tool.py` sets them. Two tools sharing a name overwrite each other in
robot_cell.tool_models. Attaching a second tool to the same group detaches the
first automatically.
"""

FLANGE_LINK = "robot_arm_flange"

bang = None

if robot:  # noqa: F821
    groups = robot.get_group_names_from_link_name(FLANGE_LINK)  # noqa: F821

    if not groups:
        print("No planning group contains link '%s'." % FLANGE_LINK)
        print("Groups on this robot: %s" % ", ".join(robot.group_names))  # noqa: F821
    else:
        group = groups[0]
        if len(groups) > 1:
            print("Link '%s' is in %d groups: %s" % (FLANGE_LINK, len(groups), ", ".join(groups)))
            print("Using '%s'. A tool can only be attached to one group in 2.x." % group)

        if attach and tool:  # noqa: F821
            tool_id = robot.attach_tool(tool, group=group)  # noqa: F821
            state = robot.robot_cell_state.tool_states[tool_id]  # noqa: F821
            print("Attached '%s' to group '%s'." % (tool_id, group))
            print("  touch links : %s" % (", ".join(state.touch_links) or "none"))
            print("  uploaded to MoveIt: %s" % bool(robot.planner))  # noqa: F821
            bang = tool_id

        if remove:  # noqa: F821
            tool_id = robot.detach_tool(group=group)  # noqa: F821
            if tool_id:
                print("Detached '%s' from group '%s'." % (tool_id, group))
                bang = "detached"
            else:
                print("Nothing attached to group '%s'." % group)

        attached = robot.get_attached_tool(group)  # noqa: F821
        print("\nCurrently attached: %s" % (attached.name if attached else "none"))
        print("Registered in cell : %s" % (", ".join(robot.robot_cell.tool_ids) or "none"))  # noqa: F821
