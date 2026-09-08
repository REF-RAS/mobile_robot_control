"""A tool carrying several TCFs.

COMPAS FAB v2.0.1

``compas_fab.robots.Tool`` was removed in compas_fab 2.x. Its role is now filled
by :class:`compas_robots.ToolModel`, which takes the same constructor arguments
but *is* the model rather than wrapping one -- so the frame lives on the tool
itself (``tool.frame``) instead of on ``tool.tool_model.frame``.

Register the tool into a robot cell under an id, then attach it in the cell
state::

    robot_cell.tool_models["multitool"] = multitool
    robot_cell_state = robot_cell.default_cell_state()
    robot_cell_state.set_tool_attached_to_group("multitool", group)
"""

from compas_robots import ToolModel

__all__ = ["MultiTool"]


class MultiTool(ToolModel):
    def __init__(self, visual, tool_frames, primary_tool_name="main", collision=None, name="attached_tool", connected_to=None):
        self.primary_tool_name = primary_tool_name
        self.tool_frames = tool_frames
        frame_in_tool0_frame = tool_frames.get(primary_tool_name)
        super(MultiTool, self).__init__(visual, frame_in_tool0_frame, collision, name, connected_to)

    def set_active_tool_frame(self, tool_name="main"):
        frame_in_tool0_frame = self.tool_frames.get(tool_name, None)
        if frame_in_tool0_frame is None:
            raise KeyError("tool_name not recognized in MultiTool, please ensure tool_name:tool_frame is available in Multitool.tool_frames")

        self.frame = frame_in_tool0_frame
        return frame_in_tool0_frame

    def add_tool_frame(self, tool_frame, tool_name):
        self.tool_frames.update({tool_name: tool_frame})
