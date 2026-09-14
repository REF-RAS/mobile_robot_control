"""Mobile robot wrapper.

COMPAS FAB v2.0.1

In compas_fab 1.x this class subclassed ``compas_fab.robots.Robot``, which held
the model, semantics, artist and client, and exposed planning directly on the
robot.  ``Robot`` no longer exists in 2.x.  The cell (robot model + semantics +
tools + rigid bodies) now lives in :class:`compas_fab.robots.RobotCell`, its
pose in :class:`compas_fab.robots.RobotCellState`, and planning moved onto a
planner such as :class:`compas_fab.backends.MoveItPlanner`.

``MobileRobot`` is therefore a plain wrapper holding those objects, plus the
mobile-base coordinate frame algebra (WCF / BCF / RCF) it always had.

Coordinate frames
-----------------
The MoveIt backend expects planning targets relative to the robot model's root
link, but returns forward kinematics relative to
``robot_cell_state.robot_base_frame``.  To keep a single convention, this class
leaves ``robot_base_frame`` at worldXY and performs the WCF <-> BCF conversion
itself, exactly as the v1 code did.  Pass frames in WCF to the planning helpers
(the default), or set ``frame_in_wcf=False`` to pass them in BCF.
"""

from compas.geometry import Frame, Point, Quaternion
from compas.geometry import Transformation
from compas_robots import Configuration

from compas_fab.robots import ConfigurationTarget
from compas_fab.robots import FrameTarget
from compas_fab.robots import RobotCell
from compas_fab.robots import TargetMode
from compas_fab.robots import ToolState

__all__ = ["MobileRobot"]


class MobileRobot(object):
    """A robot cell whose base can be moved in the world coordinate system.

    Parameters
    ----------
    robot_cell : :class:`compas_fab.robots.RobotCell`
        The cell holding the robot model, semantics and any tools.
    robot_cell_state : :class:`compas_fab.robots.RobotCellState`, optional
        The cell state.  Defaults to ``robot_cell.default_cell_state()``.
    scene_object : :class:`compas.scene.SceneObject`, optional
        Scene object used to draw the cell.  Replaces the v1 ``artist``.
    client : :class:`compas_fab.backends.RosClient`, optional
    mobile_client : :class:`mobile_robot_control.MobileRobotClient`, optional
    planner : :class:`compas_fab.backends.PlannerInterface`, optional
        Planner used for IK and motion planning.  If omitted and a ``client`` is
        given, call :meth:`attach_planner` to build a ``MoveItPlanner``.
    """

    def __init__(
        self,
        robot_cell,
        robot_cell_state=None,
        scene_object=None,
        client=None,
        mobile_client=None,
        planner=None,
        **kwargs
    ):
        if not isinstance(robot_cell, RobotCell):
            raise TypeError(
                "MobileRobot expects a compas_fab.robots.RobotCell. "
                "In compas_fab 2.x, load one with `ros_client.load_robot_cell(...)` "
                "instead of passing a RobotModel."
            )

        self.robot_cell = robot_cell
        self.robot_cell_state = robot_cell_state or robot_cell.default_cell_state()
        self.scene_object = scene_object
        self.client = client
        self.mobile_client = mobile_client
        self.planner = planner

        self._scale_factor = 1.0
        self.attributes = {}

        self._lift_height = 0  # lift height

        self._WCF = Frame.worldXY()  # world coordinate frame (WCF)
        self._BCF = Frame.worldXY()  # base coordinate frame in WCF (BCF)
        self._RCF = None  # ur robot arm coordinate frame in BCF (RCF)
        self._RCF_lift = None  # ur robot arm coordinate frame in BCF (RCF) with lift height

        self._RWCF = Frame.worldXY()  # reference world coordinate frame (RWCF)
        self._RBCF = Frame.worldXY()  # base coordinate frame in RWCF (RBCF)
        self._RRCF = None  # ur robot arm coordinate frame in RBCF (RRCF)

        self._PCF = (
            Frame.worldXY()
        )  # frame for element pick-up on mobile robot's base in RCF (PCF)

    # ==========================================================================
    # cell access
    # ==========================================================================

    @property
    def robot_model(self):
        """:class:`compas_robots.RobotModel` : The robot model of the cell."""
        return self.robot_cell.robot_model

    @property
    def model(self):
        """:class:`compas_robots.RobotModel` : Alias of :attr:`robot_model` (v1 name)."""
        return self.robot_cell.robot_model

    @property
    def robot_semantics(self):
        """:class:`compas_fab.robots.RobotSemantics` : The semantics of the cell."""
        return self.robot_cell.robot_semantics

    @property
    def semantics(self):
        """:class:`compas_fab.robots.RobotSemantics` : Alias of :attr:`robot_semantics` (v1 name)."""
        return self.robot_cell.robot_semantics

    @property
    def main_group_name(self):
        return self.robot_cell.main_group_name

    @property
    def group_names(self):
        return self.robot_cell.group_names

    def info(self):
        """Print information about the robot.

        v1's ``Robot.info()`` became ``RobotCell.print_info()`` in 2.x; this
        keeps the old call site working.
        """
        return self.robot_cell.print_info()

    # ----------------------------------------------------------------------
    # joints and links (v1 Robot delegates, now on RobotCell)
    # ----------------------------------------------------------------------

    def get_configurable_joints(self, group=None):
        return self.robot_cell.get_configurable_joints(group)

    def get_configurable_joint_names(self, group=None):
        return self.robot_cell.get_configurable_joint_names(group)

    def get_configurable_joint_types(self, group=None):
        return self.robot_cell.get_configurable_joint_types(group)

    def get_link_names(self, group=None):
        return self.robot_cell.get_link_names(group)

    def get_end_effector_link_name(self, group=None):
        return self.robot_cell.get_end_effector_link_name(group)

    def get_base_link_name(self, group=None):
        return self.robot_cell.get_base_link_name(group)

    def get_group_names_from_link_name(self, link_name):
        """Get the planning groups whose link chain contains ``link_name``.

        v1's ``Robot`` had this; 2.x does not, so it is derived from the
        semantics here.

        Groups whose link chain cannot be resolved (an end-effector group with
        no base-to-tip chain, for instance) raise inside ``get_link_names`` and
        are skipped rather than taking the whole query down.
        """
        groups = []
        for group in self.group_names:
            try:
                links = self.robot_cell.get_link_names(group)
            except Exception:
                continue
            if link_name in links:
                groups.append(group)
        return groups

    # ----------------------------------------------------------------------
    # tools
    # ----------------------------------------------------------------------

    @property
    def attached_tool(self):
        """:class:`compas_robots.ToolModel` : Tool attached to the main group, or ``None``."""
        return self.get_attached_tool()

    def get_attached_tool(self, group=None):
        group = group or self.main_group_name
        return self.robot_cell.get_attached_tool(self.robot_cell_state, group)

    def attach_tool(self, tool, group=None, tool_id=None, attachment_frame=None, touch_links=None):
        """Register a tool in the cell and attach it to a planning group.

        v1 attached a ``Tool`` to the robot directly. In 2.x the tool model
        belongs to the RobotCell and the attachment is a fact about the
        RobotCellState, so both are updated here.

        Parameters
        ----------
        tool : :class:`compas_robots.ToolModel`
        group : str, optional
        tool_id : str, optional
            Key under which the tool is registered. Defaults to the tool's name.
        attachment_frame : :class:`compas.geometry.Frame`, optional
        touch_links : list[str], optional
            Links allowed to collide with the tool. Defaults to the group's
            default touch links, which is what keeps the planner from reporting
            a self-collision the moment a tool is attached.

        Returns
        -------
        str
            The id the tool was registered under.
        """
        group = group or self.main_group_name
        tool_id = tool_id or getattr(tool, "name", None) or "attached_tool"

        self.robot_cell.tool_models[tool_id] = tool
        if tool_id not in self.robot_cell_state.tool_states:
            tool_state = ToolState(Frame.worldXY())
            if tool.get_configurable_joints():
                tool_state.configuration = tool.zero_configuration()
            self.robot_cell_state.tool_states[tool_id] = tool_state

        if touch_links is None:
            touch_links = self.robot_cell.default_touch_links(group)

        self.robot_cell_state.set_tool_attached_to_group(
            tool_id, group, attachment_frame, touch_links
        )
        self.sync_planner()
        return tool_id

    def detach_tool(self, group=None, tool_id=None, frame=None):
        """Detach the tool attached to ``group``. Returns its id, or ``None``."""
        group = group or self.main_group_name
        tool_id = tool_id or self.robot_cell_state.get_attached_tool_id(group)
        if tool_id is None:
            return None
        self.robot_cell_state.set_tool_detached(tool_id, frame)
        self.sync_planner()
        return tool_id

    def from_tcf_to_t0cf(self, frames_tcf, group=None):
        """Convert TCF frames to the planner coordinate frame (v1 name kept)."""
        group = group or self.main_group_name
        tool_id = self.robot_cell_state.get_attached_tool_id(group)
        if tool_id is None:
            return list(frames_tcf)
        return self.robot_cell.from_tcf_to_pcf(self.robot_cell_state, list(frames_tcf), tool_id)

    def from_t0cf_to_tcf(self, frames_t0cf, group=None):
        """Convert planner coordinate frames to TCF frames (v1 name kept)."""
        group = group or self.main_group_name
        tool_id = self.robot_cell_state.get_attached_tool_id(group)
        if tool_id is None:
            return list(frames_t0cf)
        return self.robot_cell.from_pcf_to_tcf(self.robot_cell_state, list(frames_t0cf), tool_id)

    def sync_planner(self):
        """Re-upload the cell and state to the backend, if a planner is attached.

        Changing the cell (attaching a tool, adding a rigid body) has no effect
        on planning until the backend is told about it.
        """
        if self.planner is not None:
            self.planner.set_robot_cell(self.robot_cell, self.robot_cell_state)

    @staticmethod
    def namespace_planner_services(planner, namespace):
        """Prefix a MoveIt planner's service names with a ROS namespace.

        compas_fab hardcodes unnamespaced MoveIt service names --
        ``/plan_kinematic_path``, ``/compute_ik`` and five others. A robot whose
        move_group runs inside a namespace answers on ``/<ns>/plan_kinematic_path``
        instead, so every call times out waiting for a service that nothing
        provides. The failure looks like "move_group is down" rather than
        "wrong name".

        ``ServiceDescription.name`` is read at call time, so the descriptions
        are copied onto the planner instance with the prefix applied. The class
        attributes are left alone, so this affects only this planner.

        Parameters
        ----------
        planner : :class:`compas_fab.backends.MoveItPlanner`
        namespace : str
            e.g. ``'robot'`` or ``'/robot'``. Falsy leaves the planner alone.

        Returns
        -------
        dict
            Attribute name -> the service name now used.
        """
        if not namespace:
            return {}
        prefixed = MobileRobot._prefixed_service_descriptions(type(planner), namespace)
        for attr, description in prefixed.items():
            setattr(planner, attr, description)
        return {attr: d.name for attr, d in prefixed.items()}

    @staticmethod
    def _prefixed_service_descriptions(planner_class, namespace):
        """Copies of ``planner_class``'s ServiceDescriptions, namespace applied."""
        from compas_fab.backends.ros.service_description import ServiceDescription

        prefix = "/" + str(namespace).strip("/")
        out = {}
        for cls in planner_class.__mro__:
            for attr, value in vars(cls).items():
                if isinstance(value, ServiceDescription) and attr not in out:
                    out[attr] = ServiceDescription(
                        prefix + value.name,
                        value.type,
                        value.request_class,
                        value.response_class,
                        value.validator,
                    )
        return out

    @staticmethod
    def namespaced_planner_class(namespace, planner_class=None):
        """A MoveItPlanner subclass whose service names carry ``namespace``.

        The names must be right *before* construction: ``MoveItPlanner.__init__``
        calls ``reset_planning_scene()``, which is itself a service call. Patching
        an instance afterwards is too late -- the constructor would already have
        timed out.

        Returns ``planner_class`` unchanged when ``namespace`` is falsy.
        """
        from compas_fab.backends import MoveItPlanner

        planner_class = planner_class or MoveItPlanner
        if not namespace:
            return planner_class
        return type(
            "Namespaced" + planner_class.__name__,
            (planner_class,),
            MobileRobot._prefixed_service_descriptions(planner_class, namespace),
        )

    def attach_planner(self, client=None, planner=None, robot_cell_state=None, namespace=None):
        """Attach a planner and upload the cell to the backend.

        A ROS client can be reconnected without reloading the geometry, so this
        is kept separate from ``__init__``.  Call it whenever ``client`` changes.

        Parameters
        ----------
        client : :class:`compas_fab.backends.RosClient`, optional
            Defaults to the currently assigned client.
        planner : :class:`compas_fab.backends.PlannerInterface`, optional
            Defaults to a new ``MoveItPlanner`` on ``client``.
        robot_cell_state : :class:`compas_fab.robots.RobotCellState`, optional
            State to upload with the cell.  Defaults to :attr:`robot_cell_state`.

        Returns
        -------
        :class:`compas_fab.backends.PlannerInterface`
        """
        self.client = client or self.client
        if self.client is None:
            raise ValueError("A client is required to attach a planner.")

        if planner is None:
            # compas_fab calls unnamespaced MoveIt services. Build the class
            # with the prefix baked in, because the constructor makes a service
            # call of its own (reset_planning_scene) and would otherwise hang.
            planner_class = self.namespaced_planner_class(namespace)
            planner = planner_class(self.client)
        elif namespace:
            self.namespace_planner_services(planner, namespace)

        self.planner = planner
        self.planner.set_robot_cell(
            self.robot_cell, robot_cell_state or self.robot_cell_state
        )
        return self.planner

    def _ensure_planner(self):
        if self.planner is None:
            raise ValueError(
                "No planner attached. Call `mobile_robot.attach_planner(ros_client)` first."
            )
        return self.planner

    # ==========================================================================
    # configurations and cell states
    # ==========================================================================

    def zero_configuration(self, group=None):
        """Get the zero configuration of a planning group."""
        return self.robot_cell.zero_configuration(group)

    def zero_full_configuration(self):
        """Get the zero configuration of all configurable joints of the robot."""
        return self.robot_cell.zero_full_configuration()

    def full_configuration(self, configuration, full_configuration=None):
        """Merge a group configuration into a full one.

        Replaces the v1 ``Robot.merge_group_with_full_configuration``.

        Notes
        -----
        ``RobotCell.configuration_to_full_configuration`` discards the result of
        its own ``merged`` call in compas_fab 2.0.1, so the merge is done here.
        """
        base = full_configuration or self.robot_cell.zero_full_configuration()
        base = self.robot_cell.fill_configuration_with_joint_names(base)
        return base.merged(configuration)

    def group_configuration(self, full_configuration, group=None):
        """Filter a full configuration down to the joints of a planning group."""
        group = group or self.main_group_name
        return self.robot_cell.full_configuration_to_group_configuration(
            full_configuration, group
        )

    def current_configuration(self, require_live=True):
        """The robot's *measured* configuration, from joint states.

        ``MobileRobotClient.get_current_configuration()`` substitutes 0.0 for
        every joint it has not heard about, so with no subscription it returns
        a perfectly valid-looking configuration of all zeros. Planning from
        that is worse than not planning: the trajectory is computed from a pose
        the robot is not in, and the first move jumps.

        Parameters
        ----------
        require_live : bool, optional
            ``True`` (default) returns ``None`` unless joint states have
            actually arrived. ``False`` gives the raw call, zeros and all.

        Returns
        -------
        :class:`compas_robots.Configuration` or None
        """
        if self.mobile_client is None:
            return None

        values = self.mobile_client.current_joint_values
        if require_live and not values:
            return None

        # Built from the MODEL's joints, intersected with what actually
        # arrived -- never from a hardcoded name list.
        #
        # MoveIt does not reject an unknown joint name, it aborts on one:
        #   Variable 'robot_ewellix_lift_top_joint' is not known to model
        #   'rbvogui_xl_plus' -> terminate called -> move_group dies (SIGABRT).
        # A stale name in a client-side list therefore takes the robot's
        # planner down, not just the request. Since the model comes from the
        # live URDF, intersecting with it makes that impossible.
        joint_values, joint_types, joint_names = [], [], []
        for joint in self.robot_cell.get_all_configurable_joints():
            if joint.name in values:
                joint_values.append(values[joint.name])
                joint_types.append(joint.type)
                joint_names.append(joint.name)

        if not joint_names:
            return None
        return Configuration(joint_values, joint_types, joint_names)

    def unknown_joint_names(self, configuration):
        """Joint names in ``configuration`` that the robot model does not have.

        Anything returned here would abort move_group if it reached
        ``/apply_planning_scene``, so callers should treat a non-empty result
        as fatal to the request rather than passing it on.
        """
        if not configuration or not configuration.joint_names:
            return []
        known = {j.name for j in self.robot_cell.get_all_configurable_joints()}
        return [n for n in configuration.joint_names if n not in known]

    def cell_state_at(self, configuration=None, group=None):
        """Get a copy of the cell state with ``configuration`` applied.

        Parameters
        ----------
        configuration : :class:`compas_robots.Configuration`, optional
            A group or full configuration.  A group configuration is merged into
            the current full configuration.  Defaults to the current state.
        group : str, optional
            Unused for now; accepted so callers can be explicit about the group
            the configuration belongs to.

        Returns
        -------
        :class:`compas_fab.robots.RobotCellState`
        """
        state = self.robot_cell_state.copy()
        if configuration is not None:
            # Refuse to build a state around a joint the model does not have.
            # Such a state, sent to /apply_planning_scene, aborts move_group
            # outright (SIGABRT, not an error response) -- so failing loudly
            # here is far cheaper than the robot's planner dying.
            unknown = self.unknown_joint_names(configuration)
            if unknown:
                raise ValueError(
                    "Configuration contains joints the robot model does not have: "
                    "%s.\nModel '%s' has: %s.\nSending these to MoveIt would abort "
                    "move_group, so the request is refused here."
                    % (
                        ", ".join(unknown),
                        self.robot_model.name,
                        ", ".join(self.robot_cell.get_all_configurable_joint_names()),
                    )
                )
            state.robot_configuration = self.full_configuration(
                configuration, state.robot_configuration
            )
        return state

    def base_frame_for_link_at(self, frame, link_name, configuration=None):
        """Get the BCF that puts ``link_name`` on ``frame``.

        ``robot_base_frame`` positions the model's *root* link, so the robot is
        anchored wherever the URDF happens to root itself -- which is not
        necessarily the point you want to drive the mobile base from. This
        solves for the root placement that lands ``link_name`` on ``frame``
        instead.

        Parameters
        ----------
        frame : :class:`compas.geometry.Frame`
            Where ``link_name`` should end up, in WCF.
        link_name : str
            The link to anchor on, e.g. ``'robot_base_footprint'``.
        configuration : :class:`compas_robots.Configuration`, optional
            Joint values used for the internal FK. Only matters when the link
            sits beyond a movable joint (the lift, for instance).

        Returns
        -------
        :class:`compas.geometry.Frame`
            Assign to :attr:`BCF`.

        Raises
        ------
        ValueError
            If the link is not in the model.
        """
        model = self.robot_cell.robot_model
        if model.get_link_by_name(link_name) is None:
            raise ValueError(
                "No link %r in the model. Root is %r." % (link_name, self.robot_cell.root_name)
            )

        state = self.cell_state_at(configuration)
        t_root_link = Transformation.from_frame(
            model.forward_kinematics(state.robot_configuration, link_name)
        )
        t_world_link = Transformation.from_frame(frame)
        return Frame.from_transformation(t_world_link * t_root_link.inverted())

    def anchor_base_on_link(self, frame, link_name, configuration=None):
        """Set :attr:`BCF` so ``link_name`` lands on ``frame``. Returns the BCF."""
        self.BCF = self.base_frame_for_link_at(frame, link_name, configuration)
        return self.BCF

    def display_cell_state(self, configuration=None, group=None):
        """Get a cell state for *drawing*, with the base placed at the BCF.

        Planning keeps ``robot_base_frame`` at worldXY (see the module
        docstring), but ``RobotCellObject.update`` positions the robot from that
        very attribute -- so drawing the planning state would park the mobile
        base at the world origin.  Use this state for the scene object.

        Returns
        -------
        :class:`compas_fab.robots.RobotCellState`
        """
        state = self.cell_state_at(configuration, group)
        state.robot_base_frame = self.BCF
        return state

    # ==========================================================================
    # kinematics and planning
    # ==========================================================================

    def forward_kinematics(
        self, configuration=None, group=None, target_mode=TargetMode.ROBOT, in_wcf=False
    ):
        """Compute forward kinematics from the robot model (no backend needed).

        Replaces the v1 ``Robot.forward_kinematics(..., options={'solver': 'model'})``.

        Parameters
        ----------
        configuration : :class:`compas_robots.Configuration`, optional
            A group or full configuration.
        group : str, optional
        target_mode : :class:`compas_fab.robots.TargetMode`, optional
            ``ROBOT`` for the planner coordinate frame, ``TOOL`` for the TCF of
            an attached tool.  Defaults to ``ROBOT``.
        in_wcf : bool, optional
            ``True`` to return the frame in WCF, ``False`` (default) to return it
            in the robot's base coordinate frame (BCF), matching v1.

        Returns
        -------
        :class:`compas.geometry.Frame`
        """
        state = self.cell_state_at(configuration, group)
        frame_BCF = self.robot_cell.forward_kinematics_target_frame(
            state, target_mode, group
        )
        return self.from_BCF_to_WCF(frame_BCF) if in_wcf else frame_BCF

    def inverse_kinematics(
        self,
        frame,
        start_configuration=None,
        group=None,
        target_mode=TargetMode.ROBOT,
        frame_in_wcf=True,
        options=None,
    ):
        """Compute an inverse kinematics solution via the backend planner.

        Parameters
        ----------
        frame : :class:`compas.geometry.Frame`
            The target frame, in WCF unless ``frame_in_wcf`` is ``False``.
        start_configuration : :class:`compas_robots.Configuration`, optional
        group : str, optional
        target_mode : :class:`compas_fab.robots.TargetMode`, optional
        frame_in_wcf : bool, optional
        options : dict, optional
            Passed through to the planner.

        Returns
        -------
        :class:`compas_robots.Configuration`
        """
        planner = self._ensure_planner()
        group = group or self.main_group_name

        frame_BCF = self.from_WCF_to_BCF(frame) if frame_in_wcf else frame
        target = FrameTarget(frame_BCF, target_mode)
        start_state = self.cell_state_at(start_configuration, group)

        return planner.inverse_kinematics(target, start_state, group, options)

    def plan_motion_to_configuration(
        self,
        target_configuration,
        start_configuration=None,
        group=None,
        tolerance_above=None,
        tolerance_below=None,
        options=None,
    ):
        """Plan a motion to a joint configuration.

        Replaces the v1 ``constraints_from_configuration`` + ``plan_motion`` pair.

        Returns
        -------
        :class:`compas_fab.robots.JointTrajectory`
        """
        planner = self._ensure_planner()
        group = group or self.main_group_name

        target = ConfigurationTarget(
            target_configuration,
            tolerance_above=tolerance_above,
            tolerance_below=tolerance_below,
        )
        start_state = self.cell_state_at(start_configuration, group)

        return planner.plan_motion(target, start_state, group, options)

    def plan_motion_to_frame(
        self,
        frame,
        start_configuration=None,
        group=None,
        target_mode=TargetMode.ROBOT,
        frame_in_wcf=True,
        tolerance_position=None,
        tolerance_orientation=None,
        options=None,
    ):
        """Plan a motion to a target frame.

        Replaces the v1 ``constraints_from_frame`` + ``plan_motion`` pair.

        Notes
        -----
        v1 took three per-axis orientation tolerances; ``FrameTarget`` takes a
        single ``tolerance_orientation`` applied to all three axes.

        Returns
        -------
        :class:`compas_fab.robots.JointTrajectory`
        """
        planner = self._ensure_planner()
        group = group or self.main_group_name

        frame_BCF = self.from_WCF_to_BCF(frame) if frame_in_wcf else frame
        target = FrameTarget(
            frame_BCF,
            target_mode,
            tolerance_position=tolerance_position,
            tolerance_orientation=tolerance_orientation,
        )
        start_state = self.cell_state_at(start_configuration, group)

        return planner.plan_motion(target, start_state, group, options)

    # ==========================================================================
    # coordinate frames
    # ==========================================================================

    @property
    def lift_height(self):
        return self._lift_height

    @lift_height.setter
    def lift_height(self, lift_height):
        self._lift_height = lift_height

    @property
    def PCF(self):
        return self._PCF

    @PCF.setter
    def PCF(self, PCF):
        self._PCF = PCF

    @property
    def BCF(self):
        return self._BCF

    @BCF.setter
    def BCF(self, BCF):
        self._BCF = BCF

    #: Link the arm is mounted on, and the frame RCF is expressed relative to.
    ARM_BASE_LINK = "robot_arm_base"
    BASE_FOOTPRINT_LINK = "robot_base_footprint"

    def compute_RCF_from_model(self, configuration=None, link_name=None):
        """Compute the arm base frame from the URDF instead of from TF.

        The TF route needs tf2_web_republisher, which serves the
        ``/republish_tfs`` service roslibpy's TFClient calls. That is a ROS 1
        package whose ROS 2 build is a community port, and it is often simply
        not running -- in which case TF itself is perfectly healthy but
        ``robot.RCF`` stays None forever.

        None of that is necessary: the robot model already describes where the
        arm sits, and the lift joint value comes from the joint states we
        already subscribe to. Forward kinematics to the arm base link gives the
        same frame, offline and deterministically.

        Parameters
        ----------
        configuration : :class:`compas_robots.Configuration`, optional
            Defaults to the live configuration from ``mobile_client`` if one is
            connected, otherwise the cell state's configuration.
        link_name : str, optional
            Defaults to :attr:`ARM_BASE_LINK`.

        Returns
        -------
        :class:`compas.geometry.Frame` or None
            The arm base relative to the robot model's root link, or None if
            the link does not exist in the model.

        Notes
        -----
        The returned frame already accounts for the lift, because the lift
        joint is part of the configuration. Leave :attr:`lift_height` at 0 when
        using this, or the lift is counted twice.
        """
        link_name = link_name or self.ARM_BASE_LINK
        if self.robot_cell.robot_model.get_link_by_name(link_name) is None:
            return None

        if configuration is None and self.mobile_client is not None:
            try:
                configuration = self.mobile_client.get_current_configuration()
            except Exception:
                configuration = None

        full = self.full_configuration(
            configuration, self.robot_cell_state.robot_configuration
        ) if configuration is not None else self.robot_cell_state.robot_configuration

        return self.robot_cell.robot_model.forward_kinematics(full, link_name)

    @property
    def RCF(self):
        """The arm base frame, in the robot's base coordinate frame (BCF).

        Tries TF first (needs tf2_web_republisher), then falls back to the
        robot model, which always works. See :meth:`compute_RCF_from_model`.
        """
        if self._RCF is None and self.mobile_client is not None:
            self.mobile_client.tf_subscribe(
                self.ARM_BASE_LINK,
                self.BASE_FOOTPRINT_LINK,
                self._receive_base_frame_callback,
                timeout=5,
            )

        if self._RCF is None:
            self._RCF = self.compute_RCF_from_model()

        if self._RCF is None:
            # Previously this fell through to `self._RCF.copy()` below and
            # raised AttributeError on None, which read as a compas bug rather
            # than a missing transform.
            return None

        if self.lift_height:
            self._RCF_lift = self._RCF.copy()
            self._RCF_lift.point.z += self.lift_height
        else:
            self._RCF_lift = self._RCF
        return self._RCF_lift

    def _receive_base_frame_callback(self, message):
        pose_point = Point(
            message["translation"]["x"],
            message["translation"]["y"],
            message["translation"]["z"],
        )
        pose_quaternion = Quaternion(
            message["rotation"]["w"],
            message["rotation"]["x"],
            message["rotation"]["y"],
            message["rotation"]["z"],
        )
        pose_frame = Frame.from_quaternion(pose_quaternion, pose_point)
        self._RCF = pose_frame
        return self._RCF

    @property
    def RWCF(self):
        """Get the reference world coordinate frame.
        :class:`compas.geometry.Frame`
        """
        return self._RWCF

    def transformation_BCF_WCF(self):
        """Get the transformation from the base coordinate frame (BCF) to the world coordinate frame (WCF).
        -------
        :class:`compas.geometry.Transformation`
        """
        return Transformation.from_change_of_basis(self.BCF, Frame.worldXY())

    def transformation_WCF_BCF(self):
        """Get the transformation from the world coordinate frame (WCF) to the base coordinate frame (BCF).
        -------
        :class:`compas.geometry.Transformation`
        """
        return Transformation.from_change_of_basis(Frame.worldXY(), self.BCF)

    def transformation_RCF_WCF(self):
        """Get the transformation from the robot arm coordinate frame (RCF) to the world coordinate frame (WCF).
        -------
        :class:`compas.geometry.Transformation`
        """
        return Transformation.concatenated(
            self.transformation_BCF_WCF(), self.transformation_RCF_BCF()
        )

    def transformation_WCF_RCF(self):
        """Get the transformation from the world coordinate frame (WCF) to the robot arm coordinate frame (RCF).
        -------
        :class:`compas.geometry.Transformation`
        """
        return Transformation.concatenated(
            self.transformation_BCF_RCF(), self.transformation_WCF_BCF()
        )

    def transformation_RCF_BCF(self):
        """Get the transformation from the robot arm frame (RCF) to the base coordinate frame (BCF).
        -------
        :class:`compas.geometry.Transformation`
        """
        return Transformation.from_frame(self.RCF)

    def transformation_BCF_RCF(self):
        """Get the transformation from the base coordinate frame (BCF) to the robot arm frame (RCF).
        -------
        :class:`compas.geometry.Transformation`
        """
        return Transformation.from_frame(self.RCF).inverted()

    def transformation_RBCF_WCF(self):
        """Get the transformation from the reference base coordinate frame (RBCF) to the world coordinate frame (WCF).
        -------
        :class:`compas.geometry.Transformation`
        """
        frame_RBCF_in_WCF = self.RWCF.to_world_coordinates(self._RBCF)
        return Transformation.from_change_of_basis(frame_RBCF_in_WCF, Frame.worldXY())

    def transformation_WCF_RBCF(self):
        """Get the transformation from the world coordinate frame (WCF) to the reference base coordinate frame (RBCF).
        -------
        :class:`compas.geometry.Transformation`
        """
        frame_RBCF_in_WCF = self.RWCF.to_world_coordinates(self._RBCF)
        return Transformation.from_change_of_basis(Frame.worldXY(), frame_RBCF_in_WCF)

    def transformation_RWCF_WCF(self):
        """Get the transformation from the reference world coordinate frame (RWCF) to the world coordinate frame (WCF).
        -------
        :class:`compas.geometry.Transformation`
        """
        return Transformation.from_change_of_basis(self.RWCF, Frame.worldXY())

    def transformation_WCF_RWCF(self):
        """Get the transformation from the world coordinate frame (WCF) to the reference world coordinate frame (RWCF).
        -------
        :class:`compas.geometry.Transformation`
        """
        return Transformation.from_change_of_basis(Frame.worldXY(), self.RWCF)

    def from_WCF_to_BCF(self, frame_WCF):
        """Represent a frame from the world coordinate system (WCF) in the robot base coordinate system (BCF).
        Parameters
        ----------
        frame_WCF : :class:`compas.geometry.Frame`
            A frame in the world coordinate frame.
        Returns
        -------
        frame_BCF : :class:`compas.geometry.Frame`
            A frame in the robot base coordinate frame.
        """
        frame_BCF = frame_WCF.transformed(self.transformation_WCF_BCF())
        return frame_BCF

    def from_BCF_to_WCF(self, frame_BCF):
        """Represent a frame from the robot's base coordinate system (BCF) in the world coordinate system (WCF).
        Parameters
        ----------
        frame_BCF : :class:`compas.geometry.Frame`
            A frame in the robot base coordinate frame.
        Returns
        -------
        frame_WCF : :class:`compas.geometry.Frame`
            A frame in the world coordinate frame.
        """
        frame_WCF = frame_BCF.transformed(self.transformation_BCF_WCF())
        return frame_WCF

    def from_WCF_to_RCF(self, frame_WCF):
        """Represent a frame from the world coordinate system (WCF) in the robot arm coordinate system (RCF).
        Parameters
        ----------
        frame_WCF : :class:`compas.geometry.Frame`
            A frame in the world coordinate frame.
        Returns
        -------
        frame_RCF : :class:`compas.geometry.Frame`
            A frame in the robot arm coordinate frame.
        """
        frame_RCF = frame_WCF.transformed(self.transformation_WCF_RCF())
        return frame_RCF

    def from_RCF_to_WCF(self, frame_RCF):
        """Represent a frame from the robot arm coordinate system (RCF) in the world coordinate system (WCF).
        Parameters
        ----------
        frame_RCF : :class:`compas.geometry.Frame`
            A frame in the robot arm coordinate frame.
        Returns
        -------
        frame_WCF : :class:`compas.geometry.Frame`
            A frame in the world coordinate frame.
        """
        frame_WCF = frame_RCF.transformed(self.transformation_RCF_WCF())
        return frame_WCF

    def from_RCF_to_BCF(self, frame_RCF):
        """Represent a frame from the robot arm coordinate system (RCF) in the robot base coordinate system (BCF).
        Parameters
        ----------
        frame_RCF : :class:`compas.geometry.Frame`
            A frame in the robot arm coordinate frame.
        Returns
        -------
        frame_BCF : :class:`compas.geometry.Frame`
            A frame in the robot base coordinate frame.
        """
        frame_BCF = frame_RCF.transformed(self.transformation_RCF_BCF())
        return frame_BCF

    def from_BCF_to_RCF(self, frame_BCF):
        """Represent a frame from the robot base coordinate system (BCF) in the robot arm coordinate system (RCF).
        Parameters
        ----------
        frame_BCF : :class:`compas.geometry.Frame`
            A frame in the robot base coordinate frame.
        Returns
        -------
        frame_RCF : :class:`compas.geometry.Frame`
            A frame in the robot arm coordinate frame.
        """
        frame_RCF = frame_BCF.transformed(self.transformation_BCF_RCF())
        return frame_RCF

    def transform_frame_from_RCF_to_BCF(self, frame_WCF):
        """Apply the transformation between RCF and BCF to a frame."""
        inverted_RCF = Frame.from_transformation(self.transformation_BCF_RCF())
        return inverted_RCF.transformed(Transformation.from_frame(frame_WCF))

    def from_WCF_to_RWCF(self, frame_WCF):
        """Represent a frame from the world coordinate system (WCF) in the reference world coordinate system (RWCF).
        Parameters
        ----------
        frame_WCF : :class:`compas.geometry.Frame`
            A frame in the world coordinate frame.
        Returns
        -------
        :class:`compas.geometry.Frame`
            A frame in the robot's coordinate frame.
        """
        frame_RWCF = frame_WCF.transformed(self.transformation_WCF_RWCF())
        return frame_RWCF
