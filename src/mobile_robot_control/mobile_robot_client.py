import time
import math
from compas.geometry import Frame, Point, Quaternion, Vector, Transformation

# from compas_fab.backends import RosClient
from compas_fab.backends.ros.messages import (
    JointTrajectory,
    JointTrajectoryPoint,
    Header,
)
from compas_fab.robots.time_ import Duration
from compas_robots import Configuration
from roslibpy import Message, Topic, Service, tf
from roslibpy.core import ServiceRequest

from threading import Timer

__all__ = ["AttrDict", "MobileRobotClient"]


class AttrDict(dict):
    def __init__(self, *args, **kwargs):
        super(AttrDict, self).__init__(*args, **kwargs)
        self.__dict__ = self


class MobileRobotClient(object):
    def __init__(self, ros_client=None):
        """_summary_

        Args:
            host (str, optional): IP address of ROS master. Defaults to 'localhost'.
            port (int, optional): Port of ROS master. Defaults to 9090.
        """
        self.ros_client = ros_client
        self.topics = {}
        self.services = {}
        self.tf_clients = {}
        # (reference_frame, target_frame) -> the callback actually registered,
        # so tf_unsubscribe can remove the right one.
        self._tf_callbacks = {}
        self.action_clients = {}

        self.cmd_vel = AttrDict(
            linear=AttrDict(x=0.0, y=0.0, z=0.0), angular=AttrDict(x=0.0, y=0.0, z=0.0)
        )
        self.tf_frame = None

        self.marker_frames = {}
        self.robot_frame = Frame.worldXY()

        self.current_joint_values = {}

    def connect(self):
        """_summary_"""
        self.ros_client.run()
        print("Is ROS connected? ", self.ros_client.is_connected)

    def disconnect(self):
        """_summary_"""
        self.ros_client.close()

    def tf_subscribe(self, target_frame, reference_frame, callback=None, timeout=None):
        """_summary_

        Args:
            target_frame (str): Name of the target frame requested.
            reference_frame (str): Name of the reference frame requested.
        """
        if not self.tf_clients.get(reference_frame):
            tf_client = tf.TFClient(
                self.ros_client,
                fixed_frame=reference_frame,
                angular_threshold=0.0,
                rate=10.0,
            )
            self.tf_clients[reference_frame] = tf_client
        else:
            tf_client = self.tf_clients.get(reference_frame)
        if callback is None:
            callback = self._receive_tf_frame_callback
        tf_client.subscribe(target_frame, callback)
        self._tf_callbacks[(reference_frame, target_frame)] = callback
        if timeout:
            # NOTE: this used to read
            #     Timer(timeout, self.tf_unsubscribe(target_frame, reference_frame)).start()
            # where the parentheses call tf_unsubscribe immediately and hand its
            # return value (None) to Timer as the function to run. So the
            # unsubscribe fired at subscribe time instead of after `timeout`,
            # and the timer then raised TypeError on None in a background
            # thread. Pass the function and its arguments separately.
            Timer(timeout, self.tf_unsubscribe, args=(target_frame, reference_frame)).start()

    def _receive_tf_frame_callback(self, message):
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
        self.tf_frame = pose_frame

    def clean_tf_frame(self):
        self.tf_frame = None

    def tf_unsubscribe(self, target_frame, reference_frame):
        """_summary_

        Args:
            target_frame (str): Name of target frame. e.g. 'marker_0'
            reference_frame (str): Name of reference frame. e.g. 'base'
        """
        if self.tf_clients.get(reference_frame):
            tf_client = self.tf_clients.get(reference_frame)
            # Unsubscribe the callback that was actually registered. This used
            # to always pass _receive_tf_frame_callback, so a subscription made
            # with any other callback (MobileRobot.RCF uses its own) was never
            # matched.
            callback = self._tf_callbacks.pop(
                (reference_frame, target_frame), self._receive_tf_frame_callback
            )
            try:
                tf_client.unsubscribe(target_frame, callback)
            except (TypeError, KeyError, ValueError):
                # roslibpy's TFClient.unsubscribe does frame["cbs"].pop(callback),
                # i.e. list.pop() with a callable as the index, which raises
                # TypeError before it can drop the frame. Nothing to do about
                # that here beyond not letting it propagate.
                pass

    def service_provide(self, service_name, service_type, handler=None):
        """Start advertising the service.
        This turns the instance from a client into a server. The callback will be invoked with every request that is made to the service.
        If the service is already advertised, this call does nothing.

        Args:
            service_name (str): Service name. e.g. '/set_ludicrous_speed'
            service_type (str): Sevice type. e.g. 'std_srvs/SetBool'
            handler (func, optional): Callback invoked on every service call. It should accept two parameters: service_request and service_response. It should return True if executed correctly, otherwise False. Defaults to None.
            e.g. def handler(request, response):
                    print('Setting speed to {}'.format(request['data']))
                    response['success'] = True
                    return True
        """
        if service_name not in self.services.keys():
            self.set_service(service_name, service_type)
        if not self.get_service(service_name).is_advertised:
            self.get_service(service_name).advertise(handler)

    def service_unprovide(self, service_name):
        if service_name in self.services.keys():
            if self.get_service(service_name).is_advertised:
                self.get_service(service_name).unadvertise()
            self.remove_service(service_name)

    def service_call(self, service_name, service_type, request_dict):
        """Start a service call.

        Args:
            service_name (str): Service name. e.g. '/set_ludicrous_speed'
            service_type (str): Sevice type. e.g. 'std_srvs/SetBool'
            request_dict (dict): Answer to te request as a dictionary. e.g. {'data': True}

        Returns:
            result (dict): Service response.
        """
        if service_name not in self.services.keys():
            self.set_service(service_name, service_type)
        service = self.get_service(service_name)
        request = ServiceRequest(request_dict)
        result = service.call(request)
        return result

    def get_service(self, service_name):
        return self.services.get(service_name)

    def set_service(self, service_name, service_type):
        self.services[service_name] = Service(
            self.ros_client, service_name, service_type
        )
        return self.services[service_name]

    def remove_service(self, service_name):
        self.services.pop(service_name)

    def is_service_available(self, service_name):
        all_services = self.ros_client.get_services()
        if service_name in all_services:
            return True
        else:
            return False

    def topic_subscribe(self, topic_name, msg_type=None, callback=None):
        if topic_name not in self.topics.keys():
            self.set_topic(topic_name, msg_type)
        if not self.topics[topic_name].is_subscribed:
            self.topics[topic_name].subscribe(callback)

    def topic_unsubscribe(self, topic_name):
        if topic_name in self.topics.keys():
            if self.topics[topic_name].is_subscribed:
                self.topics[topic_name].unsubscribe()
            self.remove_topic(topic_name)

    def topic_publish(self, topic_name, msg_type=None, msg_dict=None):
        if topic_name not in self.topics.keys():
            self.set_topic(topic_name, msg_type)
        if not self.topics[topic_name].is_advertised:
            msg = Message(msg_dict)
            self.topics[topic_name].publish(msg)

    def topic_unpublish(self, topic_name):
        if topic_name in self.topics.keys():
            if self.topics[topic_name].is_advertised:
                self.topics[topic_name].unadvertise()
            self.remove_topic(topic_name)

    def get_topic(self, topic_name):
        return self.topics[topic_name]

    def set_topic(self, topic_name, msg_type):
        self.topics[topic_name] = Topic(self.ros_client, topic_name, msg_type)
        return self.topics[topic_name]

    def remove_topic(self, topic_name):
        self.topics.pop(topic_name)

    def print_msg_callback(self, message):
        print(message["data"])

    def load_from_robot(self, load_geometry=True, **kwargs):
        # compas_fab 2.x: `load_robot` was replaced by `load_robot_cell`, which
        # returns a RobotCell (model + semantics) instead of a Robot.
        self.robot_cell = self.ros_client.load_robot_cell(load_geometry=load_geometry, **kwargs)
        return self.robot_cell

    def load_from_urdf(self):
        raise NotImplementedError

    def condition_odometry(self):
        # callback = some definition
        # self.topic_subscriber('/robot/robotnik_base_control', callback)
        # check the odom value vs beginning
        raise NotImplementedError

    def condition_laser(self):
        # self.topic_subscriber('/robot/front_3d_laser/points', callback)
        raise NotImplementedError

    def cmd_vel_clear(self):
        self.cmd_vel.linear.x = 0.0
        self.cmd_vel.linear.y = 0.0
        self.cmd_vel.linear.z = 0.0
        self.cmd_vel.angular.x = 0.0
        self.cmd_vel.angular.y = 0.0
        self.cmd_vel.angular.z = 0.0

    def _receive_joint_states(self, message):
        for key, joint_name in enumerate(message.get("name")):
            self.current_joint_values[joint_name] = message.get("position")[key]

    #: Topics carrying joint states.
    #:
    #: The ROS 2 bringup aggregates every joint onto one topic. The per-controller
    #: topics this used to subscribe to -- /robot/arm/joint_states and
    #: /robot/lift/joint_states -- do not exist on that stack; what is there is
    #: named /robot/arm/arm_joint_states_unused and
    #: /robot/lift/lift_joint_states_unused, i.e. deliberately remapped aside.
    #: Subscribing to a non-existent topic does not raise, so this read back as
    #: an arm parked at its zero pose.
    JOINT_STATE_TOPICS = ["/robot/joint_states"]

    #: Base velocity. The controller also advertises a plain 'cmd_vel', which on
    #: this stack carries geometry_msgs/TwistStamped -- the '_unstamped' variant
    #: matches the bare linear/angular message this class builds. Check with
    #: `ros2 topic info /robot/robotnik_base_control/cmd_vel` and swap if needed.
    CMD_VEL_TOPIC = "/robot/robotnik_base_control/cmd_vel_unstamped"

    #: Arm and lift are both joint trajectory controllers in ROS 2. The ROS 1
    #: names ('scaled_pos_traj_controller/command', and a Float64 position
    #: command for the lift) do not exist on this robot.
    ARM_TRAJECTORY_TOPIC = "/robot/arm/scaled_joint_trajectory_controller/joint_trajectory"
    LIFT_TRAJECTORY_TOPIC = "/robot/lift/lift_joint_trajectory_controller/joint_trajectory"

    ARM_JOINT_NAMES = [
        "robot_arm_shoulder_pan_joint",
        "robot_arm_shoulder_lift_joint",
        "robot_arm_elbow_joint",
        "robot_arm_wrist_1_joint",
        "robot_arm_wrist_2_joint",
        "robot_arm_wrist_3_joint",
    ]
    #: The lift moved from robotnik_description to ewellix_description, and with
    #: it from one joint to two. The old name 'robot_ewellix_lift_top_joint' is
    #: not in the current model -- sending it to MoveIt aborts move_group.
    #: Confirm against `ros2 topic echo /robot/joint_states --once`.
    LIFT_JOINT_NAMES = ["robot_lift_lower_joint", "robot_lift_upper_joint"]

    #: Single-joint alias kept for set_lift_height, which commands one joint.
    LIFT_JOINT_NAME = "robot_lift_upper_joint"

    def ros_message_type(self, package, name):
        """Build a message type name for the connected ROS version.

        ROS 2 rosbridge wants the three-part form 'sensor_msgs/msg/JointState';
        ROS 1 wants 'sensor_msgs/JointState'. Getting it wrong does not raise:
        a subscription simply never receives, and a publication never arrives.
        That silence is what made the arm read back as parked at zero, so the
        name is derived from the detected distro rather than hard-coded.
        """
        try:
            if self.ros_client.ros_distro.is_ros2:
                return "{}/msg/{}".format(package, name)
        except Exception:
            pass
        return "{}/{}".format(package, name)

    def joint_state_message_type(self):
        """The JointState type name this ROS version expects."""
        return self.ros_message_type("sensor_msgs", "JointState")

    def joint_states_subscribe(self, topics=None, message_type=None):
        """Subscribe to the joint state topics.

        Parameters
        ----------
        topics : list[str], optional
            Defaults to :attr:`JOINT_STATE_TOPICS`.
        message_type : str, optional
            Defaults to :meth:`joint_state_message_type`.
        """
        topics = topics or self.JOINT_STATE_TOPICS
        message_type = message_type or self.joint_state_message_type()
        for topic in topics:
            self.topic_subscribe(topic, message_type, self._receive_joint_states)
        return message_type

    def joint_states_unsubscribe(self, topics=None):
        for topic in topics or self.JOINT_STATE_TOPICS:
            self.topic_unsubscribe(topic)

    def joint_states_received(self):
        """How many joint values have arrived so far. 0 means nothing is coming."""
        return len(self.current_joint_values)

    def get_current_configuration(self, joint_names=None):
        """Configuration in lift-then-arm order, for the URScript path.

        Only joints actually present in ``current_joint_values`` are included.
        A name that has not been heard from is dropped rather than filled with
        0.0, because an invented joint value is indistinguishable from a real
        one, and a name the robot model does not have will abort move_group if
        it reaches /apply_planning_scene.

        For planning, prefer ``MobileRobot.current_configuration()``, which
        derives its names from the loaded URDF instead of the lists below.

        Parameters
        ----------
        joint_names : list[str], optional
            Ordered names to include. Defaults to
            :attr:`LIFT_JOINT_NAMES` + :attr:`ARM_JOINT_NAMES`.
        """
        joint_names = joint_names or (list(self.LIFT_JOINT_NAMES) + list(self.ARM_JOINT_NAMES))

        names, values, types = [], [], []
        for name in joint_names:
            if name not in self.current_joint_values:
                continue
            names.append(name)
            values.append(self.current_joint_values[name])
            # 2 == prismatic for the lift joints, 0 == revolute for the arm.
            types.append(2 if name in self.LIFT_JOINT_NAMES else 0)

        return Configuration(values, types, names)

    def echo_robot_odom(self):
        pass
        # rod = self.topic_subscriber(name='/robot/odom',
        #                             msg='geometry_msgs/Pose3D',
        #                             callback=lambda message: print(message['data']))

    def list_controllers(self):
        list_controllers_service = Service(
            self.ros_client,
            "/robot/controller_manager/list_controllers",
            "/robot/controller_manager/list_controllers",
        )
        request = ServiceRequest()
        print(list_controllers_service.call(request))

    def move_forward(self, vel=0.01, dist=0.1):
        move_base = self.topic_publish(self.CMD_VEL_TOPIC, self.ros_message_type("geometry_msgs", "Twist"))
        self.cmd_vel.linear.x = vel * (dist / abs(dist))
        t0 = time.time()
        while abs(dist) > (time.time() - t0) * vel:
            move_base.publish(Message(self.cmd_vel))
            time.sleep(0.01)
        self.cmd_vel_clear()
        T = Transformation.from_frame(Frame([dist, 0, 0], [1, 0, 0], [0, 1, 0]))
        self.robot_frame.transform(T)
        move_base.unadvertise()

    def move_backward(self, vel=0.01, dist=-0.1):
        self.move_forward(vel=vel, dist=-dist)

    def move_radial(self, deg=90, vel=0.1, dist=0.1):
        move_base = self.topic_publish(self.CMD_VEL_TOPIC, self.ros_message_type("geometry_msgs", "Twist"))
        x_vel = math.cos(math.radians(deg)) * vel
        y_vel = math.sin(math.radians(deg)) * vel
        self.cmd_vel.linear.x = x_vel
        self.cmd_vel.linear.y = y_vel
        t0 = time.time()
        while dist > (time.time() - t0) * abs(vel):
            move_base.publish(Message(self.cmd_vel))
            time.sleep(0.1)
        self.cmd_vel_clear()
        T = Transformation.from_frame(
            Frame([dist * x_vel / vel, dist * y_vel / vel, 0], [1, 0, 0], [0, 1, 0])
        )
        self.robot_frame.transform(T)
        move_base.unadvertise()

    def arm_move_joint(
        self,
        configuration,
        max_velocity=[0.2, 0.2, 0.2, 0.2, 0.2, 0.2],
        acceleration=[1, 1, 1, 1, 1, 1],
    ):
        joint_state_publisher = Topic(
            self.ros_client,
            self.ARM_TRAJECTORY_TOPIC,
            self.ros_message_type("trajectory_msgs", "JointTrajectory"),
        )
        joint_state_publisher.advertise()
        treq = [pos / vel for pos, vel in zip(configuration.joint_values, max_velocity)]
        treq_max = max(*treq)
        vreq = [pos / treq_max for pos in configuration.joint_values]
        rostime = self.ros_client.get_time()
        # compas 2.x dropped Data.from_data()/.data in favour of __from_data__/__data__.
        rostime1 = Duration.__from_data__(rostime)
        rostime1.secs += treq_max
        rostime1.nsecs += treq_max
        rostime2 = Duration.__from_data__(rostime1.__data__)
        rostime2.secs += treq_max
        rostime2.nsecs += treq_max

        jtp = JointTrajectoryPoint(
            positions=configuration.joint_values,
            velocities=[1, 1, 1, 1, 1, 1],
            accelerations=[1, 1, 1, 1, 1, 1],
            time_from_start=rostime1.__data__,
        )
        jtp0 = JointTrajectoryPoint(
            positions=[0, 0, 0, 0, 0, 0],
            velocities=[0, 0, 0, 0, 0, 0],
            accelerations=[0, 0, 0, 0, 0, 0],
            time_from_start=rostime,
        )
        jtp1 = JointTrajectoryPoint(
            positions=[0, 0, 0, 0, 0, 0],
            velocities=[0, 0, 0, 0, 0, 0],
            accelerations=[0, 0, 0, 0, 0, 0],
            time_from_start=rostime2.__data__,
        )
        jt = JointTrajectory(
            header=Header(stamp=rostime, frame_id=""),
            joint_names=list(self.ARM_JOINT_NAMES),
            points=[jtp0, jtp, jtp1],
        )
        # print(jtp.msg)
        # print(jt.msg)
        # msg={'header': {'seq': 0, 'stamp': {'secs': 0, 'nsecs': 0}, 'frame_id': '/world'},
        #      'joint_names': ['robot_arm_shoulder_pan_joint', 'robot_arm_shoulder_lift_joint',
        #                      'robot_arm_elbow_joint', 'robot_arm_wrist_1_joint',
        #                      'robot_arm_wrist_2_joint', 'robot_arm_wrist_3_joint'],
        #      'points': [{'positions': [0.0, -1.570796, 1.570796, 0.0, -1.570796, 0.0], 'velocities': [1.0, 1.0, 1.0, 1.0, 1.0, 1.0], 'accelerations': [1.0, 1.0, 1.0, 1.0, 1.0, 1.0], 'time_from_start': {'secs': 0, 'nsecs': 0}}]}
        print(jt.msg)
        t0 = time.time()
        while time.time() - t0 < treq_max * 5:
            joint_state_publisher.publish(jt.msg)
            time.sleep(0.01)
        joint_state_publisher.unadvertise()

    def set_lift_height(self, height, duration=5.0):
        """Command the lift to ``height`` metres.

        In ROS 1 this published a std_msgs/Float64 position to
        /robot/lift_joint_position_controller/command. That controller does not
        exist on the ROS 2 stack: the lift is driven by a joint trajectory
        controller, so the command is a single-point JointTrajectory instead.

        Parameters
        ----------
        height : float
            Target lift position, in metres.
        duration : float, optional
            Seconds allowed to reach it. Too short and the controller will
            reject the trajectory as infeasible.
        """
        topic = Topic(
            self.ros_client,
            self.LIFT_TRAJECTORY_TOPIC,
            self.ros_message_type("trajectory_msgs", "JointTrajectory"),
        )
        topic.advertise()

        secs = int(duration)
        message = {
            "header": {"frame_id": ""},
            "joint_names": [self.LIFT_JOINT_NAME],
            "points": [
                {
                    "positions": [float(height)],
                    "velocities": [0.0],
                    "accelerations": [0.0],
                    "time_from_start": {
                        "sec": secs,
                        "nanosec": int((duration - secs) * 1e9),
                    },
                }
            ],
        }
        topic.publish(Message(message))

        t0 = time.time()
        while time.time() - t0 < duration:
            time.sleep(0.1)
        topic.unadvertise()

    def rotate_in_place(self, rad=(math.pi / 2), vel=0.01):
        move_base = self.topic_publish(self.CMD_VEL_TOPIC, self.ros_message_type("geometry_msgs", "Twist"))
        self.cmd_vel.angular.z = vel
        t0 = time.time()
        while abs(rad) > (time.time() - t0) * abs(vel):
            move_base.publish(Message(self.cmd_vel))
            time.sleep(0.01)
        self.cmd_vel_clear()
        x_vec = [math.cos(rad), math.sin(rad), 0]
        y_vec = [-math.sin(rad), math.cos(rad), 0]
        T = Transformation.from_frame(Frame([0, 0, 0], x_vec, y_vec))
        self.robot_frame.transform(T)
        move_base.unadvertise()

    def stop_robot(self):
        self.cmd_vel_clear()
        move_base = self.topic_publish(self.CMD_VEL_TOPIC, self.ros_message_type("geometry_msgs", "Twist"))
        move_base.publish(Message(self.cmd_vel))
        move_base.unadvertise()

    def move_from_frame_to_frame(self, from_frame, to_frame, vel=0.01, linear=True):
        vec = Vector.from_start_end(from_frame.point, to_frame.point)
        rad1 = from_frame.xaxis.angle_signed(vec, [0, 0, 1])
        rad2 = vec.angle_signed(to_frame.xaxis, [0, 0, 1])
        self.rotate_in_place(rad1, vel * (rad1 / abs(rad1)))
        self.move_forward(vel=vel, dist=vec.length)
        self.rotate_in_place(rad2, vel * (rad2 / abs(rad2)))

    def move_to_frame(self, frame, vel=0.01, orient=False):
        vec = Vector.from_start_end(self.robot_frame.point, frame.point)
        rad1 = self.robot_frame.xaxis.angle_signed(vec, [0, 0, 1])
        rad2 = vec.angle_signed(frame.xaxis, [0, 0, 1])
        if rad1 > 0:
            self.rotate_in_place(rad1, vel * (rad1 / abs(rad1)))
        self.move_forward(vel=vel, dist=vec.length)
        if orient and rad2 != 0:
            self.rotate_in_place(rad2, vel * (rad2 / abs(rad2)))

    def record_scan(self):
        pass


if __name__ == "__main__":
    mb = MobileRobotClient(host="192.168.0.4", port=9090)
    mb.connect()
    time.sleep(1)
    # mb.list_controllers()
    # print(mb.ros.get_nodes())
    # print(mb.ros.get_services())
    # mb.echo_joint_states()
    # config = Configuration()
    # config = Configuration.from_revolute_values([1.57079, 1.57079, 1.57079, 1.57079,1.57079, 1.57079])
    # print(config.joint_values)
    # mb.arm_move_joint(config)
    # config = Configuration.from_revolute_values([0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
    # mb.arm_move_joint(config)
    # mb.set_lift_height(0.0)
    # mb.rotate_in_place()
    # mb.move_backward(vel=0.05, dist=0.2)
    # time.sleep(1)
    # mb.move_radial(deg=-90, dist=0.5)
    # frame1 = Frame([0,0,0], [1,0,0], [0,1,0])
    # xvec = [math.cos(math.radians(15)),math.sin(math.radians(15)),0]
    # yvec = [-math.sin(math.radians(15)),math.cos(math.radians(15)),0]
    # frame2 = Frame([0.3,-0.1,0], xvec, yvec)
    # mb.move_from_frame_to_frame(frame1, frame2, vel=0.05)
    # mb.stop_robot()
    # mb.move_forward(vel=0.05, dist=0.2)
    time.sleep(1)
    mb.disconnect()
