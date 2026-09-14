"""Are joint states actually reaching Grasshopper?

COMPAS FAB v2.0.1

Inputs : ros_client, robot (optional), duration (float, item), run (Button)
Outputs: out, joint_names

An all-zeros arm pose in Grasshopper while the real robot and RViz show
something else almost always means no JointState messages are arriving. The
failure is silent by construction: current_joint_values stays empty and
get_current_configuration substitutes 0.0 for every missing joint, which is
indistinguishable from a robot parked at zero.

Two things can cause it, and they need different fixes:

MESSAGE TYPE
    ROS 2 rosbridge expects 'sensor_msgs/msg/JointState'. The ROS 1 name
    'sensor_msgs/JointState' does not raise -- you just never get a message.

TOPIC NAME
    The client assumes /robot/arm/joint_states and /robot/lift/joint_states.
    A ROS 2 bringup may publish a single combined /robot/joint_states instead.

This lists every joint-state topic the robot advertises, subscribes to each
under both type names, and reports which combination delivers.
"""

import time

import roslibpy
from scriptcontext import sticky as st

REPORT_KEY = "__joint_state_diagnostic__"
TYPES = ["sensor_msgs/msg/JointState", "sensor_msgs/JointState"]

joint_names = []
lines = []


def log(msg):
    lines.append(msg)


if run:  # noqa: F821
    if not (ros_client and ros_client.is_connected):  # noqa: F821
        log("Not connected.")
    else:
        seconds = duration or 4.0  # noqa: F821
        log("Joint state diagnostic at %s" % time.strftime("%H:%M:%S"))

        try:
            distro = ros_client.ros_distro  # noqa: F821
            log("ros_distro: %s (is_ros2=%s) -> expects '%s'"
                % (distro.value, distro.is_ros2,
                   TYPES[0] if distro.is_ros2 else TYPES[1]))
        except Exception as e:
            log("ros_distro unavailable: %s" % e)

        # What does the robot actually advertise?
        try:
            topics = ros_client.get_topics()  # noqa: F821
            candidates = sorted(t for t in topics if "joint_state" in t.lower())
            log("\njoint-state topics advertised:")
            for t in candidates:
                log("    %s" % t)
            if not candidates:
                log("    NONE -- nothing is publishing joint states at all.")
        except Exception as e:
            log("Could not list topics: %s" % e)
            candidates = []

        # Try each topic under both type names.
        log("\nsubscribing to each for %.1fs:" % seconds)
        per_topic = max(0.7, seconds / max(1, len(candidates) * len(TYPES)))
        results = {}

        for topic_name in candidates:
            for msg_type in TYPES:
                received = {"names": None, "count": 0}

                def on_msg(message, _r=received):
                    _r["count"] += 1
                    _r["names"] = message.get("name")

                try:
                    topic = roslibpy.Topic(ros_client, topic_name, msg_type)  # noqa: F821
                    topic.subscribe(on_msg)
                    t0 = time.time()
                    while time.time() - t0 < per_topic and received["count"] == 0:
                        time.sleep(0.05)
                    topic.unsubscribe()
                except Exception as e:
                    log("    %-34s %-30s ERROR %s" % (topic_name, msg_type, e))
                    continue

                mark = "OK  " if received["count"] else "    "
                log("  %s%-34s %-30s %d msgs" % (mark, topic_name, msg_type, received["count"]))
                if received["count"] and received["names"]:
                    results[topic_name] = (msg_type, received["names"])
                    break  # this topic works, no need for the other type

        if results:
            log("\nWORKING COMBINATIONS:")
            all_names = []
            for topic_name, (msg_type, names) in results.items():
                log("    %s  as  %s" % (topic_name, msg_type))
                log("        %d joints: %s" % (len(names), ", ".join(names)))
                all_names.extend(names)
            joint_names = sorted(set(all_names))

            # Ask the client what it will look for, rather than repeating a
            # list here that can drift out of date the way the old lift joint
            # name did.
            from mobile_robot_control.mobile_robot_client import MobileRobotClient
            wanted = (list(MobileRobotClient.LIFT_JOINT_NAMES)
                      + list(MobileRobotClient.ARM_JOINT_NAMES))
            log("\njoints get_current_configuration() asks for:")
            for name in wanted:
                log("    %-34s %s" % (name, "FOUND" if name in joint_names else "MISSING"))
            missing = [n for n in wanted if n not in joint_names]
            if missing:
                log("\n  Missing joints read back as 0.0, which is why the arm")
                log("  sits at its zero pose. Check the names against the list above.")
        else:
            log("\nNO topic delivered under either type name.")
            log("  Nothing is publishing, or rosbridge is filtering these topics.")

        # What the live client currently holds.
        if robot and robot.mobile_client:  # noqa: F821
            mc = robot.mobile_client  # noqa: F821
            log("\nmobile_client currently holds %d joint values" % len(mc.current_joint_values))
            for k, v in sorted(mc.current_joint_values.items()):
                log("    %-34s %.4f" % (k, v))

    st[REPORT_KEY] = {"lines": lines, "joint_names": joint_names}

cached = st.get(REPORT_KEY)
if not cached:
    print("No diagnostic run yet. Press run.")
else:
    joint_names = cached["joint_names"]
    for line in cached["lines"]:
        print(line)
