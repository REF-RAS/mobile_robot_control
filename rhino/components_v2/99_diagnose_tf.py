"""Is TF data actually reaching Grasshopper?

COMPAS FAB v2.0.1

Inputs : ros_client, robot (optional), duration (float, item), run (Button)
Outputs: out, frames

There are TWO independent routes for TF, and they fail for different reasons.
This tests both so you can tell which one is broken.

ROUTE A -- raw /tf topic
    A plain roslibpy Topic subscription. Needs nothing but rosbridge. If this
    produces frame ids, TF is reaching Grasshopper and the transport is fine.

ROUTE B -- roslibpy TFClient (what MobileRobot.RCF uses)
    Calls the service /republish_tfs, type tf2_web_republisher/RepublishTFs,
    provided by the tf2_web_republisher node. That is a separate process from
    rosbridge, and it is a ROS 1 package -- the ROS 2 build is a community
    port, not part of a standard Jazzy install. If route A works and route B
    does not, the republisher is missing or its service name/type differs, and
    robot.RCF will stay None no matter how healthy TF itself is.

Message type naming differs between ROS versions ('tf2_msgs/TFMessage' vs
'tf2_msgs/msg/TFMessage'), so both are tried and the one that produces
messages is reported.

Note on frame names: TF frame ids are NOT topic names and never carry the
'/robot/' namespace. This robot's frames have a 'robot_' prefix baked into the
URDF link names instead -- robot_arm_base, robot_base_footprint. Underscore,
not slash. That is expected.

WHY THE RESULT IS CACHED
------------------------
A Button is True only during its own solve. Grasshopper then re-solves with it
False, which would overwrite the report with "Press run" before you could read
it -- the run appears to do nothing. The last result is kept in sticky and read
back on every solve, so the panel holds.
"""

import time

import roslibpy
from scriptcontext import sticky as st

REPORT_KEY = "__tf_diagnostic_report__"

frames = []
lines = []


def log(msg):
    lines.append(msg)


if run:  # noqa: F821
    if not (ros_client and ros_client.is_connected):  # noqa: F821
        log("Not connected.")
    else:
        seconds = duration or 5.0  # noqa: F821
        half = seconds / 2.0
        log("TF diagnostic at %s" % time.strftime("%H:%M:%S"))

        # ------------------------------------------------------------ services
        try:
            services = ros_client.get_services()  # noqa: F821
            tf_services = sorted(s for s in services if "tf" in s.lower())
            log("TF-related services: %s" % (", ".join(tf_services) or "NONE"))
            has_repub = any("republish_tfs" in s for s in services)
            log("  /republish_tfs present: %s%s"
                % (has_repub, "" if has_repub else "   <-- route B cannot work"))
        except Exception as e:
            log("Could not list services: %s" % e)

        try:
            topics = ros_client.get_topics()  # noqa: F821
            tf_topics = sorted(t for t in topics if "tf" in t.lower())
            log("TF-related topics  : %s" % (", ".join(tf_topics) or "NONE"))
        except Exception as e:
            log("Could not list topics: %s" % e)

        # --------------------------------------------------- route A: /tf topic
        log("\n--- Route A: raw /tf subscription (%.1fs) ---" % half)
        seen = {}

        def on_tf(message):
            for tr in message.get("transforms", []):
                parent = tr.get("header", {}).get("frame_id", "?")
                child = tr.get("child_frame_id", "?")
                key = "%s -> %s" % (parent, child)
                seen[key] = seen.get(key, 0) + 1

        # /tf carries only moving joints. Fixed joints -- flanges, sensor
        # mounts, tool plates -- are published once on /tf_static, so a link
        # missing from /tf is not missing from TF.
        worked = None
        for msg_type in ("tf2_msgs/msg/TFMessage", "tf2_msgs/TFMessage"):
            seen.clear()
            topics_open = []
            for name in ("/tf", "/tf_static"):
                t = roslibpy.Topic(ros_client, name, msg_type)  # noqa: F821
                t.subscribe(on_tf)
                topics_open.append(t)
            t0 = time.time()
            while time.time() - t0 < half:
                time.sleep(0.05)
            for t in topics_open:
                t.unsubscribe()
            if seen:
                worked = msg_type
                break

        if worked:
            log("OK  received TF using message type '%s'" % worked)
            log("    %d distinct transforms:" % len(seen))
            for name, n in sorted(seen.items(), key=lambda kv: -kv[1])[:25]:
                log("      %-52s %d msgs" % (name, n))
            frames = sorted(seen)

            # The three names the code depends on.
            names = set()
            for t in frames:
                names.update(t.split(" -> "))
            log("")
            for wanted in ("robot_arm_base", "robot_base_footprint", "robot_arm_flange"):
                log("    %-24s in TF: %s" % (wanted, wanted in names))
        else:
            log("NO messages on /tf with either message type.")
            log("    TF is not reaching Grasshopper at all -- check the robot is")
            log("    publishing and that rosbridge is not filtering the topic.")

        # ------------------------------------- route B: TFClient / republisher
        log("\n--- Route B: TFClient (what robot.RCF uses) ---")
        got = {"frame": None}

        def on_frame(transform):
            got["frame"] = transform

        try:
            tf_client = roslibpy.tf.TFClient(
                ros_client,  # noqa: F821
                fixed_frame="robot_base_footprint",
                angular_threshold=0.0,
                rate=10.0,
            )
            tf_client.subscribe("robot_arm_base", on_frame)
            t0 = time.time()
            while time.time() - t0 < half and got["frame"] is None:
                time.sleep(0.05)

            if got["frame"]:
                t = got["frame"]["translation"]
                log("OK  robot_arm_base relative to robot_base_footprint:")
                log("    x=%.4f y=%.4f z=%.4f" % (t["x"], t["y"], t["z"]))
            else:
                log("NO transform via TFClient within %.1fs." % half)
                log("    If route A worked, TF is fine and tf2_web_republisher is")
                log("    the missing piece -- robot.RCF depends on it.")
            tf_client.dispose()
        except Exception as e:
            log("TFClient failed: %s: %s" % (type(e).__name__, e))

        # --------------------------------------------------------- robot.RCF
        if robot:  # noqa: F821
            log("\n--- robot.RCF ---")
            log("mobile_client attached: %s" % bool(robot.mobile_client))  # noqa: F821
            try:
                log("robot.RCF = %s" % robot.RCF)  # noqa: F821
            except Exception as e:
                log("robot.RCF raised %s: %s" % (type(e).__name__, e))

    st[REPORT_KEY] = {"lines": lines, "frames": frames}

# Read back, so the result survives the button releasing.
cached = st.get(REPORT_KEY)
if not cached:
    print("No diagnostic run yet. Press run.")
else:
    frames = cached["frames"]
    for line in cached["lines"]:
        print(line)
