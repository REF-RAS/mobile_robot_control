"""Diagnose HTTP 404s from the ROS 2 mesh file server.

Grasshopper component script.
Inputs: ros_client (item), prefix (str, item), file_server (str, item).
Outputs: out, urls, report.

`prefix` is the ROS namespace, e.g. '/robot' -> /robot/robot_description. It must
match the namespace the robot actually publishes under; omitting it asks for
/robot_description, which nothing publishes, and you get a topic timeout rather
than anything to do with the file server.

Loads the robot cell WITHOUT geometry (URDF/SRDF come from latched topics, so
this always works), then reproduces exactly what HttpFileServerLoader.load_meshes
would do -- urljoin(base_url, filename_without_package_prefix) -- and reports the
HTTP status per unique mesh path.

The point is to see the URL that 404s, which the compas_fab traceback does not
print, and compare it against what the file server actually exposes.
"""

import itertools
from urllib.error import HTTPError, URLError
from urllib.parse import urljoin
from urllib.request import urlopen

report = []
urls = []


def log(msg):
    report.append(msg)
    print(msg)


if not (ros_client and ros_client.is_connected):  # noqa: F821
    log("Not connected.")
else:
    try:
        distro = ros_client.ros_distro  # noqa: F821
        log("ros_distro : {}  (is_ros2={})".format(distro.value, distro.is_ros2))
    except Exception as e:
        log("ros_distro : FAILED -> {}".format(e))

    # Same default the client uses when http_file_server_base_url is None.
    base = file_server or "http://{}:9190".format(ros_client.host)  # noqa: F821
    base = base.rstrip("/") + "/"
    log("base_url   : {}".format(base))

    # No geometry: isolates topic loading from mesh fetching.
    ns = prefix or ""  # noqa: F821
    log("urdf topic : {}/robot_description".format(ns))
    cell = ros_client.load_robot_cell(  # noqa: F821
        load_geometry=False,
        urdf_param_name="{}/robot_description".format(ns),
        srdf_param_name="{}/robot_description_semantic".format(ns),
    )
    log("robot      : {}".format(cell.robot_model.name))

    # Collect every mesh reference in the model, in load_geometry's own order.
    filenames = []
    for link in cell.robot_model.links:
        for element in itertools.chain(link.collision, link.visual):
            shape = element.geometry.shape
            if "filename" in dir(shape) and shape.filename not in filenames:
                filenames.append(shape.filename)

    log("meshes     : {} unique references".format(len(filenames)))
    log("")

    # Group by ROS package so a wrong server root is obvious at a glance.
    packages = {}
    for fn in filenames:
        if fn.startswith("package://"):
            packages.setdefault(fn[len("package://"):].split("/")[0], 0)
            packages[fn[len("package://"):].split("/")[0]] += 1
    log("packages referenced: {}".format(dict(packages)))
    log("")

    # Probe each unique mesh; stop detailing after the first few of each status.
    counts = {}
    for fn in filenames:
        if not fn.startswith("package://"):
            log("SKIP (not package://): {}".format(fn))
            continue

        url = urljoin(base, fn[len("package://"):])
        urls.append(url)

        try:
            with urlopen(url, timeout=10) as r:
                status = "{} OK ({} bytes)".format(r.status, len(r.read()))
        except HTTPError as e:
            status = "HTTP {} {}".format(e.code, e.reason)
        except URLError as e:
            status = "URLError {}".format(e.reason)
        except Exception as e:
            status = "{}: {}".format(type(e).__name__, e)

        key = status.split("(")[0].strip()
        counts[key] = counts.get(key, 0) + 1
        if counts[key] <= 3:
            log("{:<28} {}".format(key, url))

    log("")
    log("summary: {}".format(counts))
