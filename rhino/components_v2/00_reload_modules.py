"""Reload the workspace libraries without restarting Rhino.

COMPAS FAB v2.0.1

Standalone -- wire nothing into it and nothing out of it.

Inputs
------
reload      : Button  (item)   press to purge
clear_cell  : Toggle (item, optional)
              also drop the cached RobotCell, forcing the loader to re-fetch
              geometry from ROS. Leave False for ordinary code edits.

Outputs
-------
out         : the report (print output)
unloaded    : list of module names that were dropped
cleared     : list of sticky keys that were dropped

How to use
----------
1. Edit any file under one of the PACKAGES below and save it.
2. Press the button.
3. Recompute the whole definition (Solution > Recompute, or F5).

Step 3 is what actually reloads: this component only clears the caches, and
the fresh import happens when the other components next run. Because it is
standalone, Grasshopper is free to run it at any point in the solve -- so do
not rely on it taking effect within the same solution. Purge, then recompute.

Why clearing sys.modules is not enough
--------------------------------------
Objects cached in scriptcontext.sticky -- the MobileRobot, the planner --
belong to the class object that was loaded when they were built. Dropping the
module leaves those instances alone, still reporting the old class and still
missing anything you just added, which looks exactly like the reload having
silently failed.

Rather than hardcode key names, this walks sticky and drops any entry whose
value comes from one of the purged packages, so it keeps working if the
loader's keys change.

The RobotCell and the MoveItPlanner are kept: both are compas_fab objects, so
reloading these packages cannot make them stale. Keeping the cell avoids
another full mesh download; keeping the planner avoids resetting MoveIt's
planning scene. Use `clear_cell` when you do want the geometry re-fetched.

Cost of a reload depends on your loader
---------------------------------------
This drops the cached MobileRobot, so the loader has to rebuild it.

- gh_load_mobile_robot.py as it currently stands builds the MobileRobot only
  inside `if ... and load:`, so after a purge you must ALSO flip `load` --
  which re-downloads all 30 meshes (~16s).

- The three-key variant of that loader caches the cell separately from the
  wrapper, and rebuilds the wrapper from the cached cell with no network
  traffic. With that one, press the button and recompute; `load` stays off.
"""

import sys
import time

from compas_rhino import unload_modules
from scriptcontext import sticky as st

# The last report is kept so the outputs survive the recompute that follows the
# button press. A Button is only True during its own solve, so without this the
# panels blank out at exactly the moment you look at them.
# A plain dict under a key without 'cell' in it, so neither the instance sweep
# nor clear_cell removes it.
REPORT_KEY = "__module_reload_report__"

PACKAGES = [
    "mobile_robot_control",
    "fabrication_manager",
    "ur_fabrication_control",
    "assembly_information_model",
    "qut_assembly",
    "base_positioning",
    "marker_tracking",
    "qut_ur_fabrication_control",
]

unloaded = []
cleared = []


def _from_purged_package(value):
    """True if `value`'s class is defined in one of PACKAGES."""
    module = getattr(type(value), "__module__", "") or ""
    return any(module == p or module.startswith(p + ".") for p in PACKAGES)


if reload:  # noqa: F821
    # 1. Drop the cached instances first, while their classes still resolve.
    for key, value in list(st.items()):
        try:
            if _from_purged_package(value):
                st.pop(key, None)
                cleared.append("%s  (%s)" % (key, type(value).__name__))
        except Exception:
            # A sticky value that objects to being inspected is not worth
            # taking the whole reload down for.
            continue

    if clear_cell:  # noqa: F821
        for key in list(st.keys()):
            if "cell" in str(key).lower():
                st.pop(key, None)
                cleared.append("%s  (cached cell)" % key)

    # 2. Then purge the modules themselves. unload_modules matches on prefix,
    #    so the top-level name covers every submodule.
    for package in PACKAGES:
        unloaded.extend(unload_modules(package))

    # 3. Anything left behind means the purge did not fully take.
    still = sorted(n for n in sys.modules if any(n.startswith(p) for p in PACKAGES))
    kept = [str(k) for k in st.keys() if "cell" in str(k).lower()]

    st[REPORT_KEY] = {
        "when": time.strftime("%H:%M:%S"),
        "unloaded": sorted(unloaded),
        "cleared": cleared,
        "still": still,
        "kept": kept,
    }

# Read the report back rather than using the local lists, so the outputs hold
# after the button releases.
report = st.get(REPORT_KEY)

if not report:
    print("No reload run yet. Press the button, then recompute (F5).")
else:
    unloaded = report["unloaded"]
    cleared = report["cleared"]

    print("Last reload at %s" % report["when"])
    print("  %d modules unloaded across %d packages" % (len(unloaded), len(PACKAGES)))
    print("  %d sticky entries cleared" % len(cleared))

    if report["kept"]:
        print("  kept (set clear_cell to drop): %s" % ", ".join(report["kept"]))

    if report["still"]:
        print("\nWARNING these did not unload: %s" % ", ".join(report["still"]))
    else:
        print("\nAll target packages cleared. Recompute (F5) to re-import.")
