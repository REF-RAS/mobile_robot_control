"""Pick an item from a list, clamped to the last one.

COMPAS FAB v2.0.1

Inputs : L (list access), i (int, item)
Outputs: out, a

No COMPAS FAB API here -- this is hardened against two traps.

NULL LIST
    The original did `print(len(L))` first, so before a trajectory existed --
    which is any time planning has not run, or failed -- it threw
    "object of type 'NoneType' has no len()" and went red. That reads as a
    fault here rather than a missing plan upstream.

A CONFIGURATION IS ITERABLE, AND YIELDS JOINT NAMES
    compas_robots.Configuration implements __iter__, __len__ and __getitem__,
    iterating over its JOINT NAMES:

        list(configuration) -> ['robot_lift_lower_joint',
                                'robot_arm_shoulder_pan_joint', ...]

    So calling list() on a single Configuration silently destroys it into a
    list of strings. Index into that and you get a joint name where you
    expected a configuration -- clamped to the end of a full 15-joint state,
    that surfaces as something like 'robot_front_right_wheel_joint', which
    looks like a naming bug somewhere else entirely.

    Anything carrying `joint_values` is therefore treated as one item, never
    expanded. Set `L` to List access so Grasshopper hands over the whole list.
"""


def as_list(value):
    """Coerce to a list without expanding objects that merely look iterable."""
    if value is None:
        return []
    # A Configuration iterates as joint names -- treat it as a single item.
    if hasattr(value, "joint_values") or isinstance(value, str):
        return [value]
    if hasattr(value, "__len__") or hasattr(value, "__iter__"):
        try:
            return list(value)
        except TypeError:
            return [value]
    return [value]


a = None
items = as_list(L)  # noqa: F821

if not items:
    print("No list wired. Has plan motion produced a trajectory yet?")
else:
    index = int(i) if i is not None else 0  # noqa: F821
    index = max(0, min(index, len(items) - 1))

    a = items[index]
    print("item %d of %d" % (index, len(items)))
    print("type: %s" % type(a).__name__)
    if len(items) == 1 and hasattr(a, "joint_values"):
        print("NOTE: got a single configuration, not a list of them.")
        print("      Set this component's `L` input to List access.")
