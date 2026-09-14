"""Pick an item from a list, clamped to the last one.

COMPAS FAB v2.0.1

Inputs : L (list), i (int, item)
Outputs: out, a

No COMPAS FAB API here -- this is only hardened. The original was:

    print(len(L))
    if i < len(L):
        a = L[i]
    else:
        a = L[-1]

`len(None)` raises TypeError, so whenever the upstream trajectory was empty --
which it is any time planning has not run yet, or failed -- this threw
"object of type 'NoneType' has no len()" and went red. That reads as a fault in
this component rather than a missing trajectory further up, and it fires on
every solve until planning succeeds.
"""

a = None

if not L:  # noqa: F821
    print("No list wired. Has plan motion produced a trajectory yet?")
else:
    items = list(L)  # noqa: F821
    index = int(i) if i is not None else 0  # noqa: F821

    if index < 0:
        index = 0
    if index >= len(items):
        index = len(items) - 1

    a = items[index]
    print("item %d of %d" % (index, len(items)))
