"""Running-bond layer planes for a brick wall, in build order.

Grasshopper Python 3 Script component, in the `Design from curve` group.

Inputs
------
planes : LIST access -- the half-pitch planes along the curve, from the
         `Construct Plane` (Pl) component
height : Item, float -- from the Part component's `height` output
layers : Item, int -- number of brick courses

Outputs
-------
layer_planes : one plane per brick, ordered course by course

WHAT IT REPLACED
----------------
Five native components: Sift, Move, Entwine, Clean Tree and Weave. That
cluster could only ever produce two courses -- the count was fixed by the
topology (Sift makes two streams, one Move, two Entwine branches) rather than
by any parameter, so a taller wall meant more components rather than a bigger
number.

BUILD ORDER IS THE POINT
------------------------
The list order is the order the robot lays bricks, so it is not a detail.

`Weave` was on the canvas and interleaves: low, high, low, high. Correct
geometry, unbuildable sequence -- the robot places a brick in the upper course
before the one beneath it exists.

The loop below emits each course completely before starting the next, which is
what masonry requires. Nothing downstream re-sorts, so this is the only place
that order is decided.

WHY THE PLANES ARRIVE AT HALF PITCH
-----------------------------------
`DivLength` divides the curve every (length + mortar_offset) / 2, so the
points alternate between the two course positions:

    even indices : 0, pitch, 2*pitch ...      <- courses 0, 2, 4
    odd indices  : pitch/2, 3*pitch/2 ...     <- courses 1, 3, 5

`i % 2` picks the row, giving the half-brick interlace for free. The division
by two lives on the canvas as the `A/B` block; the `2` there is structural --
half a pitch is what running bond means -- and is not the layer count.

SINGLE SOURCE FOR THE BRICK
---------------------------
`height` comes from the Part component, which publishes `height`, `length` and
`mortar_offset` as outputs. Those are read straight out of the script's own
variables, so the brick is defined once and the wall follows it.

Before that, the canvas restated derived versions of them in sliders: an `A`
slider at 0.35 (= length + mortar) and a `Factor` slider at 0.035 (= height).
Editing the brick in the Part script left both untouched and silently wrong --
bricks overlapping or floating.

ACCESS MODE
-----------
`planes` must be LIST access. On Item access the component runs once per plane
and `planes[0::2]` slices a single Plane, which fails inside a RhinoCommon
binding with an IndexError that says nothing about Grasshopper. Renaming or
creating a parameter does not give it the access mode you assume -- this is
the same trap that cost a session on `plan motion`'s `target`.
"""

import Rhino.Geometry as rg

even = planes[0::2]  # noqa: F821
odd = planes[1::2]  # noqa: F821

layer_planes = []
for i in range(int(layers)):  # noqa: F821
    row = odd if i % 2 else even
    xf = rg.Transform.Translation(0, 0, i * height)  # noqa: F821
    for p in row:
        q = rg.Plane(p)
        q.Transform(xf)
        layer_planes.append(q)

print("%d layers, %d bricks (%d even / %d odd per layer)"
      % (layers, len(layer_planes), len(even), len(odd)))  # noqa: F821
