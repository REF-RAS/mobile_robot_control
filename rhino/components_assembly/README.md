# Fabrication assembly design — the brick wall cluster

The part of the Grasshopper definition that decides **what gets built**. It
produces the target frames that `SYSTEM_WORKFLOW.md` calls Stage 5, and feeds
them to the robot chain through `create_parts` → `Assembly`.

It touches no ROS and needs no robot.

## The chain

```
input_curve ─┐
             ├─► DivLength ─► Construct Plane ─► wall_layers ─► create_parts ─► Assembly
pitch / 2 ───┘                                        ▲
                                                      │
Part ── height, length, mortar_offset ────────────────┘
```

| Step | Does |
|---|---|
| `Part` | defines one brick; publishes `height`, `length`, `mortar_offset` |
| `A+B` | `pitch = length + mortar_offset` |
| `A/B` (B=2) | half pitch — the running-bond stagger |
| `DivLength` | a point every half pitch along the curve |
| `Construct Plane` | a plane at each point, oriented off the curve tangent |
| `wall_layers` | courses: picks alternating rows, lifts each by `i × height` |
| `create_parts` | transforms the brick onto each plane |
| `Assembly` | collects the parts |

## The brick is defined once

`Part` publishes its dimensions as outputs. A Rhino 8 Python 3 Script component
reads outputs straight from the script's global variables, so adding an output
named `length` publishes the existing `length = 0.30` with no code change.

Everything downstream derives from those outputs. Nothing restates them.

This replaced two sliders — `A` at 0.35 and `Factor` at 0.035 — that were
hand-computed from the brick and linked to it by nothing at all. Editing the
brick in the Part script left both silently wrong: bricks overlapping, or an
upper course driven into the one below. Same class of fault as the base pose in
`SYSTEM_WORKFLOW.md`: one fact, two sources, no link between them.

## Build order is decided in exactly one place

The output list order **is** the order the robot lays bricks. `wall_layers`
emits each course complete before starting the next, and nothing downstream
re-sorts.

The superseded cluster had a `Weave` wired up, which interleaves low/high/low/
high. Correct geometry, unbuildable sequence — the robot would place an upper
brick before the one beneath it existed. `Entwine` + Flatten was the correct
block of the two, and both sat on the canvas unwired, which is what an
abandoned experiment looks like.

## Gotcha: Sift Pattern pads with nulls

Recorded because it cost real time, and it will recur if anyone reaches for
`Sift` again.

`Sift Pattern` preserves item *positions* across its outputs rather than
compacting them:

```
output 0 :  p0, null, p2, null, p4, null
output 1 :  null, p1, null, p3, null, p5
```

`Move` on a null gives a null, `Entwine` carries them through, and
`create_parts` dies on line 5 with `'NoneType' object has no attribute
'Origin'` — an error naming neither Sift nor nulls.

**Flatten does not fix it.** Flatten removes branch structure; nulls are items,
not structure. `Clean Tree` with Remove Nulls is the fix.

`wall_layers` sidesteps this entirely — it slices the list in Python and emits
no nulls.

## Known gaps

- **Open ends are ragged.** Odd courses start half a brick in, so they overhang
  at one end and fall short at the other. Real masonry uses half bricks there.
  Either trim odd courses to the curve, add half-length parts at the ends, or
  close the curve into a loop so the problem does not arise.
- **No bed joint.** Courses stack at `i × height`, bricks in direct contact.
  `mortar_offset` is applied horizontally only. If a vertical joint is wanted,
  it is `i * (height + bed_joint)` — worth choosing deliberately rather than
  inheriting zero.
- **No reachability filter.** Every brick is emitted regardless of whether the
  robot can reach it from where it stands. See `SYSTEM_WORKFLOW.md` §5.2 — the
  wall is metres long and the UR20 reaches 1.75 m, so most of these targets are
  unreachable from any single base position.
- **No stability check.** Nothing verifies a brick has support beneath it.
  Correct build order gives this for a straight running bond, but not for
  openings, returns or corbels.
