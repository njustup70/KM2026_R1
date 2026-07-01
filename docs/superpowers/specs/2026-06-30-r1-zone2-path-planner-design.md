# R1 Zone2 Path Planner Design

## Goal

R1 needs a Zone2 planner for picking three R1 blocks around Merlin. The planner must keep R1 from blocking R2 as the highest priority, then choose the fastest route among valid routes, and finally end at X[11] because Zone2 to Zone3 must leave through X[11].

R1 may enter Zone2 from any X[0..17] route point. The planner should choose the entry point that gives the best route for the actual targets.

## Current Code

The current planner is `GetShortestPath(uint8_t KFS_values[12], PathContainer& path)` in `ReactorLibs/Users/PathPlaner`.

It currently:

- Treats X[0..17] as a rectangular route loop.
- Maps Merlin block IDs to one or two reachable X positions.
- Uses a greedy nearest-block choice.
- Outputs `points`, `labels`, and `have_block_xids`.

The current `Logic.cpp` and `ManuGragh.cpp` state machines consume `Zone2_Path` point by point and decide whether to pick by checking `have_block_xids`.

## Inputs

The new planner will use:

- `KFS_values[12]`: R1 block positions, where `1` means this Merlin block should be picked.
- `r2_column`: host PC value `1`, `2`, or `3`.

R2 columns:

- Column 1: Merlin `{0, 3, 6, 9}`
- Column 2: Merlin `{1, 4, 7, 10}`
- Column 3: Merlin `{2, 5, 8, 11}`

New API:

```cpp
void GetShortestPath(uint8_t KFS_values[12], uint8_t r2_column, PathContainer& path);
```

Keep a compatibility overload if useful:

```cpp
void GetShortestPath(uint8_t KFS_values[12], PathContainer& path);
```

The compatibility overload should call the new function with a neutral column value so old call sites can still compile while integration is being updated.

## Route Rules

R1 can drive on X[0..17].

R1 can pick blocks at every X point except:

- X[2]
- X[7]
- X[11]
- X[16]

R1 can rotate only at:

- X[0]
- X[1]
- X[2]
- X[7]
- X[11]
- X[16]
- X[17]

For each picked block, the planner records:

- Merlin block ID.
- X point where R1 picks it.
- X point where R1 must rotate before picking.
- Yaw R1 should hold while picking.

If the pick point is also a legal rotate point, rotate at the pick point. Otherwise rotate at the latest legal rotate point on the chosen route before the pick point.

Example: when moving from X[13] to pick at X[9], the route passes X[11], so R1 rotates at X[11] before going to X[9].

## Block Candidates

R1 target blocks are limited to Merlin:

`{0, 1, 2, 5, 8, 11, 10, 9, 6, 3}`

The planner should ignore any `KFS_values[i]` outside the valid R1 set.

Blocks with two possible pick positions must evaluate both options and let route cost decide. The known dual-candidate Merlin blocks are:

- Merlin 0
- Merlin 2
- Merlin 9
- Merlin 11

The existing mapping arrays can be kept, but the planner should treat the mapping as explicit candidate lists rather than choosing the nearest candidate greedily.

## Priority

R2 priority is stronger than speed.

The planner splits targets into:

- Priority group: R1 targets in `r2_column`.
- Normal group: all remaining R1 targets.

Every priority-group target must be picked before any normal-group target. Within each group, choose the fastest order.

If no R1 target is in the R2 column, optimize all three targets by speed.

## Search

Only three R1 blocks are picked, so exhaustive enumeration is small and safer than greedy planning.

The planner should enumerate:

- Possible Zone2 entry X point from X[0..17].
- All valid block orders that respect priority-before-normal.
- All candidate pick X points for each block.
- Clockwise and counterclockwise travel choices between route points.

Candidate cost should include:

- Movement distance in route steps.
- A small extra cost for required rotations, so equal-length routes prefer cleaner rotation behavior.
- End distance from the last pick point to X[11].

The selected route is the valid route with the lowest cost after enforcing R2 priority.

## PathContainer

`PathContainer` should keep the current fields for compatibility:

- `points`
- `labels`
- `have_block_xids`
- `have_block_count`

Add pick metadata:

```cpp
int pick_block_ids[12];
int pick_xids[12];
int rotate_xids[12];
float pick_yaws[12];
uint8_t pick_count;
```

`have_block_xids` should remain a compatibility view of `pick_xids`.

The path point list should include route points needed for movement and rotation. Required rotate points should be present in `labels`, but should not be duplicated if they are already on the route.

## Runtime Use

`Logic.cpp` and `ManuGragh.cpp` should use the new pick metadata instead of relying only on `have_block_xids`.

At each path point:

- If the current label is the next pick's rotate X point, rotate to the stored pick yaw.
- If the current label is the next pick X point, trigger the pick state.
- Otherwise continue navigation.

The pick state should use the stored pick X point for height selection and inward movement direction, preserving the current mechanism while making rotate-before-pick explicit.

## Host PC Integration

The host PC sends `r2_column` as `1`, `2`, or `3`.

Store the received value in an application-level place such as `CommCenter` or `TaskLogic`, then call:

```cpp
GetShortestPath(farcon.KFS_values, r2_column, Zone2_Path);
```

If `r2_column` is invalid, planning should fall back to speed-only ordering and still generate a path.

## Verification

Add lightweight verification for:

- Dual-candidate Merlin blocks 0, 2, 9, and 11.
- R2 priority for columns 1, 2, and 3.
- Final path point is always X[11].
- Pick X points never use X[2], X[7], X[11], or X[16].
- Rotate X points are always in `{0, 1, 2, 7, 11, 16, 17}`.
- The example route from X[13] to pick at X[9] rotates at X[11].

If embedded unit tests are awkward, keep this as a host-side C++ self-check or a compile-time guarded planner self-check.
