# Local Path Planner Requirements
The planner consumes the mapper's local traversability grid, the fused UGV
pose, and the requested goal. It produces a safe route in the fixed ENU
`map` frame.

The planner owns:

- Grid-to-world and world-to-grid conversion.
- Path-search and route scoring.
- UGV footprint and safety-clearance handling.
- Start and goal validity checks.
- Replanning triggers.
- Route validation and publication.

The planner does not own camera processing, grid construction, localization,
or simulator truth. The mapping node is responsible for constructing and
publishing the traversability grid from the image stream and any future
supporting sensors.

## 2. Planner inputs

### Proposed primary inputs

| Topic | Type | Use |
| --- | --- | --- |
| `/traversability_grid` | `uav_ugv_msgs/TraversabilityGrid` | Local traversability costs |
| `/ugv/localization/pose` | `geometry_msgs/PoseWithCovarianceStamped` | Current UGV pose in `map` |
| `/global_goal_pose` | `geometry_msgs/PoseStamped` | Requested goal in `map` |

The traversability-grid message is proposed and is not yet implemented in the
current repository.

### Current compatibility input

The current mapper publishes:

```text
/local_grid
  nav_msgs/OccupancyGrid
```

The current planner subscribes to this occupancy grid. It is a compatibility
baseline while the traversability-grid interface is implemented.

## 3. Grid contract

The proposed grid is a temporary local window expressed in the fixed ENU
`map` frame. It is not UGV-relative and is not persistent.

Initial geometry:

```text
resolution:       0.25 m per cell
maximum cells:    100,000
ground model:     flat plane at ground_z
maximum map age:  2.0 s
```

The mapper may change the grid bounds, width, and height for each image.
The planner must read these values from the message rather than assume fixed
dimensions.

For cell `(i, j)`:

```text
x_center = origin_x + (i + 0.5) * resolution
y_center = origin_y + (j + 0.5) * resolution
index = j * width + i
```

For a `map` position `(x, y)`:

```text
i = floor((x - origin_x) / resolution)
j = floor((y - origin_y) / resolution)
```

The grid frame must be `map`. The grid timestamp is the source image
acquisition time.

## 4. Cell interpretation

The proposed baseline contains one value per cell:

```text
traversability_cost
```

```text
0.0       preferred
finite >0 less preferred but potentially traversable
+inf      forbidden
NaN       unknown
```

The planner must:

- Reject `NaN` and `+inf` cells by default.
- Never convert unknown cells into preferred cells.
- Use finite costs in its route objective if desired.
- Keep planner-specific costs, such as distance or turning effort, separate
  from the mapper's terrain cost.

Probability, confidence, elevation, slope, roughness, and surface classes are
future extensions. The initial planner must not depend on them.

## 5.1 Future-compatible design

The initial planner should keep its interfaces and cost handling extensible
because the mapper may later add:

- Depth or elevation information for slope, roughness, step-height, and
  obstacle-clearance decisions.
- Per-cell traversability probability or confidence to distinguish reliable
  free space from uncertain observations.

Do not hard-code the planner around binary luminance-derived costs. Future
layers should be usable as additional route costs or validity checks without
changing the planner's grid geometry, frame, timestamp, or footprint-clearance
responsibilities.

## 5. Planning and safety requirements

The planner must reject a planning snapshot when:

- The grid is missing, stale, malformed, or not in `map`.
- Grid dimensions and array lengths do not agree.
- The UGV pose is stale or exceeds configured uncertainty limits.
- The goal is invalid or outside the grid.
- The UGV start cell is outside the grid or forbidden.
- The goal cell is outside the grid or forbidden.

Initial UGV safety assumptions:

```text
UGV footprint radius: 0.50 m
safety clearance:     0.05 m
goal tolerance:       0.50 m
```

These values must be configurable. The planner must account for the footprint
and clearance, not only the center point. It must validate the complete route,
including:

- Start connection
- Goal connection
- Every path segment
- Diagonal corner-cutting
- Grid boundaries

Unknown cells and map boundaries are blocked by default.

## 6. Replanning

Replan when any of the following occurs:

- A new valid traversability grid arrives.
- The UGV moves significantly.
- The goal revision changes.
- The previous route becomes invalid or stale.

The planner should publish no motion-authorizing route when the required
inputs are invalid or no safe route exists.

## 7. Route output

Publish:

```text
/ugv/navigation/route
  uav_ugv_msgs/PlannedRoute
```

The route must:

- Use the `map` frame.
- Preserve the exact goal message and goal revision used for planning.
- Begin at the current UGV pose or a validated nearby start.
- End at the requested goal; do not silently substitute another goal.
- Contain finite poses and connected, footprint-safe segments.
- Include the derived inflated `clearance_grid` required by the current
  controller contract.

When no safe route exists, publish an empty path and provide diagnostics.

`/local_path` is optional visualization/evaluation output. The controller
should use the validated `/ugv/navigation/route` contract.

## 8. Recommended starting checklist

1. Subscribe to the three proposed planner inputs.
2. Validate grid geometry, timestamps, frame, and array length.
3. Convert the UGV and goal positions to cells.
4. Apply footprint and clearance handling.
5. Select and implement the path-search algorithm.
6. Validate every route segment.
7. Publish `PlannedRoute` for valid routes and an empty route on failure.
8. Add tests for outside-grid, unknown, forbidden, stale, blocked-start,
   blocked-goal, diagonal, and no-route cases.
