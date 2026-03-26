# Relative Map

## Introduction
Relative map module is a middle layer connecting HDMap/Perception module and planning module. This module generates real-time relative map in the body coordinate system (FLU) and a reference line for planning. The inputs for relative map module have two parts: offline and online. The offline part is navigation line (human driving path) and the HDMap information on and near navigation line. And the online part is the traffic sign related information from perception module, e.g., lane marker, crosswalk, traffic light etc. The generation of relative map can leverage both online and offline parts. It also works with either online or offline part only.

## Inputs
  * NavigationInfo from dreamview module
  * LaneMarker from perception module
  * Localization from localization module

## Output
  * Relative map follows map format defined in modules/map/proto/map.proto

## Workflow

### 1. Module Start (`RelativeMapComponent::Init`)

The module is launched as a `TimerComponent` that fires every **100 ms** (configured in `dag/relative_map.dag`). On initialization:

1. A `VehicleStateProvider` is created to track the vehicle's position and velocity.
2. Four topic readers are created:
   - `/apollo/perception_obstacles` → `OnPerception` callback (lane marker data)
   - `/apollo/chassis` → `OnChassis` callback (vehicle speed / gear)
   - `/apollo/localization/pose` → `OnLocalization` callback (position & heading)
   - `/apollo/navigation` → `OnNavigationInfo` callback (offline navigation lines from DreamView)
3. A writer for `/apollo/relative_map` is created to publish the output `MapMsg`.
4. `RelativeMap::Init` loads configuration (`relative_map_config.pb.txt`) and configures `NavigationLane` with default lane widths and the chosen `lane_source` (typically `OFFLINE_GENERATED`).
5. `RelativeMap::Start` logs that the module has started.

### 2. Periodic Processing (`RelativeMapComponent::Proc`)

Every 100 ms the timer fires and calls `Proc`, which:

1. Creates a new `MapMsg`.
2. Calls `RelativeMap::Process(map_msg)`, which under a mutex:
   - Updates the vehicle state from the latest localization and chassis messages.
   - Copies the latest perception lane markers into `map_msg`.
   - Calls `NavigationLane::GeneratePath()` to build the current path.
   - Calls `NavigationLane::CreateMap()` to convert the path into an HD-map message.
3. Fills the message header (timestamp, module name).
4. Publishes `map_msg` on the `/apollo/relative_map` topic.

### 3. Path Generation (`NavigationLane::GeneratePath`)

Path generation follows a **priority cascade**:

#### a) Offline-navigation mode (default, `lane_source = OFFLINE_GENERATED`)

When at least one navigation line is present in `NavigationInfo`:

1. **Convert each navigation line to a local-coordinate path segment.**  
   `ConvertNavigationLineToPath` projects the vehicle onto the navigation line stored in ENU (world) coordinates, selects the ~250 m segment ahead of the vehicle, and converts it to FLU (body) coordinates with the vehicle as the origin.  
   - If the distance remaining on the current line is less than 20 m and cyclic rerouting is disabled, the line is skipped.
   - If `FLAGS_enable_cyclic_rerouting` is set and the vehicle approaches the end of a cyclic route (a route whose end point is close to its start point, i.e., a loop), the path is stitched from the end back to the beginning automatically.

2. **Sort navigation paths left to right** (by the y-coordinate of their first point in FLU space).

3. **Identify which path the vehicle is on** by finding the path with the smallest lateral distance (`min_d`) to the vehicle.

4. **Merge the vehicle's current path with perceived lane markers.**  
   `MergeNavigationLineAndLaneMarker` computes a weighted average of each navigation path point and the corresponding lane-marker centerline point. The blend is controlled by `lane_marker_weight` (default **0.1**), meaning each merged point is **90 % navigation line + 10 % lane marker**. This fuses the stable offline path with real-time lane perception.

5. **Set lane widths.**  
   - For the vehicle's current lane, perceived widths from the lane markers are used (if valid).
   - For adjacent lanes, the half-width is computed as half the average lateral distance between neighbouring path centrelines.

If no navigation line produces a valid path, the code falls back to perception-only mode (see below).

#### b) Perception-only mode (fallback)

When no offline navigation lines are available or all are invalid:

`ConvertLaneMarkerToPath` builds a path purely from the perceived left/right lane markers:

1. The lane centerline is approximated by averaging the cubic-polynomial coefficients of the left and right lane markers: `y = c0 + c1·x + c2·x² + c3·x³`.
2. Path length is set proportional to vehicle speed (capped between `min_len_for_navigation_lane` = 150 m and `max_len_for_navigation_lane` = 250 m).
3. Points are sampled every 1 m from `x = -2 m` (slightly behind the vehicle) to the computed path length, with heading (`theta`), curvature (`kappa`), and curvature rate (`dkappa`) computed analytically from the polynomial.
4. Perceived lane width is read from the `c0` coefficients of the left/right markers; if the resulting width is outside `[2·min_lane_half_width, 2·max_lane_half_width]`, the default width (1.875 m per side) is used.

### 4. Map Creation (`NavigationLane::CreateMap`)

Once paths are generated, `CreateMap` converts them into the `MapMsg` HD-map format:

1. For each navigation path in `navigation_path_list_` (or the single perception-based path), `CreateSingleLaneMap` creates one `hdmap::Lane` entry with:
   - A centre-line curve from the path points.
   - Left and right boundary curves offset laterally by the computed lane half-widths.
   - Lane type (`CITY_DRIVING`), turn type (`NO_TURN`), and speed limit.
2. For multi-lane scenarios, the left boundary of each lane reuses the right boundary of its left neighbour to avoid gaps.
3. A `Road` object is added grouping all lanes into a single road section with outer polygon boundaries.
4. Forward left/right neighbour lane IDs are set for each lane.

### 5. Module Stop

When the Cyber RT framework shuts down the component, `RelativeMap::Stop` is called, which logs that the module has stopped. No special cleanup is required because data is owned by the stack frame of each `Proc` call.
