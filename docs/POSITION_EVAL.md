# Position evaluation

No independent tram route/pathgraph was found in the recursive home-directory search, including the project and its parent. `results/position_eval/route_search.json` records matching filenames, all five ZIP inventories, and topic schemas from all 122 bags. Archives contain telemetry, messages and documentation; all bags contain only the three vehicle and four GNSS topics. The only route files are copies of `example_route.json`, a SYNTHETIC L-shaped example. No GNSS trajectory was converted into a runtime route.

## Computation and frame

The unchanged velocity estimator integrates longitudinal distance. Position uses WGS84 ECEF-to-ENU coordinates, with origin at the first valid master GNSS fix unless configured explicitly. GNSS fixes, velocity or dual-receiver heading may initialize position/heading during the first three seconds of input Header time. Afterwards GNSS cannot update position. Without a route, initial heading and height are held. Turns are therefore unobservable. `odom` names this local ENU frame; `base_link` is the child frame. An antenna-to-vehicle lever arm is not calibrated.

With an independent route, accumulated distance selects an ordered 3D polyline station, with heading from the segment tangent. Endpoints clamp and report `route_exhausted`; branch selection is not implemented. Two route-start bugs were fixed: configured pose projection now works with GNSS disabled, and explicit start station no longer shifts when initial GNSS updates the free-space anchor. Auto-GNSS projection retains its own distance anchor. Synthetic regression tests verify these mechanics, not real-world route accuracy.

## Required external geometry

The jury must supply an independently surveyed/otherwise authorized ordered route, covering the run, with provenance independent of held-out GNSS, metric ENU coordinates, and a declared WGS84 origin:

```json
{"origin_lla":[55.0,37.0,150.0],"points_enu":[[0,0,0],[100,0,0],[100,100,0]]}
```

These numbers are SYNTHETIC format examples. Points are east/north/up metres; origin is latitude/longitude degrees and altitude metres. Duplicate consecutive points are invalid. For branches, supply the selected traversal as an ordered polyline; this implementation cannot choose switches. The initial pose and points must share this origin. `route_start_s` is station in metres at episode start; -1 requests automatic projection. `route_direction` is +1 or -1 along the ordering. Direction along a route does not add reverse-velocity estimation.

```bash
ros2 launch tram_odometry odometry.launch.py use_sim_time:=true route_file:=/absolute/independent_route.json route_start_s:=0.0 route_direction:=1
```

## Evidence and definitions

One full held-out bag, `30618_d4ba7005`, is checked against the frozen split SHA-256. No model retuning or velocity-pipeline edits were made.

A. **Relative distance:** existing evaluation integrates GNSS horizontal speed and compares estimated distance increments over matched contiguous intervals (<0.25 s gaps). Error -7.587 m, -0.142410%. This is not geometric along-track error.

B. **Geometric 2D position:** diagnostic comparison with valid master GNSS fixes, transformed into the same ENU frame and matched to the nearest output within 0.05 s, after initialization. Initial alignment error is the first post-initialization matched error, not an independent initial-pose validation. RMSE is sqrt(mean squared horizontal Euclidean error); MAE is mean horizontal Euclidean error; MAX is the largest such error. Final means the last valid matched fix, not necessarily the last bag message.

- Initial post-window error: 0.002439 m.
- Final horizontal error: 2784.788 m.
- Horizontal RMSE / MAE / MAX: 1804.198 / 1495.632 / 2862.791 m.
- Final error/reference travelled distance: 52.2700%, denominator 5327.697 m, 99.7322% time coverage. GNSS velocity gaps are excluded from this denominator.
- Along-track / cross-track metrics: NOT VERIFIED; no independent route exists.
- Geometric route capability: NOT VERIFIED. The poor no-map Euclidean comparison is reported explicitly; it is not route validation.

Headers and bag-recording time have an offset. The first post-initialization scored fix is at +0.502 s relative to recording start; the initialization window itself is three seconds in the input Header domain.

After initialization the estimated heading changes by 0 degrees. Moving reference heading differs by median 37.839 degrees and maximum 68.473 degrees, versus median 0.462 degrees early in the run. Combined with small relative distance drift and small initial alignment error, this supports missing turn information as the principal cause. Origin transforms to zero; ENU orientation unit tests pass. All master fixes have nonnegative status and maximum Header gap 0.1 s in this bag, but this does not establish perfect GNSS truth, lever-arm calibration or jury-frame agreement. Route mismatch/projection is not the cause of the default no-route result because no route is used.

Removing all GNSS after the initial window and repeating replay gives exactly identical output arrays. GNSS is not a continuous runtime dependency.

## Reproduce

Use existing Python/NumPy/SciPy/Matplotlib; do not install anything:

```bash
cd /home/iam220904/rezerv_odometr/tram_odometry_solution
PYTHONNOUSERSITE=1 python3 tools/position_eval.py --bag dataset/data/30618_d4ba7005 --output results/position_eval
python3 -m unittest discover -s tests -v
source /opt/ros/humble/setup.bash
colcon build --symlink-install
source install/setup.bash
export ROS_DOMAIN_ID=75 ROS_LOCALHOST_ONLY=1
python3 tools/position_ros_check.py
```

The last script runs launch plus a 45-second 1x bag fragment with --clock 100, then stops them. It checks timestamps, frames, finite pose/twist/covariance, motion, post-warmup continuity and diagnostics. Local DDS sockets must be available. It saves `live_ros_check.json`, raw `live_position.csv`, diagnostic messages and launch/player logs. Check the saved status rather than assuming success from this documentation.

Offline files in `results/position_eval/`: summary.json/txt, per_bag_metrics.csv, comparison.npz, position_comparison.png, position_error.png, trajectory_comparison.png, evaluation.log, unit_tests.log. Plots intentionally display the no-map failure and label GNSS as reference only. No along-/cross-track plot is generated.

Limitations: one offline bag and a short ROS fragment, no independent route, no branch selection, no reverse-velocity support, approximate covariance, unverified global jury-frame/antenna alignment. Core/model/evaluation hashes are saved in velocity_files_before.sha256.
