# TRAIN map and held-out route evidence

## Status and separation

The unchanged `results/splits.json` contains **57 TRAIN / 20 DEV / 20 TEST** bags. The workspace already contained a generated map, DEV results and TEST results when this audit began. The delivered map and its runtime follower were therefore preserved: **no TEST replay, no map tuning and no follower changes in this audit**. The existing TEST artifacts are the sole reported TEST evaluation. Historical exactly-once execution and a pre-TEST immutable freeze cannot be independently proved from the available artifacts. `audit/preservation.json` is explicitly a post-evaluation record, not retroactive proof of a freeze.

The current map's geometry and origin were reconstructed exactly in a temporary directory using **TRAIN files only**. `tools/audit_route_evidence.py` reads no TEST bags. Exact node/origin equality is recorded in `results/route_eval/audit/train_provenance.json`. Temporary reconstruction is deleted; the delivered map is not overwritten. The map hash is `9bce24fd6792c18cca12df7837be3c4b6c7368aa9c2f9f3224214fb531d6b627`.

## TRAIN geometry

`src/tram_odometry/config/train_path_map.json` contains **7,379 directed nodes**, supported by **37 TRAIN bags**. Each retained node has at least 3 distinct training-bag contributors (median support 15, maximum 19). This demonstrates repeated spatial traversals. There are 3,668 eastward and 3,711 westward nodes. Bounds are east [-1949.747, 2651.541] m and north [-657.161, 657.843] m. A 5 m proximity graph has one 7,378-node component and one isolated node; proximity does not establish physical rail connectivity.

Generator: valid master GNSS fixes, rejection of implied >30 m/s jumps with a 2 m allowance, direction from +/-1 s displacement >=1.5 m, then means within 2 m grid cells and eight heading sectors. Cells observed in fewer than three TRAIN bags are removed. 701 of 441,575 input fixes were rejected. The coordinate origin is the median TRAIN latitude/longitude/altitude: [55.804825103333336, 37.420043015, 167.4321], WGS84 converted through ECEF to ENU.

**2 m is cell size, not uniform point spacing.** Median nearest-neighbor spacing is 1.390 m; maximum nearest-neighbor spacing is 5.297 m. Direction sectors help separate opposing traversal directions, but close parallel tracks can merge within a cell. Repeated traversals are averaged; turns remain in the local direction field. This is not uniform arc-length resampling.

Format: `type=directed_track_nodes`, `origin_lla`, `cell_m`, `nodes` with [east, north, up, unit east direction, unit north direction, sample count, contributing bag count], and metadata containing TRAIN bag names, generation method/time and frame.

**Incomplete requirement:** no explicit branch edges, switch choices or graph station exist. Low-support terminal/depot branches are discarded, not preserved. The map is a reusable directed track field, not a complete pathgraph. Terminal/depot labels are descriptive, not surveyed semantic labels; independent disconnected routes have not been established.

## Runtime

The existing optional `path_map_file` parameter loads this map. Empty parameter preserves no-map behavior. GNSS initializes pose/heading only in the existing three-second Header-time window. Afterwards `MapFollower` advances using unchanged estimated distance and compatible local map tangents, with lateral map correction. It holds heading off-map and reports `path_map_off_map`. No later GNSS is used by the follower.

Map and position code remain unchanged after reviewing TEST results. One inspection finding is retained as a limitation: the fallback that obtains heading from the map changes `self.yaw` after local `xyz,yaw` have already been calculated for follower initialization. Thus stationary starts without a measured heading are not fully reliable. Changing this now would require a new, separately labeled experiment and cannot be claimed against the existing frozen TEST results.

## DEV geometry audit (corrected measurement definitions)

The old DEV coverage computation carried moving directions through stops; its nearest-node residual was labeled cross-track. Those old files are preserved but should not be used for a strictly moving-sample coverage claim.

Corrected audit: displacement >=1.5 m in +/-1 s, span >=1.6 s, no Header gap >0.25 s, directions not carried through stops. Coverage means within 3 m of a compatible TRAIN node (heading dot >=0.7; up to 64 nearest nodes searched within 8 m).

- 106,249 moving DEV samples, 102,767 covered: **96.7228% pooled coverage**.
- Covered nearest-node distance RMSE: **0.69595 m**.
- Covered lateral residual RMSE: **0.43249 m**, normal to the nearest compatible map-node tangent. This is conditional on coverage and is not whole-route graph station accuracy.
- Five DEV bags have no eligible moving reference, listed in `audit/dev_geometry_summary.json`; this includes stationary bags, not only missing GNSS.
- Existing runtime replay: 18/20 DEV bags evaluated, two without reference; one final-error >50 m failure, `30618_21dd3af3`. It never engages the map and finishes about 1,488 m wrong. Per-bag uncovered bounds are now saved in `audit/dev_geometry_metrics.csv`. No fixes were made based on this audit.

## Existing TEST results, preserved

20/20 bags have saved position metrics. Errors are horizontal Euclidean differences at master fixes matched within 0.05 s after initialization, in the shared TRAIN-map ENU frame. MAE is mean Euclidean error. Final error refers to the last matched fix. Along/cross values are error components relative to the GNSS-derived local travel direction, carried through stops; **they are not arc-length station errors on a graph**.

| Metric | No map | TRAIN map |
|---|---:|---:|
| Median final error, m | 4925.98 | 7.55 |
| Worst final error, m | 9860.73 | 176.91 |
| Median position RMSE, m | 2999.96 | 4.89 |
| Pooled position RMSE, all 20 bags, m | 4229.16 | 50.51 |
| Median along-component RMSE, m | 2363.66 | 4.84 |
| Median cross-component RMSE, m | 3285.55 | 1.16 |

TRAIN-map median per-bag signed along mean / RMSE / maximum absolute: **-4.45 / 4.84 / 23.12 m**. Cross: **-0.070 / 1.16 / 5.72 m**. Global maximum absolute along/cross errors: **221.64 / 195.68 m**. Exact pooled component means/RMSE are NOT VERIFIED: old aggregation weights components by position sample count, although direction availability differs. The all-bag position pooled RMSE above is corrected from saved CSV totals, without replaying TEST.

**Drift denominator warning:** existing final-drift median is 0.13675%, but the denominator sums unfiltered GNSS fix increments, including stationary noise/jumps. For example, `30618_defd0170` has 20,825 m fix-path length versus 5,195 m of covered GNSS-speed distance in existing velocity evidence. It is not a reliable whole-run travelled-distance metric. Using the existing covered GNSS-speed denominator yields a median endpoint-error ratio of **0.14398%**, but this also has interval/coverage differences. **Exact whole-run drift percentage remains NOT VERIFIED.** No estimator rerun or tuning was used for this reporting audit.

## Major TEST failures (post-hoc diagnosis, no tuning)

Five bags exceed 50 m final error; two exceed 100 m maximum error. Six exceed 30 m final error:

- `30618_a869780d`: final 176.91 m, along mean -207.78 m, 20.60 m off-map; initial matched error only 0.001 m and existing integrated-distance error -6.29 m. This is not explained by initial position error or ordinary distance drift. A route-following/branch/direction problem is plausible, but not established without a saved full trace. Do not label it a proven wrong branch.
- `30618_defd0170`: final 86.52 m versus saved distance error -85.91 m, zero off-map distance. Strong evidence for along-distance drift; GNSS fix-path noise also corrupts the drift denominator.
- `30618_0686195f`: final 79.60 m versus distance error -77.83 m, 15.32 m off-map. Mainly consistent with distance drift, with some off-map travel.
- `30618_88548b02`: final 38.33 m versus distance error -39.93 m, zero off-map distance. Consistent with accumulated distance error.
- `30618_27e994fc`: final 143.62 m, 150.66 m off-map, distance error -45.19 m. Off-map following adds error beyond ordinary distance drift; missing geometry versus follower departure is not distinguished by saved scalar metrics.
- `30639_0be558e2`: final 54.14 m, 48.47 m off-map, distance error -21.88 m. Mixed distance/off-map failure; exact cause remains unverified.

## Reproduction without repeating TEST

Existing dependencies only; no installation:

```bash
cd /home/iam220904/rezerv_odometr/tram_odometry_solution
PYTHONNOUSERSITE=1 python3 tools/audit_route_evidence.py
source /opt/ros/humble/setup.bash
colcon build --symlink-install
source install/setup.bash
export ROS_DOMAIN_ID=76 ROS_LOCALHOST_ONLY=1
python3 tools/check_path_map_runtime.py
```

Latest check: colcon build succeeded (two packages); the map-enabled DEV run delivered 386 finite position messages in 20.019 seconds, reported `ok; path_map`, stayed alive and both processes exited cleanly. Existing velocity/performance files and the map/follower hashes remained unchanged. See `audit/completion.json`.

The runtime check plays DEV bag `30618_af7496f0` for 20 seconds with --clock 100 and the frozen map. Read `results/route_eval/audit/runtime_check.json` and logs for actual results. Local DDS sockets must be permitted. To launch manually:

```bash
ros2 launch tram_odometry odometry.launch.py use_sim_time:=true path_map_file:=/home/iam220904/rezerv_odometr/tram_odometry_solution/src/tram_odometry/config/train_path_map.json
```

The original generation/evaluation commands, for a **new isolated experiment only**, are below. Do not overwrite the delivered map/results after TEST inspection. CACHE must contain timestamp-preserving NPZs from `tools/audit_dataset.py`; generation reads TRAIN only. DEV decisions must finish before freezing map, follower, model and scripts. Run TEST once after that freeze; archive all hashes and results. Historical TEST count cannot be established by rerunning it.

```bash
mkdir -p /tmp/new_route_experiment
PYTHONNOUSERSITE=1 python3 tools/build_path_map.py CACHE --splits results/splits.json --output /tmp/new_route_experiment/map.json
PYTHONNOUSERSITE=1 python3 tools/route_eval.py CACHE --splits results/splits.json --config src/tram_odometry/config/model.json --path-map /tmp/new_route_experiment/map.json --split dev --output /tmp/new_route_experiment
sha256sum /tmp/new_route_experiment/map.json src/tram_odometry/tram_odometry/position.py tools/route_eval.py results/splits.json > /tmp/new_route_experiment/freeze.sha256
# Only after DEV decisions are complete; known metric limitations above still apply:
PYTHONNOUSERSITE=1 python3 tools/route_eval.py CACHE --splits results/splits.json --config src/tram_odometry/config/model.json --path-map /tmp/new_route_experiment/map.json --split test --output /tmp/new_route_experiment
```

Original plots and results remain under `results/route_eval/`; they predate this audit. Audit records, corrected DEV definitions, annotated TEST scalar metrics, preservation hashes and original documentation are under `results/route_eval/audit/`. No velocity estimator, calibration, split, velocity evidence or performance evidence was modified.
