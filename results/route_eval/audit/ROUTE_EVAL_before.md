# Route / path-map evaluation (train-built map, held-out test)

## Split (unchanged, `results/splits.json`)
- train (57): 30618_e3d94878, 30618_b83d854d, 30618_b8044aa0, 30618_76e1f9c7, 30618_2f104a1d, 30618_2dbce472, 30618_b15eafc3, 30618_68d1748a, 30618_b95ca60a, 30618_095a115b, 30618_7b4d83f4, 30618_87afe526, 30618_0f120b35, 30618_0e41eac3, 30618_68847170, 30618_49fe4c54, 30618_bc5e53c2, 30618_0d865417, 30618_f19a4ac3, 30618_ab5921a4, 30618_073f08d1, 30618_0652866c, 30618_1551d0a9, 30618_22c1c589, 30618_9c09b081, 30618_e2dcf65f, 30618_28538acf, 30618_616ec56b, 30618_8158f0b0, 30618_79b204dc, 30618_2161b58b, 30618_4e1e3181, 30618_2255aade, 30618_20096314, 30618_7a152380, 30618_2d2fa7be, 30618_2f2f1175, 30618_5036aa78, 30618_e151d6e4, 30618_117c2d02, 30618_74559c73, 30618_3e012faf, 30618_0a83c933, 30618_81c22fee, 30618_9bbe6faa, 30618_0259fe53, 30639_d601d28f, 30639_3b3d9eb8, 30639_c31df386, 30639_dce52be4, 30639_92226df0, 30639_e4379d7f, 30639_9c362687, 30639_253671cc, 30639_44226bde, 30639_d3c43d69, 30639_d927f360
- dev (20): 30618_6236f680, 30618_3ba2326f, 30618_21dd3af3, 30618_4d487b0d, 30618_6cb3280a, 30618_3b36e5cd, 30618_748832b9, 30618_1cc230fa, 30618_98161270, 30618_45ff6f99, 30618_bab2fe58, 30618_2366c74a, 30618_40ffd323, 30618_2cb9ce37, 30618_af7496f0, 30618_437e855c, 30639_584b6e32, 30639_2b4a6347, 30639_4285f2bc, 30639_0ab96c59
- test (20): 30618_d4ba7005, 30618_dd8e6395, 30618_e9a34502, 30618_01f73500, 30618_33bec73f, 30618_dd8d0741, 30618_2050d396, 30618_a53d5f6f, 30618_67b89902, 30618_3e9f4952, 30618_082f1d65, 30618_a869780d, 30618_defd0170, 30618_0686195f, 30618_88548b02, 30618_27e994fc, 30639_9f0b519f, 30639_50956d6e, 30639_927002c2, 30639_0be558e2

## Data inspection (train + dev GNSS only)
All train and dev bags of both trams (30618, 30639) run the same ~5.5 km line (east–west, ENU x ≈ −1550…3080 m), in both directions, with a turnaround loop at the east end and a terminal area at the west end. They overlap spatially; no unrelated tracks. A few short bags are stationary in a depot/terminal area slightly off the line. Test GNSS was not inspected for any map decision.

## Map generation (`tools/build_path_map.py`, TRAIN only)
- Input: `/sensing/gnss/master/fix` of train bags with status ≥ 0; fixes implying > 30 m/s jumps rejected (701 of 441575).
- Frame: local ENU (east, north, up) metres about `origin_lla` = median of train fixes; WGS84 → ECEF → ENU (same `LocalFrame` as the node).
- Method: 2 m cells x 8 heading sectors; mean position and unit travel direction; direction from +-1 s fix difference >= 1.5 m; keep cells seen in >= 3 train bags; fixes with implied speed > 30 m/s rejected.
- Output: `src/tram_odometry/config/train_path_map.json` (`type: directed_track_nodes`), 7379 nodes, 2 m spacing; each node = mean position + unit travel direction + support counts. Metadata lists the 37 train bags with moving GNSS. The tool reads only `split=="train"`; `tools/route_eval.py` asserts no evaluated bag is in the map's bag list.
- Directed nodes keep both tracks of the double line and both travel directions separate. There are no explicit edges; branches (depot/terminal tracks) are followed only if present in ≥ 3 train bags, and branch choice is implicit (nearest nodes aligned with current heading).

## Runtime integration (minimal)
- `position.PathMap` (loader + spatial hash) and `position.MapFollower`; `PositionTracker(path_map=...)`.
- Launch/param: `path_map_file:=/ABS/PATH/train_path_map.json` (default empty = previous behaviour).
- GNSS is used only in the existing 3 s window (anchor position + heading). After that, each 1 m of estimated path distance moves the pose along the local mean tangent of map nodes within 5 m whose direction agrees with the current heading (cos > 0.7), with a 50% lateral pull toward the centreline. Off-map: heading held, `path_map_off_map` status. If heading is unknown at lock (stationary start, no dual-antenna heading), it is taken from the nearest map node within 5 m — added after a **dev** failure (30618_bab2fe58), before any test run.
- Velocity estimator unchanged.

## Metrics
Reference = test/dev master GNSS fix in the same ENU frame, only after the 3 s window. Horizontal error per fix (estimate matched within 0.05 s). Along/cross-track = error projected on the reference travel direction (±1 s fix difference, carried through stops). Drift % = final error / reference path length. Aggregates: median over bags, plus sample-pooled RMSE.

## Dev validation (18/20 bags with GNSS; 2 without reference)
- Map coverage (moving dev fixes within 3 m of a same-direction node): median 96.83%, min 81.68%. Map residual RMSE median 1.34 m.
- Median over bags, path map: final 5.76 m, RMSE 4.37 m, cross-track RMSE 1.18 m, along-track RMSE 4.12 m.
- Failure: 30618_21dd3af3 starts ~33 m from any map node (west terminal area not in ≥3 train bags) → map never engages, 1488 m final error (same as no map). Pooled dev RMSE 217.84 m is dominated by this bag.

## Held-out test (20/20 bags; map frozen before this run)
| median over bags | no map | train path map |
|---|---:|---:|
| final error, m | 4925.98 | 7.55 |
| RMSE, m | 2999.96 | 4.89 |
| MAE, m | 2532.70 | 4.41 |
| max error, m | 4961.85 | 22.26 |
| final drift, % | 53.21 | 0.14 |
| along mean / RMSE / max, m | -1317.02 / 2363.66 / 5872.37 | -4.45 / 4.84 / 23.12 |
| cross mean / RMSE / max, m | 246.12 / 3285.55 / 6936.36 | -0.07 / 1.16 / 5.72 |
| pooled RMSE, m | 4230.99 | 50.53 |
| pooled along / cross RMSE, m | 3062.03 / 2975.44 | 49.38 / 11.56 |
| worst final error, m | 9860.73 | 176.91 |

Poor test bags (final error > 30 m), from `test_metrics.csv`:
- 30618_a869780d: final 177 m, along RMSE 211 m, cross max 196 m, 21 m off-map — an early along-track offset (~200 m) persists; likely a wrong track/branch or initial misalignment. Not diagnosed further (test data).
- 30618_27e994fc: final 144 m, 151 m travelled off-map, cross max 85 m — leaves map coverage (also the worst velocity bag).
- 30618_defd0170 (87 m), 30618_0686195f (80 m), 30618_88548b02 (38 m): mostly along-track drift from integrated speed (0.4–0.6 % of 6.7–20.8 km).
- 30639_0be558e2: final 54 m, 48 m off-map.
Tram 30639 bags are systematically worse (18–54 m final) than 30618 (2–8 m on good bags).

## Limitations
- Along-track error is uncorrected path-distance drift; no map features re-anchor it.
- No explicit graph/edges: branch choice at switches is implicit; tracks seen in < 3 train bags are missing (off-map → heading held).
- Map depends on the master antenna position; the reference uses the same antenna.
- ROS node with `path_map_file` was built (colcon) but **not run live** with a bag; offline replay uses the same `PositionTracker` code.
- Whether a map from training GNSS is allowed must be confirmed with the organisers.

## Files
`tools/build_path_map.py`, `tools/route_eval.py`, `src/tram_odometry/config/train_path_map.json`; `results/route_eval/`: `dev_summary.json`, `dev_metrics.csv`, `test_summary.json`, `test_metrics.csv`, `route_map.png`, `dev_route_overlay.png`, `test_trajectory_comparison.png`, `along_track_error.png`, `cross_track_error.png`.

## Reproduce
Caches built with `tools/audit_dataset.py` (see `docs/VELOCITY_EVAL.md`; it reads every `*.db3` under the given directory — put the bags of the wanted split there). With `PYTHONNOUSERSITE=1` if `~/.local` NumPy breaks SciPy:
```bash
python3 tools/build_path_map.py CACHE --splits results/splits.json --output src/tram_odometry/config/train_path_map.json
python3 tools/route_eval.py CACHE --splits results/splits.json --config src/tram_odometry/config/model.json --path-map src/tram_odometry/config/train_path_map.json --split dev --output results/route_eval
python3 tools/route_eval.py CACHE --splits results/splits.json --config src/tram_odometry/config/model.json --path-map src/tram_odometry/config/train_path_map.json --split test --output results/route_eval
ros2 launch tram_odometry odometry.launch.py use_sim_time:=true path_map_file:=$PWD/install/tram_odometry/share/tram_odometry/config/train_path_map.json
```
Note: re-running `build_path_map.py` regenerates `generated_utc`; node values are deterministic.
