# Velocity accuracy evaluation (held-out test split)

## Data
- Split: `results/splits.json` (unchanged). Chronological per tram, whole bags; the model was fitted on `train` only (57 bags). `dev` = 20 bags, `test` = 20 bags.
- All numbers below are for the **20 `test` bags only**; all 20 have a GNSS reference. No parameters were changed for this evaluation.

## Reference and metrics
- Reference: horizontal speed of `/sensing/gnss/master/vel`, `hypot(east, north)`. GNSS is **not** fed to the estimator (causal replay of `core.Estimator`, identical to the ROS node); it is used only after replay to score it.
- Each GNSS sample is paired with the nearest estimator output (20 Hz, Header time) within 0.05 s. Error `e = estimate − reference`.
- RMSE = sqrt(mean e²), MAE = mean |e|, bias = mean e (negative = underestimate), pooled over all matched samples of all bags.
- `raw`: every matched sample. `gnss_agree`: also requires rover speed within 0.5 m/s of master; a filter on the **reference only**, used to expose GNSS glitches.

## Motion regimes (labelled from the GNSS reference, not from the estimate)
- stop: reference speed < 0.2 m/s
- acceleration / braking: reference acceleration > +0.15 / < −0.15 m/s²
- steady: moving, |a| ≤ 0.15 m/s²
- Acceleration = centred difference of reference speed over ±0.5 s; windows with a GNSS gap > 0.25 s or with less than 0.8 s of coverage are `unclassified` (1.5% of samples).

## Results (model observer, test, 20 bags, 223 335 matched samples)
| regime | samples | RMSE m/s | MAE m/s | bias m/s |
|---|---:|---:|---:|---:|
| overall (raw) | 223 335 | 0.0701 | 0.0291 | −0.0190 |
| overall (gnss_agree) | 221 641 | 0.0519 | 0.0287 | −0.0194 |
| acceleration | 54 433 | 0.0563 | 0.0338 | −0.0190 |
| braking | 53 444 | 0.0652 | 0.0374 | −0.0221 |
| steady | 43 156 | 0.0629 | 0.0429 | −0.0339 |
| stop (raw) | 68 864 | 0.0868 | 0.0102 | −0.0070 |
| stop (gnss_agree) | 68 287 | 0.0185 | 0.0089 | −0.0082 |

Baseline (mean of fresh wheel readings /3.6) and per-bag values: `results/velocity_eval/summary.txt`, `per_bag_metrics.csv`.
The raw stop RMSE is dominated by 27 samples in 4 bags where master GNSS reports ~0 m/s while the rover receiver reports the same speed as the estimate (1.3–9.6 m/s), i.e. reference dropouts.

## Live ROS check
80 s of test bag `30618_d4ba7005` played at 1× with `--clock`; `/result/velocity` recorded (1590 msgs, 20.0 Hz). Live minus offline replay: RMSE 0.0022 m/s, max 0.020 m/s. Live vs GNSS RMSE 0.0269 m/s, offline on the same interval 0.0265 m/s. File: `results/velocity_eval/live_ros_check.json`, `live_vs_offline.png`. Short interval only; not a full-bag ROS replay.

## Files
`results/velocity_eval/`: `summary.json`, `summary.txt`, `regime_metrics.csv`, `per_bag_metrics.csv`, `velocity_comparison.png` (median-RMSE test bag), `velocity_error.png`, `velocity_zoom_transients.png`, `regime_metrics.png`, `live_ros_check.json`, `live_vs_offline.png`.

## Reproduce
Only Python + NumPy + Matplotlib (+ SciPy, imported by `tools/calibrate.py`). If `~/.local` has an incompatible NumPy, prefix with `PYTHONNOUSERSITE=1`.
```bash
cd /home/iam220904/rezerv_odometr/tram_odometry_solution
# 1. Cache the 20 test bags (audit_dataset.py reads every *.db3 under the given directory)
mkdir -p /tmp/vtest && for b in $(python3 -c "import json;print(' '.join(e['bag'] for e in json.load(open('results/splits.json')) if e['split']=='test'))"); do mkdir -p /tmp/vtest/$b; cp dataset/data/$b/*.db3 /tmp/vtest/$b/; done
python3 tools/audit_dataset.py /tmp/vtest --cache /tmp/vcache --output /tmp/vaudit
# 2. Overall metrics (reproduces results/test/metrics.json) and per-regime evaluation + plots
python3 tools/evaluate.py /tmp/vcache --splits results/splits.json --config src/tram_odometry/config/model.json --split test --output /tmp/vreeval
python3 tools/velocity_eval.py /tmp/vcache --splits results/splits.json --config src/tram_odometry/config/model.json --split test --output results/velocity_eval
```
