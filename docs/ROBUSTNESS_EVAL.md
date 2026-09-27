# Slip/anomaly robustness evidence

## Scope and outcome

No thresholds, velocity model, split, calibration or validated evaluation pipeline were changed. The original seven synthetic scenarios were rerun unchanged in a temporary output layout; all results stayed finite for both estimator and wheel-mean baseline. Original `results/stress_tests.json` was preserved. Its only automated pass condition is finiteness, not accuracy or recovery.

A separate 15-case instrumented harness covers missing cases and measures acceptance, flags, distance deviation and recovery: **13 PASS, 2 FAIL**. These are synthetic tests, not labeled real wheel-slip validation. Tests use an analytic 8+2*sin(0.1*t) m/s trajectory over 40 s, 10 Hz wheels, 20 Hz commands/observations, current model.json, and no GNSS. The detailed harness is noiseless; the original seven scenarios include reproducible wheel noise.

Criteria declared before execution: finite state, no exception, advancing state timestamps at every 20 Hz tick, maximum speed error <=1 m/s, maximum longitudinal distance deviation from clean replay <=10 m, sane wheels accepted after the fault, recovery within 5 s. Recovery means error relative to clean replay <=0.2 m/s AND `ok` status for a continuous 1 s window. These are evidence thresholds, not official scoring thresholds or estimator tuning parameters.

## Mechanisms implemented

- **Disagreement:** fresh bogie readings differing by >0.45 m/s. Prefer the reading closer to the model prediction; reject the worse candidate if its residual exceeds the other by >0.15 m/s.
- **Model residual:** gate 0.65 + 0.015*v m/s.
- **Jump:** within 0.4 s, a wheel increment exceeding 3*dt+0.2 m/s is suspicious.
- **Input validity:** reject nonfinite core inputs, wheel magnitude >40 m/s after conversion, noninteger/controller values outside [-15,15], duplicate or backward stream timestamps, and messages older than prediction by >0.25 s. ROS adapter drops nonpositive Header stamps and nonfinite values before the core; those drops now have a separate counter.
- **Reduced trust:** rejected measurements perform no Kalman correction. Accepted disagreement measurements multiply R by 9. Delayed measurements add (0.25*age)^2 variance. Reacquisition inflates velocity variance and multiplies R by 4. These do not estimate a physical slip ratio.
- **Fallback:** after 0.5 s without accepted wheels, status becomes degraded/model_only; model propagation continues. Commands older than 0.5 s use neutral command. After 2 s without accepted wheels, target acceleration becomes zero and the lagged acceleration decays, with increased uncertainty. This is coasting, not a guaranteed accurate stop model.
- **Recovery:** after >3 s without accepted wheels, mutually consistent fresh wheels without a jump may be reacquired. Slip suspicion expires after 0.5 s without renewed suspicion. Fresh accepted wheels clear stale status. A stop reset requires both nearly zero wheels AND already low estimated speed.
- **Adaptation:** accepted consistent wheel acceleration updates an auxiliary acceleration and bounded +/-0.6 m/s² disturbance estimate. This is online bias correction, not physical adhesion estimation or coefficient retuning.

Implementation: `src/tram_odometry/tram_odometry/core.py`. Values are from current `config/model.json` plus Config defaults. No model settings were changed.

## Detailed synthetic results

| Scenario | Result | Max speed error m/s | Max distance deviation m | Recovery s |
|---|---|---:|---:|---:|
| Clean control | PASS | 0.0085 | 0 | 0 |
| Front sudden +5 m/s spike | PASS | 0.0085 | <0.001 | 0.5 |
| Rear sudden +5 m/s spike | PASS | 0.0085 | <0.001 | 0.5 |
| Front +5 m/s slip for 5 s | PASS | 0.0085 | <0.001 | 0.5 |
| Front/rear disagreement +0.48 m/s | PASS | 0.0085 | <0.001 | 0.4 |
| Both wheels missing for 2 s | PASS | 0.0143 | 0.020 | 0 |
| Driver command missing for 5 s | PASS | 0.0085 | 0.001 | 0 |
| Wheel NaN/Inf for 2 s | PASS | 0.0143 | 0.020 | 0 |
| Invalid controller value 16 | PASS | 0.0085 | <0.001 | 0 |
| Rear wheel missing for 15 s | PASS | 0.0085 | <0.001 | 0 |
| Both wheels +5 m/s slip for 2 s | PASS | 0.0146 | 0.022 | 0.5 |
| Both wheels locked for 1 s | PASS | 0.0132 | 0.008 | 0.5 |
| Backward Header samples, -2 s | PASS | 0.0530 | 0.015 | 0 |
| One front Header jumps +60 s | **FAIL** | **3.1965** | **580.374** | **Not recovered** |
| Both wheels +5 m/s slip for 10 s | **FAIL** | **5.1897** | **55.148** | **3.4** |

All 15 cases remain finite with no core exceptions. The future Header case fails output-state time continuity: the predictor advances far ahead and rejects subsequent normal-time input. Its state time remains stuck during this observation. The long simultaneous-slip case reacquires mutually consistent but wrong wheels; recovery after the fault does not erase accumulated distance error. Neither failure was hidden or addressed by retuning.

Distance is integrated longitudinal distance compared to clean replay, **not geometric map position**. Existing seven scenarios overlap the new cases; do not add 7+15 as independent coverage. Per-case flags, rejected counts, observed downweighting and acceptance records are saved. Null rejection-update metrics mean no applicable rejected update occurred; that metric is NOT VERIFIED for that case.

## Explicit trust-reduction evidence

In the paired update test the predicted state and rear measurement are identical; only the stored opposing-wheel observation differs. Both rear updates are accepted. Nominal R=0.000625, disagreement R=0.005625. Velocity correction falls from 0.0447365 to 0.0240630 m/s (53.79% of nominal). Thus the measurement influence decreases relative to prediction; a ninefold R increase does not imply a ninefold correction reduction.

The front-slip scenario rejects 51 measurements and observes 50 downweighted accepted updates. Rejected updates cause zero immediate velocity correction in the instrumented test. `acceptance_events.json` contains before/after values; counts inferred from covariance are not new estimator-internal counters.

## Diagnostics and minimal fixes

`/result/diagnostics` (`diagnostic_msgs/DiagnosticArray`) already exposes slip suspicion, missing accepted wheels, rejection/duplicate/out-of-order counts, velocity/distance uncertainty and model disturbance. `DiagnosticStatus.message` begins with estimator mode (`ok`, `degraded`, `model_only`); its suffix is position mode. The existing merged `values[status]` is position mode, so consumers must not use it as estimator mode. With no route, diagnostic severity can stay WARN due to unobservable position direction even when estimator mode is `ok`.

Minimal changes in node.py:

1. Reset `last_diagnostic` when starting a new clock episode. Previously backward clock jumps suppressed diagnostics until the old timestamp was reached. A focused test failed before the change and passed afterwards.
2. Count pre-core invalid ROS values/stamps in `invalid_inputs` (episode counter).
3. Publish `command_stale` as a diagnostic boolean.

`rejected` counts core validation/model rejections, not pre-core invalid messages. There is no dedicated recovery flag or downweight counter; recovery is observed through resumed acceptance, expired slip suspicion and mode returning to `ok`. No slip ratio or adhesion coefficient is estimated. No estimator behavior/model changes were made.

## Real ROS checks

- Unmodified DEV bag `30618_af7496f0`, 20 s at 1x with --clock 100: **390 velocity +390 position messages**, 19 diagnostic messages, all outputs and numeric diagnostic fields finite, maximum observed velocity receipt gap 0.0531 s, node/player exited cleanly.
- Separate direct synthetic ROS publishers: 18 s with front spike/slip, all vehicle-input dropout, wheel NaN, recovery, then a 50 s backward /clock jump. **361 messages per output**, finite, maximum observed gap 0.0999 s. Slip suspicion, model_only, stale command, 40 invalid messages, recovery to `ok`, and diagnostics after clock reset were all observed. The original bag was never changed.
- This is short operational validation only. Future-dated Headers and prolonged simultaneous slip were tested in the core harness, **not ROS**. No false restart loop or node crash was observed.
- **Real labeled slip validation: NOT VERIFIED.** No labels were supplied. Numeric diagnostics are checked as numbers; textual lists/status fields are not falsely treated as nonfinite values.

## Reproduce

Existing Python/NumPy/SciPy/Matplotlib and ROS dependencies only:

```bash
cd /home/iam220904/rezerv_odometr/tram_odometry_solution
PYTHONNOUSERSITE=1 python3 tools/robustness_eval.py
source /opt/ros/humble/setup.bash
source install/setup.bash
python3 -m unittest discover -s tests -v
colcon build --symlink-install
source install/setup.bash
export ROS_DOMAIN_ID=77 ROS_LOCALHOST_ONLY=1
python3 tools/robustness_ros_check.py
```

The synthetic evaluator automatically reruns the unchanged original stress script in a temporary layout and saves a copy under `results/robustness_eval/`; original results remain unchanged. The ROS script starts/stops launch/player itself and needs local DDS sockets. Use its dedicated domain without other publishers. Results are overwritten on rerun.

Evidence: `summary.json/txt`, `scenario_metrics.csv`, original-stress rerun files, acceptance_events.json, traces.npz, scenario_plots.png, runtime_robustness.json, runtime_samples.json, diagnostics_sample.txt, unit-test logs and ROS logs. No performance/velocity/route evidence was rewritten.

## Remaining scoring risks

1. Long common-mode slip/lock is not independently observable and can be reacquired as truth, causing substantial distance error.
2. Future-dated input Headers are not bounded against arrival time; they can move the predictor ahead and prevent timely recovery. Timestamp-offset policy needs a separate validated change.
3. Evidence is synthetic and short-run ROS, not labeled real slip. Finite-state tests do not guarantee accuracy through long dropouts, unknown acceleration or adversarial combinations.
