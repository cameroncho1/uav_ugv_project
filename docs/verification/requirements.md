# Executable requirements

Implemented definitions: version `1`, with explicit R7 scope suffix. [Interfaces](interfaces.md) | [Specification](milestone_plan.md)

`G` means always over observed coverage; `F[0,tau]` means a response exists within the closed deadline. These are sampled/event-trace checks, not a proof of continuous physical safety. Physical margins and timing slack retain separate units; no general STL robustness score is claimed.

## Shared semantics

- Evaluate each declared clock epoch separately. Missing streams/clocks, gaps, invalid samples, unsupported scope, and unexercised conditional properties are inconclusive. Explicit profile exclusions are not applicable.
- Known violations take precedence over incomplete evidence elsewhere. Missing state-changing events cannot establish a violation whose trigger/state is uncertain.
- Dynamic state expires at `max_gap_ns`; goal revisions and static geometry latch. Data gaps never authorize extrapolation. Repeated trigger samples do not restart deadlines.
- Checks use simulation time unless the profile declares a consumer/actuator steady clock. Frozen simulation cannot establish physical settling.
- `quiet = abs(linear) <= linear_epsilon_mps AND abs(angular) <= angular_epsilon_rps`. Defaults are 0.001 m/s and 0.001 rad/s. R4 physical thresholds are separate.
- Provisional profiles are runnable simulation assumptions, not calibrated limits. Calibrate/freeze runtime thresholds during M2 before M3 held-out trials.

## R1: sampled footprint clearance

`G_samples(clearance >= -contact_tolerance_m)`.

Compute signed disk-to-oriented-box/upright-cylinder clearance from independent geometry at actual truth samples. Defaults: radius 0.5 m, tolerance 0 m. Explicit empty geometry passes with no finite nearest-obstacle margin; missing geometry is inconclusive. Output: sampled minimum clearance, eligible coverage, violating sample timestamps, and `scope: sampled_disk_proxy`.

An obstacle can lie between safe endpoint samples; M1 reports sampled scope and does not certify swept clearance. Disk overlap is a footprint-proxy failure, not automatically chassis contact. M2 supplies resolved collision geometry, transforms, and sampling limits.

## R2: goal arrival with dwell

For each active revision: `F[0,deadline](physical_distance <= physical_tolerance AND estimated_distance <= goal_tolerance AND quiet for dwell)`.

Defaults: estimated tolerance 0.5 m, physical tolerance 0.85 m, dwell 1 s, deadline 60 s. The physical tolerance preserves the legacy camera scorer's default; the two distances remain separate. A new revision starts a new obligation at its record time, or the evaluation boundary for an existing goal. Dwell resets on motion, invalid input, or a revision change; it cannot cross goals or a missing-data interval.

Set `deadline_ns` from the declared scenario. If using the completion plan's `2*L/0.5 + 30 s`, compute it from the predeclared reference length, not the actual trajectory after the run. M1 does not infer `L`. A completed dwell before the deadline is required; a truncated deadline without arrival is inconclusive. Output: revision count, completed arrivals, maximum arrival latency, and separate minimum physical/estimated distances.

## R3: pose-guard response and sustained hold

Each guard-invalid interval creates `F[0,response_ns](quiet)`, followed by quiet commands until observed recovery or the evaluation end. Default bound: 0.5 s. A short recovery does not cancel an already-created response obligation. An already quiet command gives zero latency.

Input is the actual consumer guard verdict, including invalid frame/covariance/stamp and receipt-age failures. No scalar-only approximation is substituted. Timely zero followed by nonzero motion during the same known invalid interval fails. Same-source command sequences preserve intermediate values at equal timestamps. Ambiguous cross-source order or unknown recovery is inconclusive. Output: trigger/completion counts, response latency/slack, eligible coverage, and hold violations.

## R4: actuator expiry and physical settling

Controlled `command_loss` events require a known nonzero pre-loss output and a last accepted input. Later accepted input invalidates that loss observation. Measure from the last accepted input's actual receipt time, not process-kill time.

1. Watchdog output becomes quiet within `expiry_ns` (default 0.40 s: provisional 0.35 s timeout plus nominal 50 ms delivery).
2. Independent measured motion becomes quiet within `settling_ns` (default 0.75 s) and remains quiet for `dwell_ns` (default 0.20 s). Dwell confirmation may follow the stopping deadline; stop onset must meet it.

Physical thresholds default to 0.02 m/s and 0.02 rad/s. Timing uncertainty overlapping the deadline is inconclusive. A paused simulation cannot confirm settling. Report output and physical latency/slack separately; both are required for R4 pass. The worker's sleep period is not a hard scheduling guarantee. Calibration and real actuator telemetry are M2.

## R5: hazard response

For each qualifying `hazard_observed`: `F[0,response_ns](fresh_clear_route OR completed_quiet_dwell)`.

Defaults: 2 s response, 0.20 s stop dwell, 0.5 m route footprint radius. The route must be published after the trigger, carry the same active goal revision, and clear every segment against independent geometry containing the hazard ID. Cross-source equal-time route publication is ambiguous. An old/wrong-goal route never qualifies. A quiet command must have complete dwell coverage; stopping confirmation is the response time for this definition.

The trigger asserts actual grid observation and envelope qualification. M1 does not infer visibility or `d_min` from truth alone. Appearance-to-observation and physical stopping remain separate runtime measurements for M2. Output: trigger/completion counts, command/route response latency, and timing slack.

## R6: conditional localization accuracy

`G_estimate_samples((guard_valid AND uav_fresh) -> position_error <= error_limit_m)`.

Defaults: error 0.5 m, UAV age below 0.5 s. Match each acquisition-stamped estimate to exact or bounded interpolated independent truth. No truth extrapolation or interpolation across epochs/sequence gaps. Guard validity is causal; offline truth interpolation does not enter estimation/navigation.

Output: conditional sample count/RMSE/max error/margin, full-run matched sample count/RMSE, and joint eligible duration. No fresh eligible UAV observations is unexercised, not a successful accuracy bound. Full-run RMSE may include ineligible samples and does not replace the conditional verdict. Missing truth/estimate/guard coverage remains explicit.

## R7: new fault detection

For actual `fault_start`: `F[0,response_ns](new matching alarm)`. Default bound: 5 s.

- `1.class`: scope, sensor, channel, and predicted class must match.
- `1.channel`: a channel-scoped alarm must match sensor/channel.
- `1.binary`: a global binary alarm; no class-identification claim.

An alarm already active before onset does not count as new detection. A recorded clear state followed by a matching alarm establishes a new detection; unknown prior state, unsupported scope, and a truncated horizon are inconclusive. An equal-time cross-source onset/alarm is ambiguous. Measure latency to decision emission, not window start/center. A complete horizon without a new alarm fails. Output: per-run triggers/completions, maximum latency/minimum slack, and pre-existing alarms.

The current offline models do not predict fault classes. Ground-truth `fault_type` cannot be copied into predictions. Non-overlapping 100-sample windows have sensor-specific durations; binary offline results do not establish a uniform five-second online guarantee.

## R8: nominal false alarms

`G_nominal_exposure(no configured alarm flag)`.

Parameters: `warmup_ns`, `recovery_ns`, `merge_gap_ns` default zero; `decision_keys` defaults `[binary]`. Keys are `binary`, `channel:SENSOR:CHANNEL`, or `class:SENSOR:CHANNEL:CLASS`. Coverage requires every configured key. Other keys do not clear an active flag. Declare all channels/classes needed for the claimed scope.

Exclude actual fault intervals and configured recovery; warmup begins at each evaluation interval. Nominal flags fail, regardless of whether other channels are missing. Count alarm episodes by key, merging interruptions within `merge_gap_ns`. Report eligible nominal exposure, alarm events, events/hour, and fault-free-run status. No eligible nominal exposure is inconclusive.

Report aggregation uses independent source runs for binomial statistics, not windows/events. Fault-free source runs support per-run false-alarm probability; faulted runs contribute only descriptive nominal exposure/event counts. Exact two-sided 95% intervals and the zero-alarm one-sided upper bound are distinct outputs. With zero eligible trials, probability and interval are null.
