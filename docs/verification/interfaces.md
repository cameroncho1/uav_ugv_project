# Verification interfaces

Implemented schema: `1`. Package: `uav-ugv-verification` 0.1.0. [Requirements](requirements.md) | [CLI](../../verification/README.md)

ROS bag/setup mappings, native telemetry, clock binding, and SDF geometry are specified in [simulator integration](simulator_integration.md). Numerical record and monitor contracts remain independent of ROS.

## Python API

```python
from uav_ugv_verification import evaluate, inspect_run, load_profile

profile = load_profile("verification/examples/full_run.yaml")
inventory = inspect_run("verification/examples/full_run", profile)
evaluation = evaluate("verification/examples/full_run", profile)
payload = evaluation.to_dict()
```

`evaluate_trace(reader, profile)` supports additional adapters directly through the public `TraceReader` protocol. The protocol exposes a manifest, stream names, ordered `records(stream, interval)` iterators, and `inspection()`. Readers validate the complete input before evaluation. Monitors consume normalized records; reports consume saved results. Adapters and numerical checks have no ROS imports. Stream alignment retains one current/next record per stream plus equal-time batches; event checks retain event-window metadata, not whole recordings. Response checks may rescan a numerical stream per event.

`inspection()` returns `run_id`, `source_run_id`, `adapter`, `execution_status`, `intervals`, `source_sha256`, and `streams`. Each stream summary contains integer `records`, `partitions` keyed by `CLOCK/EPOCH`, total `sequence_gaps`, and `sequence_gaps_by_partition`. The evaluator uses these validated summaries to identify missing state-changing events. `source_sha256` maps source artifact names to SHA-256 strings.

## Run manifest

Normalized run directories contain `manifest.json` and the declared JSONL files. Paths are relative to the run directory and cannot escape it.

| Field | Type | Meaning |
| --- | --- | --- |
| `schema_version` | integer `1` | Input schema |
| `run_id` | nonempty string | Unique attempted run |
| `source_run_id` | nonempty string | Independent source-drive group for statistics |
| `adapter` | `normalized` | Persisted normalized input |
| `status` | `completed`, `aborted`, `failed` | Execution status, separate from property verdicts |
| `intervals` | nonempty list | `{clock, epoch, start_ns, end_ns}`; one interval per clock/epoch; end > start |
| `streams` | object | Canonical stream name to relative JSONL path; explicit empty files are permitted |
| `provenance` | object | Preserved metadata: code commit/dirty patch hash, world, seed, effective parameters, dependency revisions, synthetic/recorded origin |
| `frame` | string, default `map` | Normalized global frame |
| `reference_point` | string, default `model_origin` | Shared pose reference after any adapter transforms |

The CSV adapter synthesizes a `dataset_csv` manifest from the existing files and completion metadata. It does not require or consume a custom CSV manifest. No navigation capability is inferred from raw wheel odometry or truth. `source_run_id` comes from `faults.json.source_run` when present; otherwise the directory name. Record files and metadata are hashed.

## JSONL record

One record per line; schema is inherited from the manifest:

```json
{"time_ns":1000000000,"clock":"sim","epoch":0,"source":"controller","sequence":4,"valid":true,"data":{"linear":0.0,"angular":0.0}}
```

Required envelope: integer nonnegative `time_ns`, `epoch`, `sequence`; nonempty `clock`, `source`; object `data`. Optional `valid` defaults true; optional `frame` identifies global pose/route data. Sequences strictly increase per stream/clock/epoch; source stays constant within that partition. Sequence gaps retain missing-evidence status. Timestamps may repeat in sequence order but cannot decrease within an epoch. Clock reset requires a declared new epoch.

Optional `source_sequence` is a source-wide causal counter. Only this explicitly shared counter can order equal-time records across streams from the same source. Per-stream `sequence` values are not comparable across streams; otherwise consequential equal-time order remains ambiguous.

Times are integer nanoseconds in the named clock domain, not implicitly receipt timestamps. Event/decision times identify actual application/emission. Pose times identify acquisition. Consumer/actuator timing uses explicit steady-clock partitions; unsynchronized domains cannot be compared. Extra payload metadata is retained. Envelope fields and required payload fields are strictly checked; NaN/infinity and duplicate JSON/YAML keys are invalid. Explicitly invalid samples may omit payload fields and create a coverage gap. Event records must remain valid, complete events.

| Stream | Required `data` fields | Interpretation |
| --- | --- | --- |
| `truth_pose`, `estimated_pose` | finite `x`, `y`; envelope `frame: map` | Positions of manifest reference point; optional covariance/reference metadata is retained |
| `goal` | `x`, `y`, `revision`; `frame: map` | Latched goal state; same revision cannot change coordinates |
| `requested_command`, `actuator_output` | finite `linear`, `angular` | m/s and rad/s; output is emission, not measured motion |
| `actuator_input` | `linear`, `angular`, boolean `accepted` | Actual accepted/rejected input at actuator receipt time |
| `measured_motion` | `linear`, `angular`, boolean `simulation_advancing` | Independent motion measurement; optional nonnegative `uncertainty_ns` bounds stop-time uncertainty |
| `guard_state` | boolean `guard_valid`, nonempty `reason` | Full consumer guard result; covers frame/covariance/receipt rejection |
| `obstacles` | `shapes` list; optional boolean `static` | Resolved independent geometry; `[]` explicitly declares empty geometry |
| `route` | `goal_revision`, `points` of at least two `[x,y]` pairs; `frame: map` | Authoritative route with actual publication time |
| `uav_observation` | object, possibly empty | Presence/validity and acquisition age provide freshness; positions are not interpreted as map coordinates |
| `detector_decision` | boolean `alarm`, `scope` | `binary`, `channel`, or `class`; channel/class need `sensor`, `channel`; class also needs predicted `fault_class` |
| `events` | `kind`, `event_id` | Actual events below; scheduled/requested events do not qualify as actual onset |
| `grid`, `raw.*` | object | Preserved adapter payload; no standalone verdict |

Box shape: `{kind: box, x, y, half_x, half_y, yaw?}`. Cylinder: `{kind: cylinder, x, y, radius}`. Extents/radius are positive; coordinates/yaw finite. Optional IDs are unique. R5 requires the observed hazard's ID in the geometry snapshot. Geometry marked `static: true` latches until replaced; other observations expire at the rule's gap limit. Global poses must already be transformed to ENU `map`; raw body-frame sensor data remains `raw.*`.

| Event kind | Additional fields | Use |
| --- | --- | --- |
| `fault_start`, `fault_end` | `sensor`, `channel`, `fault_class`; shared `event_id` | Actual fault interval and detection trigger |
| `command_loss` | none | Controlled loss begins after the last accepted actuator input |
| `hazard_observed` | `goal_revision`, `obstacle_id`, boolean `qualifying` | Hazard enters the observed route/grid under the declared distance envelope |

CSV timestamps convert from decimal seconds to nearest integer nanoseconds without pretending to restore lost source precision. Fault transitions use per-sample masks across IMU/wheel/GPS; actual first affected/clear samples define events. Short-fault gaps remain nominal. The planned `faults.json` envelope is preserved in provenance.

## Profile

Required fields: `schema_version: 1`, string `id`, string `version`, `calibration` in `fixture/provisional/calibrated`, positive integer `max_gap_ns`, and `requirements` object. Unknown fields/parameters are rejected.

Each selected R1-R8 rule requires boolean `applicable` and `mandatory`. Optional `clock` defaults `sim`; `max_gap_ns` overrides the profile; `parameters` overrides typed defaults. An excluded rule requires a nonempty `reason`. Unselected rules report `NOT_APPLICABLE`. Standard profiles are in [verification/profiles](../../verification/profiles).

Recorded R4 profiles require a `steady:` clock. Only explicitly synthetic `calibration: fixture` profiles permit a different clock label for analytic tests.

Dynamic values are right-continuous on `[sample_time, min(next_sample, sample_time + max_gap_ns))`. Goal revisions and explicitly static geometry latch. No data extends through a source sequence gap. Exact response deadlines use closed intervals; R6 truth interpolation alone may bracket future truth for offline scoring.

## Results and reporting

`result.json` contains schema/evaluator version, evaluator-source hash, run/source IDs, profile ID/version/hash/calibration, execution status, preserved provenance, source file hashes, eight requirement results, exit code, and reason. Each requirement includes definition version, mandatory flag, verdict/reason, normalized parameters, clock/epoch intervals, eligible duration, trigger/completion/missing counts, up to 32 witnesses, and per-interval metrics. Omitted witnesses are counted; original recordings remain the full evidence.

Verdicts: `PASS`, `FAIL`, `INCONCLUSIVE`, `NOT_APPLICABLE`. Across epochs, witnessed failure takes precedence; otherwise any incomplete/unexercised interval makes the requirement inconclusive. Missing state-changing actuator/fault events prevent R4/R8 from manufacturing failures based on unknown state. No obligation crosses an epoch boundary.

Exit codes: `1` witnessed mandatory failure; otherwise `2` incomplete mandatory evidence, uncompleted run, invalid input, or no mandatory applicable rule; otherwise `0`. Optional failures remain visible. Invalid input emits `error.json` and removes stale result artifacts in that output directory; each retained attempt needs its own output directory.

Reports load saved `result.json` files without source recordings. Identical copies deduplicate; conflicting versions of the same run/profile/evaluator identity are rejected. Cohorts separate profile content, evaluator implementation, simulation commit/dirty patch, and calibration status. Missing simulation revision is labeled `unrecorded`. Verdict count plots combine cohorts only as labeled descriptive counts; CSV/JSON retain separation.

Detector intervals group by `source_run_id`, preventing variants/windows from increasing the independent trial count. Incomplete sources do not count as clean successes. R8 per-run probability uses fault-free source runs only; nominal event/exposure rates may also include faulted runs' nominal portions. Zero eligible trials produce null estimates/intervals. Statistical independence remains an experiment assumption, not a property the reader can prove.
