# Finite-state verification of UGV navigation decisions

## Architecture and assumptions — report section 3

The UGV controller uses a persistent supervisor to authorize the existing path follower. Its five modes are INIT, NAVIGATE, HOLD_GUARD, HOLD_FAULT, and GOAL. Every mode except NAVIGATE requests zero linear and angular velocity. NAVIGATE permits the follower and does not require a nonzero command. The existing pose and route validators remain authoritative.

![Decision precedence](figures/supervisor.svg)

Memory includes a fault latch, a bounded clear-decision count, a recovery-age class, and a separate completed-goal latch. The latter prevents estimated-pose drift or an intervening hold from restarting an already completed goal. Completion means one valid estimated pose entered tolerance; R2's independent physical tolerance and quiet dwell remain separate checks.

The detector interface is optional and disabled in the supplied camera deployment because an online producer is unavailable. When enabled, missing/stale/invalid decisions hold. Only distinct, valid, contiguous clear decisions count toward recovery. Release requires ten clear decisions and at least five simulation seconds from the first clear acquisition stamp, while the latest input remains fresh. Controller callbacks do not count as clear windows. New goals do not clear fault recovery.

## Formal method — report section 4

A versioned decision table supplies mode precedence to a Python reference model and generated C++ data. Independently specified temporal safety and next-step predicates constrain navigation authorization, completed-goal persistence, reset behavior, readiness, and exact recovery-memory updates. An independent completion monitor is composed with the state graph.

Breadth-first exploration checks every reachable abstract transition from detector-disabled and detector-enabled initialization. The abstraction retains count and latch memory and represents elapsed recovery time as unstarted, young, or mature. Runtime numerical validators establish that maturity corresponds to an elapsed five-second acquisition-time interval. Exact nanosecond tests cover the threshold and freshness/gap boundaries.

Examples are `G(permit -> goal_valid & pose_valid & route_valid & !fault_latched & !goal_completed)` and `G(goal_completed & !new_valid_goal -> X goal_completed)`. The complete **${properties}-property** set is in `properties.csv`. These are discrete safety/next-decision obligations, not unconditional eventual recovery or physical arrival.

## Implementation and conformance — report section 5

The pure C++ transition and numerical validators have no ROS or simulator dependencies. The ROS controller processes every detector decision, including alarm/clear bursts, and reevaluates permission before each periodic command publication. Exact acquisition stamps, producer session/sequence, receipt age, and goal revisions remain explicit. Startup configuration cannot silently change detector participation at runtime.

The compiled transition probe embeds policy and implementation-source digests. Conformance compares every explored context/input and resulting mode, latches, count, age, and permission against the compiled function. Separate ROS tests exercise the actual controller's subscriptions and published Twist commands using declared input fixtures. The live simulator and actuator-watchdog checks remain distinct evidence.

## Results and analysis — report section 6

The graph contains **${controller_states} controller states**, **${product_states} product states**, and **${transitions} transitions**. All properties hold in this finite abstraction, and the compiled C++ function conforms on all ${transitions} transitions. **${detected_mutations} unsafe mutations** are rejected with retained shortest counterexamples. Mutations include premature recovery, double counting, invented timer maturity, forgotten goal memory, and vacuous indefinite holds.

The controlled ROS fixture passes **${controlled_cases} cases** covering missing input, exact recovery behavior, duplicate/freshness handling, goal drift, new revisions, stale routes, bursts, readonly startup configuration, and clock rewind. `cases.csv`, `mutations.csv`, source hashes, and the saved proof identify the exact evidence.

![Observed recovery](figures/recovery.svg)

Ten decision endpoints spaced 0.5 seconds apart span 4.5 seconds. The separate timer prevents release at that point. A later command tick may release once five seconds have elapsed if the latest decision remains fresh.

![Completed-goal memory](figures/goal_memory.svg)

Estimated drift and intervening holds preserve completion. A new valid revision can authorize a new mission only after the other navigation conditions are satisfied.

## Proof boundary and course connection

The theorem concerns the finite permission policy under validated-input and time-abstraction assumptions. It does not prove middleware delivery, timer scheduling, detector effectiveness, perception correctness, continuous collision freedom, or physical braking. Clock epochs are locally inferred; the current messages cannot intrinsically identify numerically coincident old messages after rewind. Historical M3/M4 traces are not retroactively covered by this proof.

This material supplies the architecture/assumptions, formal method, implementation, and quantitative analysis requested by the linked course report template. The provided course materials identify Fall 2025; current-year deadlines and grading are not inferred. `docs/verification/supervisor_course_alignment.md` records the primary sources and scope. Codex assistance was used for implementation/documentation; correctness is supported by the explicit checks and recorded evidence rather than by that assistance.
