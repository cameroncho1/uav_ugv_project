# W10 course alignment and source provenance

[Supervisor specification](supervisor.md) | [Report template](https://docs.google.com/document/d/1lMb6jLL3d95x2ixvJQQkNiAKz2m37j4iMq9wWyq-SWA/edit)

The supplied local `CourseIntro.pptx` contains 20 slides and identifies **Fall 2025 CS 513**. The instructor's published [2025 project page](https://jdeshmukh.github.io/teaching/cs513-autocps-fall-2025/projects.html) links to the same report template used by this repository. The linked home, project, policy, schedule, and report-template documents were read in full. The proposal/resources/ideas documents were independently read during scope review.

Earlier repository links assumed a 2026 course site. Those URLs were not accessible in this research. The requirements below are attributed to the supplied 2025 materials; no 2026 deadline or grading rule is inferred from them. The 2025 introduction reports a 25% project component, split between a 10% presentation and 15% report/demo. It does not provide a feature-specific scoring rubric. The retrieved project resources do not mandate a particular verification tool.

## Deliverable mapping

| Course expectation | W10 material | Scope |
| --- | --- | --- |
| Model the application and environment | Supervisor state, memory, validated-input assumptions, clock abstraction, and mode diagram | Discrete software decisions; physical plant remains in the simulator |
| Write formal checkable requirements | Temporal safety/next-step property table with explicit atomic propositions | Includes completion and recovery memory, not only five mode names |
| Apply verification/testing and formal reasoning | Reachable-state/product-monitor exploration, mutation counterexamples, compiled C++ conformance, and ROS integration tests | Proof for the declared finite abstraction; observed timing remains experimental |
| Demonstrate a working autonomous stack | Actual controller fixture plus separate headless assisted navigation and guard/watchdog checks | Controlled detector inputs do not establish detector accuracy |
| Explain assumptions and architecture | Report section 3 material: components, decision boundary, interfaces, and assumptions | Explicit missing producer and transport/clock assumptions |
| Explain methods and implementation | Sections 4-5 material: shared policy, independent properties, C++ generation/conformance, ROS wiring | Identifies the added persistent supervisor and test method |
| Present quantitative results and analysis | Section 6 material: graph/transition counts, conformance coverage, mutation witnesses, recovery trace, and measured trials | Failed development pilots remain separate; no continuous-safety claim |

The report template has seven sections: introduction; background/related work; solution overview; methods/algorithms; implementation; experimental results with analysis; and conclusion. The supplied introduction specifies a ten-page report. W10 supplies a concise insertion for sections 3-6 rather than another complete project report.

## Requirement traceability

| Project requirement | Supervisor contribution | Additional evidence needed |
| --- | --- | --- |
| R1 footprint clearance | Navigation requires a validated clear route | Independent geometry/truth and recorded clearance; no continuous-safety theorem |
| R2 goal arrival | Completion/revision memory prevents resuming the same completed mission | Physical tolerance, quiet dwell, and arrival deadline remain trace checks |
| R3 guard response | Invalid pose cannot authorize navigation; zero is requested every hold tick | Actual command scheduling and sustained hold are measured through R3 |
| R4 actuator expiry | Existing independent watchdog remains in the command path | Actuator-output and physical-settling traces after command loss |
| R5 hazard response | Invalid routes hold; new routes must match the active goal | Actual hazard observation/response producer and timing remain unavailable |
| R6 localization accuracy | Full pose validity gates navigation | Independent acquisition-matched truth; no estimator accuracy claim from the model |
| R7/R8 detector quality | Versioned, validated detector interface and fail-safe enabled-input handling | An online producer and held-out labeled evaluation remain separate work |

## Formal reasoning and limits

The assurance argument combines three distinct evidence types:

1. Exhaustive finite-state safety checking under validated-input and timer-abstraction assumptions.
2. Conformance of the compiled C++ transition over every explored reachable context/input.
3. ROS command-gating tests and separate measured simulator trials for integration and timing.

These do not prove detector effectiveness, continuous collision freedom, physical braking, or unconditional eventual arrival. The controller's estimated-threshold completion latch does not replace R2's physical and quiet-dwell checks. The model, properties, negative controls, and saved witnesses make the correctness argument inspectable.

## Course policy and tooling acknowledgment

The retrieved [course policy](https://jdeshmukh.github.io/teaching/cs513-autocps-fall-2025/policies.html) permits generative tools with citation/disclosure and requires an explanation of correctness. Codex assistance was used for implementation and documentation. Correctness is supported by the independently scoped requirements, explicit property checks, negative controls, C++ conformance, and recorded runtime tests. Tool assistance itself is not verification evidence.

Source URLs, hashes, material year, and retrieval metadata accompany the result artifacts. Current course submission requirements should be taken from the applicable published course instructions; the historical source dates are preserved here.
