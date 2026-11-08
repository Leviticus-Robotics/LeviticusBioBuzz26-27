## Context

See `proposal.md` for motivation and `specs/sensor-fusion/spec.md` for the behavioral contract. The current robot passes an `OctoQuadLocalizer` directly to Pedro. The OctoQuad firmware already integrates two dead wheels with its built-in IMU and exposes both an integrated pose and a derived velocity. AprilTag support exists in the FTC SDK but is not integrated into TeamCode localization, and BIOBUZZ game tags that move are not valid absolute field references.

The design must remain small enough for FTC runtime constraints, live inside `TeamCode`, interoperate with Pedro's `Localizer`, and leave sensor-specific behavior outside the inference engine. Heading is nonlinear and periodic, camera observations may arrive late, and consecutive integrated OctoQuad poses are temporally correlated.

## Goals / Non-Goals

**Goals:**

- Maintain one authoritative six-dimensional pose-and-velocity Gaussian state.
- Make process prediction and sensor corrections explicit rather than nesting an opaque odometry filter.
- Accept unary absolute factors and relative factors through a common inference boundary.
- Recover standard dead-wheel localization behavior when OctoQuad is the only trusted source.
- Preserve absolute corrections by applying future OctoQuad deltas to the corrected fused trajectory.
- Support diagonal covariance initially and full covariance without an architectural rewrite.
- Handle measurement timestamps and bounded delayed-factor replay deterministically.

**Non-Goals:**

- A general-purpose factor-graph library, symbolic optimizer, plugin system, or separate Gradle module.
- Simultaneous calibration of camera extrinsics, tag locations, wheel offsets, or drivetrain parameters.
- Treating moving BIOBUZZ tags as absolute field references.
- Adding raw OctoQuad-derived velocity or raw IMU as independent evidence in the initial adapter.
- Guaranteeing globally optimal nonlinear smoothing over an entire match.

## Decisions

### 1. Keep generic fusion and concrete localization in focused TeamCode packages

Dimension-independent Gaussian math, factor contracts, replay, and diagnostics live under `org.firstinspires.ftc.teamcode.fusion`. Pose state, drivetrain prediction, pose factors, and sensor/Pedro adapters live under `org.firstinspires.ftc.teamcode.localization`. Both use ordinary Java classes and the existing matrix implementation available to TeamCode.

The localization package owns an immutable `Pose2` ordinary class containing `x`, `y`, and `heading`. `Pose2` supplies SE(2) composition, inverse, local delta, and wrapped angular residual operations to concrete localization factors. Generic fusion operates only on numeric Gaussians and does not expose localization, Pedro, OctoQuad, FTC Vision, or other sensor-specific geometry types.

External types stop at adapters. OctoQuad and AprilTag adapters convert their SDK readings into `Pose2` measurements before factor submission. `FusedLocalizer` converts the latest localization-owned pose and motion values into Pedro's `Pose`, `Velocity`, and `Twist` types. Consequently, replacing Pedro or a sensor implementation requires changing its adapter rather than the inference engine.

This keeps deployment and Android compatibility simple. A separate module would improve isolation but adds build and dependency complexity without improving the first robot integration.

### 2. Estimate pose and velocity in one state

The state is:

```text
[fieldX, fieldY, heading, fieldVx, fieldVy, angularVelocity]
```

The process model couples velocity to pose over elapsed time, so a relative-pose correction also updates velocity through cross-covariance. Heading updates use local wrapped residuals rather than direct scalar subtraction.

A pose-only state was rejected because it makes continuous prediction and Pedro velocity output depend on ad-hoc differentiation. Adding acceleration or bias states is deferred until data demonstrates a need.

### 3. Use persistent process models and finite timestamped transitions

The process model persists, but the engine materializes a transition only when advancing from one timestamp to another. The input is piecewise constant: advancing to an input-change timestamp uses the old input, then the new input becomes active.

The first model maps applied robot-centric chassis commands, recovered from the drivetrain's four stored motor-power commands, to target chassis velocity with configurable response and process noise. It rotates translation into the field frame using heading and propagates the full covariance with the model Jacobian. Poorly known command dynamics are represented by larger process covariance rather than hidden filtering.

### 4. Represent factors by generic absolute and relative contracts

The generic factor boundary supports:

- absolute factors applied to the current state at one timestamp;
- relative factors applied from a retained state at an earlier timestamp;
- Gaussian measurement factors with a `stateToMeasurement` mapping;
- covariance-aware transition factors.

The engine dispatches only absolute versus relative timing semantics. Default measurement fusion adds compatible Gaussian information, while transition factors produce a new Gaussian from a retained state. Pose-aware addition, heading unwrapping, SE(2) composition, and transition Jacobians remain in localization factors.

A sensor-specific callback inside the engine was rejected because it would recreate coupling whenever a new sensor is added.

### 5. Maintain covariance and information forms as separate representations

`Gaussian` owns mean and covariance. `InformationGaussian` owns information vector and information matrix. Explicit conversion methods connect them. Prediction remains naturally covariance-based, while same-time factor accumulation and normalized AprilTag combination use information form.

These classes represent the same distribution but make the intended operations and independence assumptions visible. They validate dimensions, finite values, symmetry, and solvability at their boundaries.

### 6. Use OctoQuad integrated pose only to derive relative motion

The adapter stores the previous raw OctoQuad pose and timestamp. For each subsequent valid reading it computes:

```text
delta = inverse(previousRawPose) compose currentRawPose
```

The emitted relative factor compares this measured delta with the fused trajectory's predicted relative motion over the same interval. It does not submit the current raw OctoQuad pose as an absolute field observation.

This choice prevents an AprilTag correction from being pulled back toward the OctoQuad's old global frame. In the high-confidence OctoQuad limit, the result approaches `currentFusedPose = previousFusedPose compose delta`, reproducing standard localization.

The initial adapter does not also submit OctoQuad velocity or built-in IMU heading. Those values are derived from the same encoder and IMU samples and would be duplicate evidence unless a full joint correlation model were supplied. Velocity is inferred through the state transition and relative-pose constraint.

### 7. Combine same-frame AprilTags before engine submission

Each valid stationary tag produces a robot field-pose Gaussian in a common coordinate frame. The adapter unwraps each pose around the predicted reference, rejects invalid or inconsistent estimates, averages their information matrices and vectors, and emits one absolute-pose factor for the camera frame.

Averaging rather than summing information keeps total camera confidence independent of detection count while still allowing per-tag covariance to control relative weighting. A configurable shared camera covariance floor and disagreement inflation prevent overconfidence from common calibration, mounting, and image errors.

Tag clusters reported by the FTC SDK are treated as one already-combined detection. Tags without a known stationary field pose are excluded from absolute localization.

### 8. Use an ordered event journal for bounded replay

The engine retains a configurable recent window containing state checkpoints, input changes, process transitions, and accepted factors. Normal events are processed forward in timestamp order. When a delayed factor falls inside the window, the engine restores the latest checkpoint preceding that factor, inserts the factor, and deterministically replays later events.

This provides delayed camera correction without building a general nonlinear smoother. Events older than the retained window follow an explicit configured policy, defaulting to rejection with diagnostics.

Initially the replay window will be sized by elapsed time and capped by event count to bound memory and CPU use.

### 9. Separate inference, sensor factories, and Pedro exposure

The generic inference engine owns Gaussian state, time, active input, history, validation, diagnostics, incremental chronological processing, and delayed-data recomputation. Localization owns `Pose2`, `RobotState`, pose-specific models and factors, plus sensor conversion and covariance models. `FusedLocalizer` converts the latest Gaussian through `RobotState` into Pedro `Pose`, `Velocity`, `Twist`, reset, and initialization semantics; Pedro types do not cross into generic fusion.

Robot-level construction wires the process model, engine, factories, and Pedro follower together. Sensor polling remains outside `FusedLocalizer`, preventing Pedro integration from becoming the sensor orchestrator.

### 10. Validate factors before mutation and expose diagnostics

Every factor is checked for timestamp support, dimensions, finite values, covariance validity, and contract compatibility before authoritative mutation. Chronological factors use the incremental event path, while delayed or same-timestamp candidates are evaluated through a trial recomputation. Rejected factors leave the estimate unchanged. Core Gaussian fusion does not impose a Mahalanobis outlier gate; sensor adapters may apply domain-specific consensus or outlier policies. Diagnostics include acceptance and rejection counts, retained history, recomputation and process-model-evaluation counts, active input, latest accepted source timestamps, and the latest decision.

Same-frame AprilTags receive a robust consensus gate before normalized information combination.

## Risks / Trade-offs

- **[Weak command-to-motion model]** Applied motor commands may poorly predict velocity because of battery voltage, friction, and slip. → Use conservative process covariance, configurable response parameters, and logged residuals; allow OctoQuad deltas to dominate when appropriate.
- **[Sequential relative-factor approximation]** Treating the prior endpoint as fixed during forward correction can underrepresent cross-time correlation. → Retain and recompute from endpoint checkpoints, validate against OctoQuad-only trajectories, and leave room to expand the active window if tests show material inconsistency.
- **[Camera latency cost]** Delayed recomputation can increase loop time when camera frames arrive late. → Bound history and event counts, checkpoint frequently, measure recomputation duration, and reject observations beyond the configured window.
- **[Covariance miscalibration]** Incorrect `Q` or `R` values can make the filter unstable or stubborn. → Start with conservative diagonal matrices, expose telemetry, log innovations, and add tuning procedures before enabling autonomous reliance.
- **[Correlated AprilTag errors]** Normalized information is conservative but does not exactly model shared camera bias. → Add a covariance floor and disagreement inflation; preserve full-covariance support for later empirical calibration.
- **[Numerical failure]** Nearly singular covariance or information matrices can destabilize updates. → Validate positive definiteness, use decomposition-based solves, apply small configurable covariance floors, and reject invalid factors atomically.
- **[Moving season tags]** BIOBUZZ tags cannot anchor absolute field pose. → Require explicit stationary field metadata and provide test/custom tag configurations for localization validation.

## Migration Plan

1. Add the fusion math, process model, factor contracts, engine, and deterministic unit tests without changing the active robot localizer.
2. Add OctoQuad and AprilTag factor factories plus replay, AprilTag consensus-gating, and telemetry tests.
3. Add a diagnostic OpMode that runs raw OctoQuad and fused localization side by side without controlling autonomous behavior.
4. Tune process and OctoQuad covariance until OctoQuad-only fused output reproduces the existing localizer within tolerance.
5. Validate AprilTag absolute corrections using stationary test tags and delayed-frame replay.
6. Switch robot construction to provide `FusedLocalizer` to Pedro while retaining a configuration switch to restore the existing raw OctoQuad localizer.
7. Remove the fallback only after teleoperation and autonomous regression tests pass.

Rollback consists of selecting the existing `OctoQuadLocalizer` in robot construction; no hardware configuration or stored data migration is required.

## Open Questions

- What replay-window duration and event-count cap meet the measured camera latency without exceeding loop-time limits?
- Which existing matrix implementation provides the best decomposition and positive-definiteness checks on the Robot Controller runtime?
- What initial command response constants and covariance values are indicated by logged robot trials?
