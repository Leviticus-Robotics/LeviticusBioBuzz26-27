## Context

See `proposal.md` for motivation and `specs/generic-gaussian-fusion/spec.md` for the behavioral contract. The implementation before this change already separated Gaussian math, factor types, a timestamped inference engine, localization state wrappers, and sensor adapters into Java packages, but all of them remained below `localization`. The engine constructor and state accessor used `RobotState`, and its factor dispatch was originally built around pose-specific factors. This change corrects that ownership without creating a separate Gradle module or a broad factor-graph framework.

The code runs on the FTC Robot Controller, uses Java 8-compatible source, and must preserve bounded CPU and memory behavior. Localization integration, delayed camera replay, relative OctoQuad transition, applied drivetrain-command handling, explicit state-reset behavior, diagnostics, and the fallback localizer must continue working while imports and package names change.

## Goals / Non-Goals

**Goals:**

- Make the fusion math, factor contracts, process-model contract, inference engine, result types, and diagnostics independent of localization and robot SDK types.
- Preserve a small API centered on `Gaussian` rather than adding a generic Java state-type hierarchy.
- Keep absolute-versus-relative timing semantics distinct while providing reusable linear-Gaussian measurement behavior only where it adds implementation value.
- Keep all heading wrapping, SE(2) composition, pose/velocity indices, drivetrain inputs, and sensor conversions in localization.
- Prove reuse with a non-pose subsystem test and enforce package direction with architecture tests.

**Non-Goals:**

- A nonlinear factor-graph optimizer, symbolic residual system, whole-match smoother, dependency-injection framework, or plugin system.
- A separate Gradle module or publication of the fusion framework as a library.
- Generalizing FTC hardware, Pedro, or localization adapters beyond the package-boundary changes needed here.
- Changing the six-dimensional localization state, process equations, sensor covariance models, or robot-facing behavior.
- Preserving the old internal Java package names through compatibility shims.

## Decisions

### 1. Organize generic code under focused fusion subpackages

The generic layer will use this dependency direction:

```text
fusion.core
  Gaussian, InformationGaussian, MatrixOps, GaussianFusion

fusion.factor
  Factor, AbsoluteFactor, RelativeFactor, MeasurementFactor

fusion.engine
  FusionEngine<I>, ProcessModel<I>, FactorApplicationResult, FusionDiagnostics<I>

localization
  depends on fusion; fusion never depends on localization
```

Localization keeps its existing focused subpackages for core pose/state types, concrete models and factors, sensor adapters, Pedro adapters, and runtime wiring. A single flat `fusion` package was considered, but preserving `core`, `factor`, and `engine` groupings makes ownership clear without adding modules or abstraction layers.

### 2. Keep the generic state representation as `Gaussian`

`FusionEngine<I>` will accept an initial `Gaussian`, return a `Gaussian`, and store Gaussian checkpoints. It will not accept or return `RobotState`. The input type remains generic because domains need meaningful input objects, while the state does not gain a Java type parameter: its dimension and semantics already live in its mean and covariance.

Localization converts at its boundary:

```text
RobotState -> Gaussian -> FusionEngine -> Gaussian -> RobotState
```

Making the engine generic over both an input type and a domain state wrapper was rejected because it would require conversion/factory interfaces throughout replay and factors while adding no mathematical capability.

### 3. Use a four-interface factor hierarchy with behavior at the narrowest useful level

The hierarchy will be:

```text
Factor
├── AbsoluteFactor
│   └── MeasurementFactor
└── RelativeFactor
```

- `Factor` owns source and timestamp metadata only.
- `AbsoluteFactor` applies to the Gaussian at its own timestamp.
- `RelativeFactor` alone owns `fromTimestampNanos` and applies using the retained Gaussian at that earlier timestamp.
- `MeasurementFactor` supplies a Gaussian measurement and `stateToMeasurement` mapping and inherits a default absolute application through `GaussianFusion`.
- Concrete relative factors implement their transition directly through `RelativeFactor.apply`; a separate transition alias would add no behavior or timing semantics.

The engine dispatches only on `AbsoluteFactor` and `RelativeFactor`. It does not know that a particular absolute factor is a linear measurement or that a particular relative factor is a transition. This permits future nonlinear or specialized factors without adding sensor branches to the engine while keeping the standard Gaussian cases concise.

Collapsing the hierarchy to only measurement and relative interfaces was considered, but it would conflate timing semantics with mathematical implementation. Conversely, adding transition aliases, residual, optimizer, and manifold interfaces was rejected as unnecessary for the current sequential Gaussian engine.

### 4. Keep angular and domain topology outside generic fusion

Generic fusion treats means as Euclidean vectors. It does not know that a component is an angle or that a pose belongs to SE(2). `AbsolutePoseFactor` unwraps measured heading around the predicted heading before constructing its measurement Gaussian. `RelativePoseFactor` owns pose composition, angular residuals, delta-to-velocity inference, and covariance Jacobians. `RobotState` normalizes headings when presenting localization state.

Adding circular-dimension configuration to the generic Gaussian classes was rejected because it would embed one domain's topology into every consumer. A future subsystem with non-Euclidean state can provide specialized factors at the absolute or relative contract boundary.

### 5. Preserve timestamp and replay behavior in the generic engine

Timestamped inputs, factor events, bounded checkpoints, deterministic ordering, and delayed-data recomputation are useful beyond localization, so they move with the engine. Chronological events use an incremental fast path; only delayed or same-timestamp events recompute retained history to restore deterministic ordering. Event ordering remains:

1. input changes;
2. relative factors;
3. absolute factors;
4. output-only prediction ticks.

The engine predicts elapsed intervals through `ProcessModel<I>` and applies factor interfaces without inspecting domain content. Relative factors retrieve the retained state at their declared start time. Unsupported stale factors are rejected atomically. Immutable `Gaussian` instances are retained and passed directly instead of being defensively reconstructed; their own accessors remain defensive.

`FactorApplicationResult` stores only `ACCEPTED` or `REJECTED` status plus its reason and exposes `isAccepted()` as the convenience query. Sensor adapters that only establish a baseline or have no new factor do not call the engine. Latest-source diagnostics retain the maximum accepted timestamp per source so delayed arrival cannot move them backward. Diagnostics name full-history work as recomputation and count process-model evaluations directly, rather than calling every chronological update a replay.

`resetState` validates the replacement Gaussian dimension, clears replay events and source timestamps, and preserves the currently active input as the new baseline input. Correcting or reinitializing state therefore does not imply that the consuming domain changed its command.

Separating replay into a localization wrapper was rejected because it would make the reusable engine unable to handle delayed measurements, a common requirement for other sensors and subsystems.

### 6. Keep concrete pose factors in localization

`AbsolutePoseFactor` remains a localization type implementing the generic measurement contract with the pose `stateToMeasurement` mapping. `RelativePoseFactor` implements `RelativeFactor.apply` directly with SE(2) composition and covariance propagation; there is no separate transition interface. `PoseVelocityProcessModel` remains a localization type implementing the generic process-model contract with `ControlInput`.

`AprilTagFrameProcessor` filters, deduplicates, gates, and combines camera frames into absolute pose factors. `OctoQuadFactorFactory.update` retains its previous sample and emits relative pose factors. `AprilTagPoseObservation` remains a semantic immutable value with defensive covariance access. `LocalizationConfig` owns injected covariance and camera-fusion tuning instead of hiding hard-coded matrices inside the runtime.

`FusionMecanumDrivetrain` owns the concrete Pedro `Mecanum`, delegates each command before recording process input, reads the four `DcMotorEx.getPower()` values, and applies inverse mecanum mixing to recover the applied robot-centric forward, strafe, and turn command. `ControlInput` therefore contains only those three robot-centric applied commands; Pedro's boolean drivetrain argument remains the unrelated `manual` flag. This captures Pedro normalization, clipping, and cached-write behavior without pretending the inspection strategy supports arbitrary drivetrains or exposing motor types to generic fusion. Pedro output adapters continue to consume `RobotState`. No SDK or geometry type crosses into fusion.

### 7. Migrate directly and enforce the boundary with tests

The old generic classes will be moved rather than duplicated, and every production/test import will be updated in one change. Compatibility wrappers were rejected because they would preserve the incorrect ownership and create two apparent APIs.

Verification will include:

- covariance/information and Gaussian-fusion math tests under fusion;
- generic engine replay and validation tests using no localization types;
- a non-pose position/velocity subsystem example using a distinct input type and partial measurements;
- an architecture test rejecting localization, Pedro, FTC Vision, OctoQuad, or hardware imports from fusion;
- existing localization, AprilTag, OctoQuad, Pedro, state-reset, applied-mecanum-input, and delayed-recomputation regression tests.

## Risks / Trade-offs

- **[Breaking internal imports]** Existing TeamCode callers will fail until migrated. → Move classes and update all repository callers and tests in one implementation change; do not retain duplicate wrappers.
- **[Over-generalized API]** Abstract factor contracts could grow into a factor-graph framework. → Keep factors limited to applying one retained/current Gaussian and explicitly exclude joint optimization and symbolic residual APIs.
- **[Hidden localization dependency]** A convenience method or test may reintroduce `RobotState` or heading assumptions into fusion. → Add source-level architecture tests and a non-pose integration test.
- **[Behavior drift during package extraction]** Constructor or recomputation changes could alter localization output. → Run deterministic regression tests for pose, velocity, covariance, event ordering, delayed recomputation, state reset, and applied mecanum-command recovery.
- **[Archived OpenSpec history]** The earlier sensor-fusion change described some inference responsibilities as localization-owned. → Treat this change as the final package-boundary authority and keep the archived change plus synchronized main specification reconciled with it.

## Migration Plan

1. Add the generic package structure and factor hierarchy with tests while leaving robot construction unchanged.
2. Move Gaussian math and generic factor/engine types into `fusion`, generalize the engine boundary to `Gaussian`, and update unit tests.
3. Update localization state wrappers, concrete pose factors, process model, runtime, sensor adapters, Pedro adapters, diagnostics, and constants to consume fusion APIs.
4. Remove the superseded generic classes from localization and run architecture checks to confirm the one-way dependency.
5. Run the complete TeamCode unit suite and Android Java compilation, then validate both OpenSpec changes for consistency.

Rollback is a source-level revert of the package migration; there is no persisted-data, hardware, or deployment migration.
