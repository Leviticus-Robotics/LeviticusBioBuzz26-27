## Why

The Gaussian math, factor contracts, and inference engine were originally housed under localization and depended on `RobotState` or pose-specific behavior, preventing reuse for mechanisms such as elevators, arms, and turrets. Extracting a small domain-independent fusion layer establishes the intended dependency boundary before additional sensors and subsystems build against localization-specific APIs.

## What Changes

- Add a reusable `org.firstinspires.ftc.teamcode.fusion` package containing dimension-independent Gaussian representations, information-form fusion, matrix validation, process-model contracts, incremental timestamped inference with bounded delayed-data recomputation, diagnostics, and factor contracts.
- Define a minimal factor hierarchy in which `Factor` provides common metadata, `AbsoluteFactor` applies to one state, `RelativeFactor` applies a transition from an earlier state, and `MeasurementFactor` provides reusable Gaussian measurement fusion with a `stateToMeasurement` mapping.
- Generalize the inference engine into `FusionEngine<I>` so it stores and returns `Gaussian` values without knowing about `Pose2`, `RobotState`, heading indices, Pedro, AprilTags, or OctoQuad.
- Keep `Pose2`, `RobotState`, `ControlInput`, `PoseVelocityProcessModel`, absolute/relative pose factors, sensor adapters, injected localization tuning, runtime wiring, and Pedro integration under `localization` as concrete consumers of the fusion framework.
- Have the Pedro drivetrain adapter delegate commands first, inspect the four stored motor-power commands, and convert them back into the applied robot-centric forward/strafe/turn input used by prediction.
- Represent actual factor-application attempts as accepted or rejected through `FactorApplicationResult`; sensor updates that produce no factor simply do not call the engine.
- Preserve timestamped input handling, bounded delayed-factor recomputation, Gaussian measurement fusion, covariance-aware transitions, and diagnostics while making applied-command capture and state-reset semantics explicit.
- Add architecture and reuse tests proving the fusion package has no localization or robotics dependencies and can estimate a non-pose subsystem state.
- **BREAKING**: Move existing fusion math, engine, process-model interface, factor interfaces, and related diagnostics to the new package and update their Java package names and imports.

## Capabilities

### New Capabilities

- `generic-gaussian-fusion`: Defines a domain-independent Gaussian inference engine, factor hierarchy, prediction contract, incremental timestamped processing with delayed-data recomputation, and dependency boundaries suitable for localization and non-pose subsystems.

### Modified Capabilities

None.

## Impact

- Moves and renames internal TeamCode Java APIs currently under `localization.core`, `localization.engine`, and `localization.factor`; all TeamCode callers and tests must update imports.
- Refactors localization into a consumer of the generic fusion layer while preserving robot-facing pose, velocity, sensor, Pedro, and telemetry behavior. State reset now explicitly preserves the active process input while clearing event history and relative-sensor baselines.
- Reconciles the archived `add-extensible-sensor-fusion` change and synchronized main specification with the final package boundary and no-core-Mahalanobis design.
- Introduces no new Gradle module, external dependency, hardware requirement, or runtime service.
