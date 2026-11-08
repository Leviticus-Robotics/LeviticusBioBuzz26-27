## 1. Generic Fusion Core

- [x] 1.1 Move `Gaussian`, `InformationGaussian`, `MatrixOps`, and `GaussianFusion` into `fusion.core`, remove pose-specific helpers from the generic math API, and verify covariance/information round-trip, validation, and arbitrary-dimension fusion tests pass.
- [x] 1.2 Verify `fusion.core` compiles without imports from localization, Pedro, FTC hardware or vision, AprilTag, or OctoQuad packages by adding and passing an architecture dependency test.

## 2. Generic Factor and Engine Contracts

- [x] 2.1 Implement the `Factor`, `AbsoluteFactor`, `RelativeFactor`, and `MeasurementFactor` hierarchy under `fusion.factor`, including default Gaussian measurement application, direct relative transition application, and unit coverage for each dispatch path and incompatible dimensions.
- [x] 2.2 Move `ProcessModel<I>`, `FactorApplicationResult`, and `FusionDiagnostics<I>` under `fusion.engine`, generalize the timestamped inference engine as `FusionEngine<I>`, and verify its constructor, state accessor, checkpoints, and event journal use only `Gaussian` plus the generic input and factor contracts.
- [x] 2.3 Update factor handling so chronological events use an incremental path while delayed or same-time events deterministically recompute bounded history, dispatch only absolute versus relative semantics, preserve input/relative/absolute/tick ordering, and atomically reject unsupported or stale factors; verify in-order and delayed-arrival runs produce matching latest means and covariances.
- [x] 2.4 Add a non-pose two-dimensional position/velocity subsystem test with its own input type, process model, scalar measurement, transition, delayed factor, and bounded history, and verify it uses no localization classes.

## 3. Localization as a Fusion Consumer

- [x] 3.1 Keep `Pose2`, `RobotState`, and `ControlInput` in `localization.core`, update `RobotState` conversion boundaries to consume and expose generic Gaussians, and verify six-dimensional pose/velocity and heading-wrap tests remain unchanged.
- [x] 3.2 Keep `PoseVelocityProcessModel` under localization while implementing the generic process-model contract, keep `ControlInput` limited to applied robot-centric forward/strafe/turn commands, and verify stationary, translation, strafe, turn, combined prediction, and applied-mecanum-input tests pass.
- [x] 3.3 Refactor `AbsolutePoseFactor` to implement the generic measurement contract while retaining heading unwrapping and its pose `stateToMeasurement` mapping, and verify pose fusion updates correlated velocity without introducing pose logic into `fusion`.
- [x] 3.4 Refactor `RelativePoseFactor` to implement `RelativeFactor.apply` directly while retaining SE(2) composition, delta-derived velocity, and covariance propagation, and verify OctoQuad-only motion plus absolute-correction-followed-by-odometry tests pass.
- [x] 3.5 Update `LocalizationRuntime`, `LocalizationConfig`, AprilTag and OctoQuad processors, `FusionMecanumDrivetrain`, diagnostics, constants, and tests to use `FusionEngine<ControlInput>` and the moved fusion APIs; verify pose reset clears relative baselines while preserving active input, Pedro prediction records inspected applied motor commands, and robot-facing pose, velocity, covariance, telemetry, and fallback behavior are preserved.

## 4. Cleanup and Verification

- [x] 4.1 Remove superseded generic classes and factor interfaces from localization, update every repository import, and verify searches find no duplicate generic API or forbidden reverse dependency from fusion to localization.
- [x] 4.2 Run `./gradlew :TeamCode:testDebugUnitTest :TeamCode:compileDebugJavaWithJavac` and verify the complete TeamCode unit suite and Android Java compilation succeed.
- [x] 4.3 Reconcile the archived `add-extensible-sensor-fusion` planning artifacts and synchronized main spec with the generic package boundary and current no-core-Mahalanobis design, then run strict OpenSpec validation and verify no specification or design contradictions remain.
