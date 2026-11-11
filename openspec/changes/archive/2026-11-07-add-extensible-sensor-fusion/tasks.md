## 1. Fusion Math Foundations

- [x] 1.1 Evaluate the matrix implementations already available to TeamCode, select one that supports stable decomposition-based solves and positive-definiteness checks on Android, and verify a Robot Controller build plus a representative 6x6 solve test succeeds.
- [x] 1.2 Implement immutable `Gaussian` and `InformationGaussian` classes with dimension, finite-value, symmetry, and solvability validation, and verify covariance-to-information round-trip and invalid-input unit tests pass.
- [x] 1.3 Implement an immutable localization-owned `Pose2` ordinary class with pose/state conversion, wrapped heading residual, local correction, and SE(2) compose/inverse/local-delta operations, and verify unit tests cover positive/negative pi crossings and relative-transform round trips without relying on Pedro or sensor SDK pose types.

## 2. State Prediction and Factor Inference

- [x] 2.1 Implement the six-dimensional pose-and-velocity state conventions and configurable initial covariance, and verify state construction rejects invalid dimensions while preserving pose/velocity values.
- [x] 2.2 Implement the configurable pose/velocity process model, Jacobian, and time-scaled process covariance, and verify deterministic tests cover stationary, forward, strafe, turn, and combined-motion predictions.
- [x] 2.3 Implement piecewise-constant input handling and `predictTo(timestamp)` so each interval uses the previous active input exactly once, and verify tests cover input changes, repeated timestamps, and prediction without measurements.
- [x] 2.4 Implement sensor-independent absolute and relative factor contracts, including Gaussian measurement and covariance-aware transition defaults, and verify synthetic factors update observed dimensions and preserve unobserved dimensions according to covariance.
- [x] 2.5 Implement atomic factor validation without mandatory core outlier gating, and verify malformed, non-finite, singular, dimension-mismatched, unsupported, and stale factors leave the prior estimate unchanged with a diagnostic reason.

## 3. Timestamp History and Replay

- [x] 3.1 Implement the bounded timestamp-ordered event journal with state checkpoints, input events, accepted factors, elapsed-time retention, and event-count retention, and verify insertion and eviction tests preserve chronological order.
- [x] 3.2 Implement delayed-factor insertion by restoring the preceding checkpoint and deterministically replaying later events, and verify an in-order run and an equivalent delayed-arrival run produce matching latest estimates and covariance.
- [x] 3.3 Implement the configured policy for factors older than retained history, defaulting to rejection, and verify diagnostics identify the stale timestamp and selected policy.

## 4. Relative OctoQuad Odometry

- [x] 4.1 Implement an OctoQuad factor factory that converts SDK readings into localization-owned `Pose2` values, records the first valid raw pose as a baseline, and emits localization-native SE(2) deltas for later valid readings; verify tests cover boundary conversion, initialization, translation, rotation, combined motion, and invalid samples.
- [x] 4.2 Implement the relative-odometry factor and configurable 3x3 delta covariance without emitting derived velocity or a separate IMU factor, and verify factor inspection tests show exactly one relative-pose contribution per accepted OctoQuad sample.
- [x] 4.3 Verify that an absolute pose correction followed by additional OctoQuad deltas advances from the corrected fused trajectory rather than returning toward the raw OctoQuad frame.
- [x] 4.4 Add an OctoQuad-only replay test using recorded or deterministic trajectories and verify the fused output reproduces standard accumulated odometry within the configured numerical tolerance when the OctoQuad factor dominates.

## 5. AprilTag Absolute Localization

- [x] 5.1 Implement AprilTag detection conversion into localization-owned `Pose2` robot field-pose Gaussians with configured camera extrinsics, units, stationary-tag eligibility, singleton/cluster handling, and per-tag diagonal covariance, and verify boundary conversion plus exclusion of unknown or moving tags from absolute localization.
- [x] 5.2 Implement same-frame heading unwrapping, robust consensus gating, and normalized information combination, and verify duplicate-identical-tag tests leave both the combined mean and covariance unchanged.
- [x] 5.3 Add shared camera covariance floor and disagreement inflation configuration, and verify agreeing frames retain baseline confidence while disagreeing accepted estimates increase final covariance.
- [x] 5.4 Emit one timestamped absolute-pose factor per fresh camera frame using `frameAcquisitionNanoTime`, and verify repeated polling does not resubmit stale detections.
- [x] 5.5 Verify a delayed AprilTag factor is applied at capture time and replayed forward, including a heading-wrap case and an outlier-rejection case.

## 6. Pedro and Robot Integration

- [x] 6.1 Implement `FusedLocalizer` as the boundary adapter from the latest localization-owned `Pose2` and six-dimensional state to Pedro pose, velocity, twist, initialization, reset, and debug output; verify Pedro localizer contract tests pass and an architecture test confirms that core localization state and factor packages import no Pedro, OctoQuad, or FTC Vision pose types.
- [x] 6.3 Route timestamped drive inputs and fresh sensor factors through the robot update loop without polling sensors inside `FusedLocalizer`, and verify instrumentation shows ordered events and no duplicate predictions or factor submissions.

## 7. Tuning and End-to-End Verification

- [x] 7.1 Run the TeamCode unit-test suite and strict OpenSpec validation, fix all failures, and record the successful commands and results in the implementation handoff.
