## Why

Robot localization is currently centered on a single OctoQuad localizer, which makes it difficult to combine command-model predictions, relative odometry, AprilTag field observations, and future sensors without coupling fusion logic to specific hardware. A shared probabilistic inference layer will preserve each source's uncertainty and timing while allowing new sensor factors to be added without redesigning localization.

## What Changes

- Add a dimension-independent Gaussian inference engine in `TeamCode`, with localization-owned pose and robot-state types adapting that generic engine without depending on Pedro or sensor SDK geometry types.
- Model drivetrain commands as timestamped process transitions that predict the robot state and covariance over finite time intervals.
- Convert consecutive OctoQuad readings into relative-pose factors so odometry follows the best fused pose instead of anchoring the estimate to the OctoQuad's drifting global frame.
- Convert AprilTag detections into absolute field-pose factors, combining correlated same-frame detections with normalized information before submission.
- Preserve factor timestamps and a bounded recent state history so delayed observations can be applied at their capture time and propagated forward.
- Expose the latest fused pose and velocity through Pedro's `Localizer` interface.
- Add validation, outlier rejection, configuration, telemetry, and tests for prediction, factor fusion, heading wrapping, timing, and sensor-only degradation behavior.

## Capabilities

### New Capabilities

- `sensor-fusion`: Defines timestamped Gaussian state estimation, process transitions, relative odometry factors, absolute pose factors, delayed observation handling, and Pedro localization output.

### Modified Capabilities

None.

## Impact

- Adds reusable `org.firstinspires.ftc.teamcode.fusion` packages plus concrete localization models, factors, and sensor adapters under `org.firstinspires.ftc.teamcode.localization`.
- Changes robot construction so Pedro consumes the fused localizer instead of a raw `OctoQuadLocalizer`.
- Uses the existing OctoQuad, AprilTag, Pedro, and matrix dependencies; no separate Gradle module or plugin framework is introduced.
- Requires empirical tuning of process, OctoQuad delta, and AprilTag covariance models.
