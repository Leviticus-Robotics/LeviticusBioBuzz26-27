# Architecture overview

## What problem the system solves

No single localization source provides everything the robot needs:

- Drive commands are available continuously, but commanded movement is not actual movement.
- OctoQuad dead-wheel odometry measures short-term motion accurately, but its integrated global pose drifts.
- AprilTags provide occasional global field references, but arrive slowly and after image-processing latency.

The estimator combines these sources according to their uncertainty. Commands predict plausible motion, OctoQuad constrains measured local displacement, and AprilTags anchor the trajectory globally.

The output is not just a pose. It is a probability distribution over:

\[
X_k = [x, y, \theta, v_x, v_y, \omega]^T.
\]

The mean is the best current estimate. The covariance describes its uncertainty and correlations.

## The central loop

For consecutive loop timestamps \(t_{k-1}\) and \(t_k\):

```text
previous posterior X(k-1)
          │
          ├── predictive factor + applied input
          ▼
joint Gaussian [X(k-1), X(k)]
          │
          ├── relative factors, such as OctoQuad delta
          ├── absolute factors, such as AprilTag pose
          ▼
corrected joint Gaussian
          │
          └── marginalize X(k-1)
          ▼
current posterior X(k)
```

The temporary joint state is essential. A relative observation constrains the relationship between two states, so the estimator must retain their cross-covariance while applying it.

## Architectural layers

### Generic fusion

The `fusion` package understands only:

- Gaussian states.
- Timestamped factors.
- Residuals, Jacobians, and covariance.
- A previous-state prior and an ordinary batch of absolute and relative factors.
- Tick history and chronological replay.

It does not know what the state coordinates mean. It can be used with a state other than robot pose as long as every factor agrees on the dimension and coordinate convention.

### Localization model

The `localization` package assigns physical meaning to the state:

- `RobotState` defines `[x, y, heading, vx, vy, omega]`.
- `Pose2` defines planar composition, inversion, local deltas, and angle wrapping.
- `PoseVelocityProcessModel` predicts motion from applied drivetrain commands.
- `AbsolutePoseFactor`, `PoseDeltaFactor`, and `MotionDeltaFactor` define localization likelihoods.

### Sensor boundaries

Sensor adapters own hardware- and SDK-specific conversion:

- `OctoQuadFusionSource` polls Pedro's raw OctoQuad localizer.
- `OctoQuadMotionAdapter` converts consecutive integrated poses into local deltas.
- `AprilTagFrameProcessor` converts eligible detections into one absolute pose factor per camera frame.

The fusion engine never polls hardware and never imports either sensor SDK.

### Consumer boundaries

`FusedLocalizer` implements Pedro's `Localizer` interface. Pedro sees only the fused pose and velocity. `FusionMecanumDrivetrain` wraps the real drivetrain and records the motor powers that were actually applied so the process model receives the correct input.

## What kind of estimator this is

The system is best described as a factor-oriented, stochastic-cloning-style EKF with bounded out-of-sequence replay.

It shares ideas with a factor graph, but it is not a full fixed-lag smoother:

- It temporarily estimates two consecutive states jointly.
- It marginalizes the old state after each tick.
- It caches historical posteriors for delayed replay.
- It does not simultaneously optimize every retained historical state.

This keeps runtime and implementation size appropriate for an FTC robot while preserving the correlations required by consecutive relative measurements.

## Responsibility rules

The design follows several important boundaries:

- A sensor adapter describes what was observed; it does not modify the estimate.
- A factor owns the mathematics of its observation: residual, Jacobian, covariance, and timestamps.
- The engine owns generic inference, scheduling, replay, and marginalization.
- The Pedro adapter owns loop-time polling and presentation to consumers.
- The composition root decides which concrete factor sources are active.

These rules allow a sensor, process model, or consumer to be substituted without teaching the fusion engine about it.

## Important assumptions

The current implementation assumes:

- Relative factors connect consecutive retained ticks.
- One predictive factor establishes every advancing tick.
- Factors in one batch are conditionally independent given the state unless their covariance already models shared error.
- All state and sensor timestamps use the same monotonic nanosecond clock domain.
- Covariances are symmetric positive definite.
- Nonlinear factors are linearized once per evaluation; there is no iterative optimizer inside a tick.
- Pose correction uses additive `[x, y, heading]` coordinates with angle wrapping, not a full Lie-manifold retraction.

The later documents explain the consequences of each assumption.

