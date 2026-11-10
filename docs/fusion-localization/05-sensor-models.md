# Sensor models

## OctoQuad: local motion, not an absolute field anchor

The raw OctoQuad/Pedro localizer reports an integrated pose in its own drifting odometry frame. Treating that integrated pose as an absolute field measurement would cause a problem:

1. An AprilTag corrects the fused field pose.
2. The next OctoQuad absolute pose remains in the old drifting frame.
3. OctoQuad pulls the fused estimate back toward that old frame.

The implementation therefore compares consecutive raw poses:

\[
\Delta O_k=O_{k-1}^{-1}\oplus O_k.
\]

`OctoQuadFactorFactory` combines this delta with the OctoQuad-reported velocity in one generic `MotionDeltaFactor` connecting the two sample ticks. Translational velocity is converted from the raw odometry frame to robot-local coordinates so it remains valid after a global AprilTag heading correction. The factor says:

> Whatever the globally corrected pose was previously, the robot moved locally by this measured delta.

After an AprilTag correction, future OctoQuad deltas continue from the corrected fused pose instead of restoring the raw OctoQuad origin.

## OctoQuad polling lifecycle

`OctoQuadFusionSource.poll(previousTimestamp, tickTimestamp)`:

1. Calls the raw Pedro `OctoQuadLocalizer.update()` exactly once.
2. Reads its current pose and velocity from `MotionState`.
3. Converts the pose to localization-owned `Pose2`.
4. Passes the complete motion sample to `OctoQuadFactorFactory` with the loop timestamp.

Construction and reset synchronously establish a pose baseline at the engine timestamp. Each later strictly newer sample produces one motion factor.

If a sample is unavailable, the factory is reset. This deliberately breaks the delta chain instead of creating one relative factor spanning a missed tick, because the engine only accepts consecutive-tick relative factors.

The OctoQuad factor has one configurable six-dimensional joint covariance over pose delta and velocity. The default is block diagonal as an initial tuning value; logged sensor data should eventually be used to estimate its cross-covariance. It is not currently scaled with elapsed time.

## OctoQuad and the process are peer factors

Commands produce a full-state `ProcessFactor`; OctoQuad produces a full-state `MotionDeltaFactor`. Both enter the same factor product. OctoQuad is not promoted into a special prediction path, and either factor can independently advance the six-state estimate.

## Why OctoQuad motion is one factor

The pose delta and any velocity computed from the same dead wheels/IMU share underlying data. Treating them as independent observations would double-count that sensor information unless their joint covariance were modeled explicitly.

The current design puts pose delta and reported velocity in one residual with one joint covariance. This makes OctoQuad-only operation recover ordinary pose-and-velocity odometry without pretending the two outputs are statistically independent.

## AprilTags: absolute field observations

An eligible AprilTag detection provides a robot field pose:

\[
Z_A=[x_A,y_A,\theta_A].
\]

`AprilTagFrameProcessor` is responsible for FTC Vision conversion and same-frame combination. It does not use the current fused pose to solve the detection. The detection's `robotPose` is treated as an independently produced absolute observation.

The processor:

1. Chooses one acquisition timestamp for the frame.
2. Rejects null detections, mismatched-frame detections, detections without robot pose, and tags that are not stationary references.
3. Converts position to inches and yaw to radians.
4. Applies the configured field-convention transform.
5. Combines accepted same-frame pose observations into one `AbsolutePoseFactor`.

Unknown, moving, or otherwise unsuitable tags should fail the configured `stationaryReference` predicate.

## Same-frame consensus

Detections from one image are not independent. They share camera pose, calibration, image timing, and image noise. Simply multiplying five same-frame tag likelihoods would make the frame appear five times as trustworthy.

The processor instead:

1. Selects the observation with the lowest total covariance-weighted disagreement as a consensus reference.
2. Unwraps all headings around that reference.
3. Gates each observation using its covariance plus the shared camera covariance floor.
4. Averages accepted information matrices and information vectors rather than summing them.
5. Adds the shared camera covariance floor.
6. Adds empirical disagreement covariance scaled by `disagreementInflation`.

Therefore, more agreeing tags can stabilize the frame estimate without automatically dividing covariance by the tag count.

The consensus gate is sensor preprocessing, not a hidden global fusion-engine outlier gate. Once an `AbsolutePoseFactor` reaches `FusionEngine`, the engine applies it according to its declared covariance even if it is far from the current estimate.

## AprilTag timing and deduplication

The factor timestamp is `frameAcquisitionNanoTime`, not processing-completion time. This allows delayed replay to place the observation near when the image was captured.

`AprilTagFrameProcessor` remembers the latest processed frame timestamp and returns no factor for duplicate or older frames. `AbsolutePoseFactor.observationId()` provides a second deduplication layer using source plus timestamp.

An external vision loop typically calls:

```java
AbsolutePoseFactor factor = aprilTagProcessor.fromDetections(detections);
if (factor != null) fusedLocalizer.submitFactor(factor);
```

`submitFactor()` only queues the observation. The next atomic localization commit attaches it to history and publishes the corrected estimate.

## Sensor covariance meaning

All pose factor covariance matrices use coordinates `[x, y, heading]`:

- Position variances are in squared inches.
- Heading variance is in squared radians.
- Off-diagonal terms represent correlated measurement error.

Covariance describes observation uncertainty, not a manual blend percentage. Halving a standard deviation quarters its variance and quadruples its information.

