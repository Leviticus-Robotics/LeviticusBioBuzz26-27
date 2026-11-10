# Tuning, testing, and extension

## Covariance is the tuning interface

The estimator does not use explicit blend percentages. Relative trust follows from covariance:

\[
\text{information}=R^{-1}.
\]

If one source has one quarter of another source's variance in a coordinate, it contributes approximately four times as much local information in that coordinate.

Tune standard deviations from data first, then square them to obtain diagonal variances. Avoid tuning variances as arbitrary gains.

## Current configuration values

`FusedLocalizer.Config.defaults()` supplies:

- Initial six-state covariance.
- OctoQuad joint pose-delta and velocity covariance.
- Per-detection AprilTag covariance.
- Shared camera covariance floor.
- Same-frame AprilTag consensus gate.
- Disagreement inflation.
- Field-convention transform.
- Retained history duration.

`PoseVelocityProcessModel.Config.defaults()` supplies:

- Velocity response rate.
- Maximum forward, strafe, and angular velocities.
- Six process-noise rates per second.

Treat defaults as starting points, not sensor specifications.

## Recommended tuning sequence

### 1. Verify coordinates and timing

Before covariance tuning:

- Confirm positive X, positive Y, and positive heading agree across Pedro, AprilTag, and fusion coordinates.
- Confirm distances are inches and headings are radians.
- Confirm every timestamp uses the same monotonic clock domain.
- Confirm AprilTag timestamps describe acquisition time.
- Confirm OctoQuad deltas rotate correctly at nonzero heading.

Covariance cannot fix a frame, unit, sign, or timestamp error.

### 2. Tune the process model

Log applied motor input and measured motion. Adjust:

- Maximum forward, strafe, and angular velocities.
- Velocity response rate.
- Process noise per second.

The mean model should capture broad behavior, not every mechanical detail. Process noise must cover command saturation, wheel slip, voltage variation, impacts, and model mismatch.

If process covariance is too small, commands dominate real motion measurements. If it is too large, prediction becomes noisy and velocity may be weakly constrained.

### 3. Characterize OctoQuad deltas

Run repeated known motions and compare measured local deltas with reference deltas. Estimate covariance separately for:

- Forward displacement.
- Lateral displacement.
- Heading change.

Examine off-diagonal correlations. Rotation can produce correlated translation error when tracking-wheel geometry is imperfect.

The current covariance is fixed per sample. If loop intervals vary materially, consider a future model that scales or otherwise predicts delta uncertainty from elapsed time and motion magnitude.

### 4. Characterize AprilTags

Collect stationary and moving logs across:

- Distance to tag.
- Viewing angle.
- Number of visible tags.
- Different field regions and lighting.

Tune per-detection covariance, shared floor, consensus gate, and disagreement inflation from residual distributions. A constant covariance is simple but cannot reflect the strong range/angle dependence of vision quality; a future observation model may provide covariance per detection.

### 5. Validate consistency

For a residual \(r\) with innovation covariance \(S\), inspect normalized innovation squared:

\[
\operatorname{NIS}=r^TS^{-1}r.
\]

Persistent large values indicate underestimated covariance, model bias, bad timing, or frame errors. Persistent tiny values can indicate overly conservative covariance or correlated evidence counted as independent.

The engine currently has no global NIS rejection gate. Add gating at the sensor adapter when the source has clear validity semantics, or add an explicit generic policy only if it is intentionally designed and observable in diagnostics.

## Testing strategy

### Pure mathematical tests

Test:

- Gaussian/information round trips.
- Joint-prediction covariance blocks.
- Marginal extraction.
- Simultaneous update order invariance.
- Positive-definite validation.

### Factor tests

For every analytic factor:

- Compare analytic Jacobians to central numerical differences over randomized valid states.
- Test angle wrap near \(-\pi\) and \(+\pi\).
- Verify zero residual at the exact measurement.
- Verify covariance dimensions and units.

`ProcessModel` deliberately uses numerical differentiation by default. Add an analytic override only after profiling shows a meaningful robot-side cost, and then verify it against the numerical implementation over randomized states.

### Engine tests

Test:

- One prediction evaluation per normal tick.
- Delayed replay matches chronological in-order fusion.
- Replay starts at the affected tick.
- History pruning and stale-factor rejection.
- Predictor double-count rejection.
- Duplicate observation handling.
- Independent same-timestamp observations are both retained.
- Failed commits do not publish partial state.

### Sensor tests

Test:

- OctoQuad first-sample baseline behavior.
- Local delta correctness at rotated poses.
- Reset and missing-sample re-baselining.
- AprilTag frame deduplication.
- Stationary-reference filtering.
- Heading consensus across the wrap boundary.
- Same-frame information averaging and disagreement inflation.

### Integration and performance tests

The retained benchmark simulates:

- 50 Hz robot ticks.
- Process prediction every tick.
- OctoQuad relative factors every tick.
- 10 Hz AprilTag factors with 100 ms latency.
- A full two-second history window.

Run host tests with:

```bash
./gradlew :TeamCode:testDebugUnitTest --no-daemon
```

Host timing is useful for detecting regressions, but final validation must run on the actual Control Hub under camera, telemetry, and follower load.

## Adding a new automatic sensor

For a sensor sampled once per loop:

1. Define the measurement semantics and coordinate frame.
2. Implement an immutable factor with analytic residual/Jacobians and covariance.
3. Implement `AutomaticFusionSource.poll(tickTimestamp)`.
4. Return at most one factor for that source/tick, or explicitly document independent multiple factors.
5. Reset any relative baseline in `reset()`.
6. Add the source to `FusedLocalizer` construction.
7. Add Jacobian, timing, reset, replay, and integration tests.

Do not give the sensor adapter access to `FusionEngine.state()` unless the sensor algorithm truly defines a state-dependent observation. A sensor should normally estimate its measurement independently; the factor compares it with the fused state later.

## Adding an asynchronous sensor

For a camera or network result that arrives after capture:

1. Preserve its acquisition timestamp.
2. Convert it into a factor outside the engine.
3. Call `FusedLocalizer.submitFactor()`.
4. Let the next tick commit perform atomic insertion and replay.

The measurement must arrive before its timestamp falls outside retained history.

## Adding another relative source

Implement `RelativeFactor` for a measurement relating consecutive ticks and return it from an `AutomaticFusionSource`, or submit it asynchronously with its timestamps. It enters the same product as the process and OctoQuad factors. Ensure the complete tick batch makes the new state observable and do not submit the same physical observation twice.

## Known limitations and possible future work

Current deliberate limitations include:

- Relative factors must connect consecutive ticks.
- Absolute factors are attached to the nearest tick rather than creating an exact-time state.
- Pose corrections use additive local coordinates instead of full `SE(2)` tangent-space retraction.
- Factors are linearized once per evaluation, without iteration.
- Delayed updates replay the affected chronological suffix.
- The information update uses dense small matrices and allocates temporary EJML objects.
- AprilTag and OctoQuad covariance models are presently constant.

Potential upgrades should be driven by measured need:

- Exact-time interpolation states for high camera latency or low loop rates.
- Motion-dependent sensor covariance.
- A manifold pose residual/retraction for very large corrections.
- Direct out-of-sequence cross-covariance updates for long delays.
- An incremental fixed-lag smoother for arbitrary historical factors and loop closures.
- Fixed-size matrix buffers if profiling shows allocation or matrix overhead on the Control Hub.

Each upgrade changes complexity or estimator semantics. Preserve the current factor boundary and add comparative log-replay tests before replacing the working estimator.

## Operational checklist

Before enabling fused localization in competition:

- Verify raw and fused localization side by side.
- Exercise heading wrap repeatedly.
- Apply large AprilTag corrections and confirm OctoQuad does not pull back to its raw origin.
- Drop OctoQuad samples and confirm clean re-baselining.
- Delay and duplicate AprilTag frames deliberately.
- Confirm history is long enough for worst-case camera latency.
- Monitor rejected factors and latest source timestamps.
- Confirm loop timing on the Control Hub.
- Retain the raw-localizer fallback until field logs show stable covariance and timing.

