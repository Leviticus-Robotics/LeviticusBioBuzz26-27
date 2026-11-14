# Localization architecture

The production localization path intentionally has two stages:

```text
OctoQuad Localizer
       |
       v
FusedLocalizer <--- delayed absolute pose Gaussians (for example AprilTags)
       |
       v
PredictiveLocalizer <--- latest applied drivetrain input
```

## FusedLocalizer

`FusedLocalizer` is the authoritative estimator. It wraps one ordinary Pedro
`Localizer`, applies each measured relative pose change directly, and retains a
short timestamped history. An absolute measurement is supplied through
`addAbsoluteMeasurement(source, timestamp, gaussian)`.

When an absolute measurement arrives late, the localizer inserts a state at its
capture time, fuses the two pose Gaussians, then replays every later odometry
increment and absolute measurement. Its state is pose-only: a three-dimensional
Gaussian for x, y, and heading. Velocity is the latest velocity reported by the
wrapped Pedro localizer.

A camera result slightly newer than the latest odometry tick is queued until that
tick exists. A result between two retained ticks is inserted by interpolating the
measured relative pose transform, rather than rounding it to an endpoint.

This means an absolute correction changes the global frame without making the
OctoQuad fight that correction. Future OctoQuad deltas continue from the corrected
pose.

Internally, responsibilities are separated by domain rather than collected in a
utility class:

- `LocalizationHistory` owns ticks, interpolation, deduplication, pruning, and replay.
- `OdometryModel` propagates a pose estimate and its uncertainty through measured motion.
- `PoseEstimate` gives a three-dimensional Gaussian its pose semantics and applies
  absolute corrections.
- `FusedLocalizer` only coordinates those pieces with Pedro's `Localizer` interface.

## PredictiveLocalizer

`PredictiveLocalizer<I>` wraps `FusedLocalizer` and a generic `ProcessModel<I>`.
It is an event handler, so `FusionMecanumDrivetrain` sends it the motor powers that
were actually applied. Calling `predict(horizonSeconds)` projects the latest fused
state forward using the current input.

Prediction is deliberately not fed back into the fused estimate. The authoritative
state therefore remains ordinary odometry plus absolute corrections, while callers
that care about latency—such as shooting-on-the-move code—can request a prediction
for the time at which an action will occur.

## Supporting probability types

`Gaussian` stores mean and covariance. `InformationGaussian` stores the equivalent
information vector and information matrix. `GaussianFusion.product` combines
independent Gaussian information; `informationAverage` conservatively combines
correlated observations from one camera frame. AprilTag frame processing uses the
latter before submitting one absolute Gaussian to the localizer.

Gaussian construction is inexpensive and defensive-copying, while `validate()` is
called explicitly at external measurement and configuration boundaries. Internal
prediction and replay do not repeatedly perform Cholesky validation on values that
were produced by already validated mathematics.

## Robot wiring

`Constants.createFusedLocalizer(...)` constructs the wrapped OctoQuad localizer,
the fused localizer, and the process model, then registers the predictive localizer
as a drivetrain event handler. `RobotCombined` wraps it in `MercurialLocalizer` and
submits completed AprilTag frames through the generic absolute-measurement API.
