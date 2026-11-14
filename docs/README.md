# Fusion and localization documentation

The current system is documented in [Localization architecture](fusion-localization/README.md).

The short version is:

1. `FusedLocalizer` wraps an ordinary Pedro localizer such as OctoQuad.
2. Measured odometry deltas move a three-dimensional Gaussian pose estimate.
3. Timestamped absolute pose Gaussians correct retained history, after which later
   measurements are replayed.
4. `PredictiveLocalizer` optionally projects the latest corrected state forward from
   the motor powers published by `FusionMecanumDrivetrain`.
5. Prediction is an output for latency compensation; it never compounds into the
   authoritative fused pose.

## Source map

| Layer | Package | Responsibility |
|---|---|---|
| Gaussian math | `fusion` | Mean/covariance and information-form Gaussian combination |
| Process prediction | `localization.prediction` | Process-model contract, applied input, and command-response model |
| Localization | `localization.pedro` | Odometry fusion, history replay, future prediction, drivetrain input events |
| Vision boundary | `localization.apriltag` | Convert one camera frame into one absolute pose Gaussian |
| Composition root | `constants.Constants` | Construct and wire the production objects |
