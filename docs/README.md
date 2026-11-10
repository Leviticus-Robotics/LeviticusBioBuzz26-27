# Fusion and localization documentation

This folder explains the robot's fused localization system from the mental model down to sensor-specific behavior and tuning.

Read the documents in order if the system is new to you:

1. [Architecture overview](fusion-localization/01-architecture-overview.md) — responsibilities, data flow, and terminology.
2. [Probability and estimation mathematics](fusion-localization/02-probability-and-math.md) — Gaussians, prediction, factors, information updates, and marginalization.
3. [Factors and prediction](fusion-localization/03-factors-and-prediction.md) — the factor interfaces, process model, joint prediction, and double-counting rules.
4. [Engine lifecycle and timing](fusion-localization/04-engine-lifecycle-and-timing.md) — tick commits, history, delayed measurements, replay, pruning, and deduplication.
5. [Sensor models](fusion-localization/05-sensor-models.md) — OctoQuad and AprilTag behavior in detail.
6. [Pedro and robot integration](fusion-localization/06-pedro-and-robot-integration.md) — how the estimator is driven by the robot loop and exposed to Pedro/Mercurial.
7. [Tuning, testing, and extension](fusion-localization/07-tuning-testing-and-extension.md) — covariance tuning, diagnostics, failure modes, and adding a new sensor.

## One-paragraph summary

The estimator maintains a six-dimensional Gaussian robot state. Once per loop, a predictive factor creates a correlated joint distribution over the previous and current state. Absolute and relative sensor factors constrain that joint distribution through covariance-weighted residuals. The previous state is then marginalized, leaving the current posterior. Recent ticks are retained so delayed observations can be inserted at their measurement time and only the affected suffix replayed. The generic fusion package knows nothing about FTC hardware, Pedro, OctoQuad, AprilTags, or robot pose conventions; those responsibilities live in localization and adapter packages.

## Source map

| Layer | Package | Responsibility |
|---|---|---|
| Gaussian math | `fusion.core` | Gaussian representations, validation, joint prediction, information-form updates, marginalization |
| Generic inference | `fusion.factor`, `fusion.engine` | Factor contracts, prediction capability, tick history, replay, diagnostics |
| Localization model | `localization.core`, `localization.engine`, `localization.factor` | Robot state, planar pose math, command process model, pose factors |
| Sensor adapters | `localization.sensor` | Convert hardware or vision results into factors |
| Consumer adapters | `localization.pedro` | Implement Pedro's `Localizer` and capture applied drivetrain input |
| Composition root | `constants.Constants` | Construct and connect the concrete implementation |

