# Pedro and robot integration

## Construction

`Constants.createLocalizer(hardwareMap, drivetrain)` is the localization composition root. It creates and connects:

```text
Constants + PoseVelocityProcessModel.Config
      │
      ├── initial RobotState
      ├── PoseVelocityProcessModel
      ├── CommandProcessSource
      ├── OctoQuadLocalizer
      ├── OctoQuadFusionSource
      ├── AprilTagFrameProcessor
      └── createLocalizer(sources)
              ├── FusionEngine
              └── FusedLocalizer
```

The configured `FusedLocalizer` uses:

- `CommandProcessSource` and OctoQuad as automatically polled factor sources.
- A two-second engine history.
- AprilTags as externally submitted asynchronous factors.

The robot constructs its drivetrain first and passes that instance into the localizer factory. Constants registers `CommandProcessSource` as a handler of the drivetrain's applied-input events. No component-bundle type is needed.

## One `FusedLocalizer.update()`

The update method intentionally captures one timestamp and uses it for the whole loop:

```java
long tickTimestamp = clock.now();
```

It then:

1. Polls every `AutomaticFusionSource` once with the previous and current timestamps.
2. Collects non-null factors.
3. Atomically commits the uniform factor batch.
4. Throws if the engine rejects the tick.

Using one timestamp prevents sensor polling duration from accidentally producing mismatched interval endpoints.

## Applied drive input

The process model needs the command that was actually applied during the interval, not merely the command requested by a higher-level controller.

`FusionMecanumDrivetrain` delegates driving to Pedro's real mecanum drivetrain and then reads the resulting motor powers. It transforms wheel powers back into normalized chassis components:

\[
u=[u_{forward},u_{strafe},u_{turn}].
\]

`FusionMecanumDrivetrain` is an `EventManager<ControlInput>`. The robot registers `CommandProcessSource` as its handler. At the next advancing localization update, that source captures the most recently published input in a new ordinary `ProcessFactor`.

The intended time interpretation is:

```text
drive powers applied at t(k-1)
          │
          └── active over interval ──> localization update at t(k)
```

The robot loop or Pedro follower owns localization update timing. `FusionMecanumDrivetrain` only records the motor command that was actually applied; it never polls localization itself.

## Pedro-facing state

Pedro's `Localizer` consumers see a `MotionState` containing:

- `Pose(x, y, heading)` from the fused mean.
- `Velocity(vx, vy, omega)` from the fused mean.

Pedro does not see factors, sensor histories, or covariance. It sees the current best state estimate through its normal interface.

`setPose()` resets position while preserving the previously estimated velocities. `reset()` resets pose and velocity to zero. Both operations:

- Reset automatic relative-source baselines.
- Replace engine history with a new initial Gaussian at the current clock time.

Resetting OctoQuad's factor baseline is mandatory. Otherwise, the next raw reading would create a delta across two incompatible fused origins.

## Mercurial integration

The fused Pedro localizer can be wrapped by `MercurialLocalizer`. That wrapper publishes Pedro `MotionState` updates through Mercurial's event system.

This does not cause every subscribed subsystem to perform every possible action. A subscriber receives new motion data; its current mode decides whether and how to use it. For example, a shooter in idle mode can ignore targeting updates, while a target-tracking mode consumes them.

Localization remains independent of shooter, drivetrain-control, and autonomous orchestration decisions.

## AprilTag submission path

AprilTags are not polled inside `FusedLocalizer.update()` because camera processing may complete asynchronously. The robot exposes:

```java
robot.submitAprilTags(detections);
```

That method:

1. Converts the newest eligible frame to an absolute factor.
2. Queues the factor through `FusedLocalizer.submitFactor()`.
3. Lets the next normal localization commit perform historical insertion and replay.

No camera callback directly changes the published state halfway through a robot tick.

## Hardware ownership

Hardware ownership is intentionally at the boundaries:

- `OctoQuadFusionSource` owns polling the raw OctoQuad localizer.
- `FusionMecanumDrivetrain` owns reading applied motor powers.
- The camera/OpMode owns obtaining AprilTag detections.
- `FusionEngine` owns no hardware.

This makes generic fusion deterministic and host-testable.

## Runtime data-flow example

For a loop where an AprilTag result also becomes available:

```text
camera thread/OpMode
    └── detections → AprilTagFrameProcessor → queued AbsolutePoseFactor

Pedro/localization loop
    ├── one clock timestamp
    ├── OctoQuad.update() → pose + velocity → MotionDeltaFactor
    ├── active motor input → ProcessFactor
    └── FusionEngine.commitTick()
            ├── insert queued AprilTag at historical tick
            ├── replay affected suffix
            ├── apply current OctoQuad factor
            └── publish current RobotState

Pedro follower / Mercurial consumers
    └── read or receive final fused MotionState
```

## Debug output

`FusedLocalizer.debug()` reports:

- Pose and velocity mean.
- Covariance diagonal.
- Current pending drive input.
- History item count.
- Last recomputation duration.
- Accepted and rejected factor totals.
- Latest timestamp per source.
- Last engine decision.

These values can be forwarded to telemetry without exposing sensor-specific internals to the fusion engine.

