# Factors and prediction

## The generic factor contract

Every `Factor` provides:

```java
String source();
long[] timestamps();
double[] residual(StateValues values);
SimpleMatrix covariance(StateValues values);
SimpleMatrix[] jacobians(StateValues values);
```

This is deliberately behavioral. The engine does not use `instanceof` to decide the mathematics of an absolute or relative sensor. It asks the factor to evaluate itself against timestamped state values.

`Factor.linearize()` packages those results into an immutable `LinearizedFactor` after validating dimensions and covariance.

## Factor arity

The interfaces describe only arity:

- `AbsoluteFactor` declares one timestamp.
- `RelativeFactor` declares a from timestamp and a to timestamp.

There is deliberately no predictive-factor hierarchy. A process transition and an odometry observation are both likelihood factors relating two consecutive states. The engine treats them identically.

## Process model and process factor

`ProcessModel<I>` defines a family of conditional transitions parameterized by input `I`:

```java
double[] predictMean(double[] prior, I input, double dt);
SimpleMatrix processNoise(double[] prior, I input, double dt);
SimpleMatrix transitionJacobian(double[] state, I input, double dt);
```

`transitionJacobian()` has a central-difference default. It computes coordinate differences through the model's `residual()` method, so a model-specific wrapped heading residual remains continuous across the angle branch cut. An analytic override remains possible, but should only be added when profiling justifies its additional derivation and maintenance cost.

Calling `factor(from, to, input)` creates an immutable `ProcessFactor<I>`. The factor captures:

- The interval timestamps.
- The actual input used over that interval.
- The process model.
- The elapsed time.

The fusion engine itself has no input type and stores no active command. Inputs belong to the factor whose behavior depends on them.

As an ordinary relative factor, `ProcessFactor` defines

\[
r_u=X_k-f(X_{k-1},u_k)
\]

using the code's `actual - predicted` residual convention, with Jacobian blocks

\[
[-F_k\quad I].
\]

The process factor is submitted once in the same tick batch as OctoQuad and other observations. The previous posterior supplies the prior on \(X_{k-1}\); all factors jointly constrain \((X_{k-1},X_k)\).

## The localization process model

`PoseVelocityProcessModel` converts robot-centric drive power into target field-frame velocity.

For heading \(\theta\):

\[
v_x^*=\cos\theta\,v_f-\sin\theta\,v_s,
\]

\[
v_y^*=\sin\theta\,v_f+\cos\theta\,v_s.
\]

The commanded velocity is not reached instantaneously. With response rate \(\lambda\) and interval \(\Delta t\), the blend is

\[
\alpha=1-e^{-\lambda\Delta t}.
\]

Velocity evolves as

\[
v_k=v_{k-1}+\alpha(v^*-v_{k-1}),
\]

and pose is integrated using the average of the old and new velocities. Angular velocity follows the same first-order response.

Process covariance is diagonal and scales with elapsed time:

\[
Q_k=\operatorname{diag}(q)\Delta t.
\]

The inherited central-difference Jacobian automatically captures the dependence of field-frame velocity on heading. This keeps the derivative synchronized with future changes to the transition equations.

## Supplying the process factor

`CommandProcessSource` holds the drive input active for the current interval. Once per loop it creates a `ProcessFactor` and returns it through the same `AutomaticFusionSource` interface used by OctoQuad. `FusedLocalizer` gathers all available factors and submits one uniform list to the engine.

The engine does not require that a process factor exist specifically. It requires the submitted constraints, taken together, to make the new state observable. Both the process factor and `MotionDeltaFactor` can independently advance the normal six-state estimate; when both are present, they are peer likelihoods in the same batch.

## Observation identity and independence

`observationId()` controls deduplication:

- A non-null ID means two factors with the same ID represent the same observation and must not be counted twice.
- A null ID means every instance is an independent observation, even if source and timestamp match.

Current pose factors use source plus timestamp, or source plus interval endpoints, as their IDs.

Deduplication is not a substitute for covariance modeling. Two different IDs can still have correlated errors. For example, detections from one camera frame share calibration and image noise, so they are combined into one camera factor before reaching the engine.

## Adding a generic factor

A factor should be immutable and side-effect-free:

```java
public final class ScalarAbsoluteFactor implements AbsoluteFactor {
    private final long timestamp;
    private final double measurement;
    private final double variance;

    @Override public String source() { return "example"; }
    @Override public long timestampNanos() { return timestamp; }

    @Override public double[] residual(StateValues values) {
        return new double[] {values.state(timestamp)[0] - measurement};
    }

    @Override public SimpleMatrix covariance(StateValues values) {
        return MatrixOps.diagonal(variance);
    }

    @Override public SimpleMatrix[] jacobians(StateValues values) {
        SimpleMatrix h = new SimpleMatrix(1, values.state(timestamp).length);
        h.set(0, 0, 1);
        return new SimpleMatrix[] {h};
    }
}
```

The residual sign and Jacobian must be consistent. Analytic Jacobians should be compared against central numerical differences in tests before deployment.

