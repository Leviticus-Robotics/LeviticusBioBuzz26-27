package org.firstinspires.ftc.teamcode.fusion.factor;

import org.ejml.simple.SimpleMatrix;

/**
 * An immutable probabilistic constraint over one or more timestamped states.
 *
 * <p>Factors are retained and may be linearized more than once during nonlinear optimization.
 * Implementations must therefore be immutable, deterministic, and side-effect-free.</p>
 */
public interface Factor {
    String source();
    long[] timestamps();
    double[] residual(StateValues values);
    SimpleMatrix covariance(StateValues values);

    /**
     * Linearizes this factor around the supplied trajectory. Most factors only need to define
     * their residual and covariance; specialized factors may override this with analytic
     * Jacobians when profiling shows that it matters.
     */
    default LinearizedFactor linearize(StateValues values) {
        long[] connectedTimestamps = timestamps();
        double[] center = residual(values);
        SimpleMatrix[] jacobians = new SimpleMatrix[connectedTimestamps.length];
        double epsilon = 1e-6;

        for (int stateIndex = 0; stateIndex < connectedTimestamps.length; stateIndex++) {
            long perturbedTimestamp = connectedTimestamps[stateIndex];
            double[] state = values.state(perturbedTimestamp);
            SimpleMatrix jacobian = new SimpleMatrix(center.length, state.length);

            for (int column = 0; column < state.length; column++) {
                double[] high = state.clone();
                double[] low = state.clone();
                high[column] += epsilon;
                low[column] -= epsilon;
                double[] highResidual = residual(replacing(values, perturbedTimestamp, high));
                double[] lowResidual = residual(replacing(values, perturbedTimestamp, low));
                if (highResidual.length != center.length || lowResidual.length != center.length) {
                    throw new IllegalArgumentException("factor residual dimension changed during linearization");
                }
                for (int row = 0; row < center.length; row++) {
                    jacobian.set(row, column,
                            residualDifference(row, highResidual[row], lowResidual[row])/(2*epsilon));
                }
            }
            jacobians[stateIndex] = jacobian;
        }

        return new LinearizedFactor(connectedTimestamps, center, covariance(values), jacobians);
    }

    default double residualDifference(int row, double high, double low) {
        return high - low;
    }

    default long timestampNanos() {
        long latest = Long.MIN_VALUE;
        for (long timestamp : timestamps()) latest = Math.max(latest, timestamp);
        return latest;
    }

    static StateValues replacing(StateValues values, long timestampNanos, double[] replacement) {
        return new StateValues() {
            @Override public double[] state(long requestedTimestamp) {
                return requestedTimestamp == timestampNanos
                        ? replacement.clone() : values.state(requestedTimestamp);
            }
        };
    }
}
