package org.firstinspires.ftc.teamcode.fusion.engine;

import org.ejml.simple.SimpleMatrix;
import org.firstinspires.ftc.teamcode.fusion.factor.ProcessFactor;

/**
 * Deterministic process transition represented as an ordinary relative factor.
 * Implementations must be side-effect-free, and must not mutate the prior state or input.
 */
public interface ProcessModel<I> {
    double[] predictMean(double[] prior, I input, double elapsedSeconds);
    SimpleMatrix processNoise(double[] prior, I input, double elapsedSeconds);

    /**
     * Central-difference transition Jacobian. Implementations may override this with an analytic
     * Jacobian for better runtime performance. The model's residual defines coordinate differences,
     * so wrapped coordinates such as headings remain continuous across their branch cut.
     */
    default SimpleMatrix transitionJacobian(double[] state, I input, double elapsedSeconds) {
        int dimension = state.length;
        SimpleMatrix jacobian = new SimpleMatrix(dimension, dimension);
        double epsilon = 1e-6;
        for (int column = 0; column < dimension; column++) {
            double[] highState = state.clone();
            double[] lowState = state.clone();
            highState[column] += epsilon;
            lowState[column] -= epsilon;
            double[] highPrediction = predictMean(highState, input, elapsedSeconds);
            double[] lowPrediction = predictMean(lowState, input, elapsedSeconds);
            double[] difference = residual(lowPrediction, highPrediction);
            if (difference.length != dimension) {
                throw new IllegalArgumentException("predicted state dimension changed");
            }
            for (int row = 0; row < dimension; row++) {
                jacobian.set(row, column, difference[row]/(2*epsilon));
            }
        }
        return jacobian;
    }

    default double[] residual(double[] predicted, double[] actual) {
        if (predicted.length != actual.length) {
            throw new IllegalArgumentException("state dimensions do not match");
        }
        double[] result = new double[predicted.length];
        for (int i = 0; i < result.length; i++) result[i] = actual[i] - predicted[i];
        return result;
    }

    default double[] normalize(double[] state) { return state; }

    default ProcessFactor<I> factor(long fromTimestampNanos, long toTimestampNanos, I input) {
        return new ProcessFactor<>("process", fromTimestampNanos, toTimestampNanos, this, input);
    }

}
