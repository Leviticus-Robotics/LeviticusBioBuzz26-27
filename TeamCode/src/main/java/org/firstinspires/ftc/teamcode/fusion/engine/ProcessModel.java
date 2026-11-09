package org.firstinspires.ftc.teamcode.fusion.engine;

import org.ejml.simple.SimpleMatrix;
import org.firstinspires.ftc.teamcode.fusion.core.Gaussian;
import org.firstinspires.ftc.teamcode.fusion.core.MatrixOps;

/**
 * Deterministic process transition used during both forward prediction and history replay.
 * Implementations must be side-effect-free, and must not mutate the prior state or input.
 */
public interface ProcessModel<I> {
    double[] predictMean(double[] prior, I input, double elapsedSeconds);
    SimpleMatrix processNoise(double[] prior, I input, double elapsedSeconds);

    default double[] residual(double[] predicted, double[] actual) {
        if (predicted.length != actual.length) {
            throw new IllegalArgumentException("state dimensions do not match");
        }
        double[] result = new double[predicted.length];
        for (int i = 0; i < result.length; i++) result[i] = actual[i] - predicted[i];
        return result;
    }

    default double residualDifference(int row, double high, double low) { return high - low; }
    default double[] normalize(double[] state) { return state; }

    default Gaussian predict(Gaussian prior, I input, double elapsedSeconds) {
        double[] mean = prior.mean();
        double[] predicted = predictMean(mean, input, elapsedSeconds);
        int dimension = mean.length;
        SimpleMatrix jacobian = new SimpleMatrix(dimension, dimension);
        double epsilon = 1e-6;
        for (int column = 0; column < dimension; column++) {
            double[] high = mean.clone();
            double[] low = mean.clone();
            high[column] += epsilon;
            low[column] -= epsilon;
            double[] highPrediction = predictMean(high, input, elapsedSeconds);
            double[] lowPrediction = predictMean(low, input, elapsedSeconds);
            for (int row = 0; row < dimension; row++) {
                jacobian.set(row, column,
                        residualDifference(row, highPrediction[row], lowPrediction[row])/(2*epsilon));
            }
        }
        SimpleMatrix covariance = jacobian.mult(prior.covariance()).mult(jacobian.transpose())
                .plus(processNoise(mean, input, elapsedSeconds));
        return new Gaussian(normalize(predicted), MatrixOps.symmetrize(covariance));
    }
}
