package org.firstinspires.ftc.teamcode.fusion.core;

import java.util.Collections;
import java.util.List;
import org.ejml.simple.SimpleMatrix;
import org.firstinspires.ftc.teamcode.fusion.factor.LinearizedFactor;

/** Gaussian operations shared by single-state and augmented factor updates. */
public final class GaussianUpdater {
    private GaussianUpdater() { }

    /** Applies one linearized Gaussian constraint. */
    public static Gaussian apply(Gaussian prior, double[] residual,
                                 SimpleMatrix observation, SimpleMatrix measurementNoise) {
        return applyAll(prior, Collections.singletonList(
                new LinearizedFactor(residual, measurementNoise, observation)));
    }

    /** Applies independent constraints simultaneously in information form. */
    public static Gaussian applyAll(Gaussian prior, List<LinearizedFactor> constraints) {
        if (constraints == null) throw new IllegalArgumentException("constraints are null");
        if (constraints.isEmpty()) return prior;
        return solve(prior.mean(), MatrixOps.inverse(prior.covariance()), constraints);
    }

    /**
     * Solves one two-state factor problem. The previous Gaussian is the only prior; every
     * relationship involving the new state is supplied uniformly through constraints.
     */
    public static Gaussian solveAugmented(Gaussian previous, double[] currentInitial,
                                          List<LinearizedFactor> constraints) {
        if (currentInitial == null || currentInitial.length != previous.dimension()) {
            throw new IllegalArgumentException("current initial state dimension does not match prior");
        }
        if (constraints == null || constraints.isEmpty()) {
            throw new IllegalArgumentException("new state has no constraints");
        }
        int dimension = previous.dimension();
        int jointDimension = 2*dimension;
        double[] initial = new double[jointDimension];
        System.arraycopy(previous.mean(), 0, initial, 0, dimension);
        System.arraycopy(currentInitial, 0, initial, dimension, dimension);

        SimpleMatrix priorInformation = new SimpleMatrix(jointDimension, jointDimension);
        priorInformation.insertIntoThis(0, 0, MatrixOps.inverse(previous.covariance()));
        return solve(initial, priorInformation, constraints);
    }

    private static Gaussian solve(double[] initial, SimpleMatrix priorInformation,
                                  List<LinearizedFactor> constraints) {
        SimpleMatrix information = priorInformation.copy();
        SimpleMatrix gradient = new SimpleMatrix(initial.length, 1);
        for (LinearizedFactor constraint : constraints) {
            SimpleMatrix observation = constraint.observation();
            if (observation.numCols() != initial.length) {
                throw new IllegalArgumentException("observation dimension does not match state");
            }
            SimpleMatrix measurementInformation = MatrixOps.inverse(constraint.covariance);
            information = information.plus(observation.transpose()
                    .mult(measurementInformation).mult(observation));
            gradient = gradient.plus(observation.transpose()
                    .mult(measurementInformation).mult(MatrixOps.column(constraint.residual)));
        }
        SimpleMatrix covariance = MatrixOps.inverse(information);
        SimpleMatrix correctedMean = MatrixOps.column(initial).minus(covariance.mult(gradient));
        return new Gaussian(MatrixOps.vector(correctedMean), MatrixOps.symmetrize(covariance));
    }

    public static Gaussian marginal(Gaussian joint, int offset, int dimension) {
        if (offset < 0 || dimension <= 0 || offset + dimension > joint.dimension()) {
            throw new IllegalArgumentException("marginal range is outside state");
        }
        double[] mean = new double[dimension];
        System.arraycopy(joint.mean(), offset, mean, 0, dimension);
        SimpleMatrix covariance = joint.covariance().extractMatrix(
                offset, offset + dimension, offset, offset + dimension);
        return new Gaussian(mean, MatrixOps.symmetrize(covariance));
    }
}
