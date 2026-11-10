package org.firstinspires.ftc.teamcode.fusion.core;

import java.util.Collections;
import java.util.List;
import org.ejml.simple.SimpleMatrix;

/** Gaussian operations shared by single-state and augmented factor updates. */
public final class GaussianUpdater {
    private GaussianUpdater() { }

    /** Applies one linearized Gaussian constraint. */
    public static Gaussian apply(Gaussian prior, double[] residual,
                                 SimpleMatrix observation, SimpleMatrix measurementNoise) {
        return applyAll(prior, Collections.singletonList(
                new Constraint(residual, observation, measurementNoise)));
    }

    /** Applies independent constraints simultaneously in information form. */
    public static Gaussian applyAll(Gaussian prior, List<Constraint> constraints) {
        if (constraints == null) throw new IllegalArgumentException("constraints are null");
        if (constraints.isEmpty()) return prior;
        int stateDimension = prior.dimension();
        SimpleMatrix information = MatrixOps.inverse(prior.covariance());
        SimpleMatrix gradient = new SimpleMatrix(stateDimension, 1);
        for (Constraint constraint : constraints) {
            if (constraint.observation.numCols() != stateDimension) {
                throw new IllegalArgumentException("observation dimension does not match state");
            }
            SimpleMatrix measurementInformation = MatrixOps.inverse(constraint.covariance);
            information = information.plus(constraint.observation.transpose()
                    .mult(measurementInformation).mult(constraint.observation));
            gradient = gradient.plus(constraint.observation.transpose()
                    .mult(measurementInformation).mult(MatrixOps.column(constraint.residual)));
        }
        SimpleMatrix covariance = MatrixOps.inverse(information);
        SimpleMatrix correctedMean = MatrixOps.column(prior.mean())
                .minus(covariance.mult(gradient));
        return new Gaussian(MatrixOps.vector(correctedMean), MatrixOps.symmetrize(covariance));
    }

    /**
     * Solves one two-state factor problem. The previous Gaussian is the only prior; every
     * relationship involving the new state is supplied uniformly through constraints.
     */
    public static Gaussian solveAugmented(Gaussian previous, double[] currentInitial,
                                          List<Constraint> constraints) {
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

        SimpleMatrix information = new SimpleMatrix(jointDimension, jointDimension);
        information.insertIntoThis(0, 0, MatrixOps.inverse(previous.covariance()));
        SimpleMatrix gradient = new SimpleMatrix(jointDimension, 1);
        for (Constraint constraint : constraints) {
            if (constraint.observation.numCols() != jointDimension) {
                throw new IllegalArgumentException("observation dimension does not match joint state");
            }
            SimpleMatrix measurementInformation = MatrixOps.inverse(constraint.covariance);
            information = information.plus(constraint.observation.transpose()
                    .mult(measurementInformation).mult(constraint.observation));
            gradient = gradient.plus(constraint.observation.transpose()
                    .mult(measurementInformation).mult(MatrixOps.column(constraint.residual)));
        }
        SimpleMatrix covariance = MatrixOps.inverse(information);
        SimpleMatrix correctedMean = MatrixOps.column(initial).minus(covariance.mult(gradient));
        return new Gaussian(MatrixOps.vector(correctedMean), MatrixOps.symmetrize(covariance));
    }

    /** Horizontally concatenates equal-height Jacobian blocks. */
    public static SimpleMatrix observation(SimpleMatrix[] jacobians) {
        if (jacobians == null || jacobians.length == 0) {
            throw new IllegalArgumentException("no Jacobians supplied");
        }
        int rows = jacobians[0].numRows();
        int columns = 0;
        for (SimpleMatrix jacobian : jacobians) {
            if (jacobian.numRows() != rows) {
                throw new IllegalArgumentException("Jacobian row dimensions do not match");
            }
            columns += jacobian.numCols();
        }
        SimpleMatrix result = new SimpleMatrix(rows, columns);
        int column = 0;
        for (SimpleMatrix jacobian : jacobians) {
            result.insertIntoThis(0, column, jacobian);
            column += jacobian.numCols();
        }
        return result;
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

    /** One already-linearized likelihood embedded in the state being updated. */
    public static final class Constraint {
        public final double[] residual;
        public final SimpleMatrix observation;
        public final SimpleMatrix covariance;

        public Constraint(double[] residual, SimpleMatrix observation, SimpleMatrix covariance) {
            this.residual = MatrixOps.copyVector(residual, "constraint residual");
            this.observation = MatrixOps.copy(observation);
            this.covariance = MatrixOps.requirePositiveDefinite(
                    covariance, residual.length, "constraint covariance");
            if (observation.numRows() != residual.length) {
                throw new IllegalArgumentException("observation and residual dimensions do not match");
            }
        }
    }
}
