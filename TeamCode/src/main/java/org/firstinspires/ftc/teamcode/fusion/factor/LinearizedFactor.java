package org.firstinspires.ftc.teamcode.fusion.factor;

import org.ejml.simple.SimpleMatrix;
import org.firstinspires.ftc.teamcode.fusion.core.MatrixOps;

/** One factor evaluated into a local linear Gaussian constraint. */
public final class LinearizedFactor {
    public final double[] residual;
    public final SimpleMatrix covariance;
    public final SimpleMatrix[] jacobians;

    public LinearizedFactor(double[] residual, SimpleMatrix covariance,
                            SimpleMatrix... jacobians) {
        if (jacobians == null || jacobians.length == 0) {
            throw new IllegalArgumentException("factor has no Jacobians");
        }
        this.residual = MatrixOps.copyVector(residual, "factor residual");
        this.covariance = MatrixOps.requirePositiveDefinite(
                covariance, residual.length, "factor covariance");
        this.jacobians = new SimpleMatrix[jacobians.length];
        for (int i = 0; i < jacobians.length; i++) {
            this.jacobians[i] = MatrixOps.copy(jacobians[i]);
            if (jacobians[i].numRows() != residual.length) {
                throw new IllegalArgumentException("Jacobian residual dimension does not match");
            }
        }
    }

    /** Horizontally concatenates the state Jacobian blocks. */
    public SimpleMatrix observation() {
        int columns = 0;
        for (SimpleMatrix jacobian : jacobians) columns += jacobian.numCols();
        SimpleMatrix result = new SimpleMatrix(residual.length, columns);
        int column = 0;
        for (SimpleMatrix jacobian : jacobians) {
            result.insertIntoThis(0, column, jacobian);
            column += jacobian.numCols();
        }
        return result;
    }
}
