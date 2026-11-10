package org.firstinspires.ftc.teamcode.fusion.factor;

import org.ejml.simple.SimpleMatrix;
import org.firstinspires.ftc.teamcode.fusion.core.MatrixOps;

/** Immutable residual, covariance, and analytic Jacobians supplied by a factor. */
public final class LinearizedFactor {
    public final long[] timestamps;
    public final double[] residual;
    public final SimpleMatrix covariance;
    public final SimpleMatrix[] jacobians;

    public LinearizedFactor(long[] timestamps, double[] residual, SimpleMatrix covariance,
                            SimpleMatrix[] jacobians) {
        if (timestamps == null || timestamps.length == 0) {
            throw new IllegalArgumentException("factor has no timestamps");
        }
        if (jacobians == null || jacobians.length != timestamps.length) {
            throw new IllegalArgumentException("one Jacobian is required per timestamp");
        }
        this.timestamps = timestamps.clone();
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
}
