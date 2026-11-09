package org.firstinspires.ftc.teamcode.fusion.factor;

import org.ejml.simple.SimpleMatrix;
import org.firstinspires.ftc.teamcode.fusion.core.MatrixOps;

/** Residual, covariance, and per-state Jacobians at one factor linearization point. */
public final class LinearizedFactor {
    public final long[] timestamps;
    public final double[] residual;
    public final SimpleMatrix covariance;
    public final SimpleMatrix[] jacobians;

    public LinearizedFactor(long[] timestamps, double[] residual,
                            SimpleMatrix covariance, SimpleMatrix[] jacobians) {
        if (timestamps == null || timestamps.length == 0) {
            throw new IllegalArgumentException("factor has no state timestamps");
        }
        if (jacobians == null || jacobians.length != timestamps.length) {
            throw new IllegalArgumentException("factor Jacobians do not match state timestamps");
        }
        this.timestamps = timestamps.clone();
        this.residual = MatrixOps.copyVector(residual, "factor residual");
        this.covariance = MatrixOps.requirePositiveDefinite(
                covariance, residual.length, "factor covariance");
        this.jacobians = new SimpleMatrix[jacobians.length];
        for (int i = 0; i < jacobians.length; i++) {
            this.jacobians[i] = MatrixOps.copy(jacobians[i]);
            if (this.jacobians[i].numRows() != residual.length) {
                throw new IllegalArgumentException("factor Jacobian row count is invalid");
            }
        }
    }
}
