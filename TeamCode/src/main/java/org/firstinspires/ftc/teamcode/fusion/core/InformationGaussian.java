package org.firstinspires.ftc.teamcode.fusion.core;

import org.ejml.simple.SimpleMatrix;

/** Immutable information-vector/information-matrix Gaussian representation. */
public final class InformationGaussian {
    private final double[] informationVector;
    private final SimpleMatrix informationMatrix;

    public InformationGaussian(double[] informationVector, SimpleMatrix informationMatrix) {
        this.informationVector = MatrixOps.copyVector(informationVector, "information vector");
        this.informationMatrix = MatrixOps.requirePositiveDefinite(
                informationMatrix, informationVector.length, "information matrix");
    }

    public int dimension() { return informationVector.length; }
    public double[] informationVector() { return informationVector.clone(); }
    public SimpleMatrix informationMatrix() { return informationMatrix.copy(); }

    public Gaussian toGaussian() {
        SimpleMatrix covariance = MatrixOps.inverse(informationMatrix);
        double[] mean = MatrixOps.vector(MatrixOps.solve(informationMatrix, MatrixOps.column(informationVector)));
        return new Gaussian(mean, covariance);
    }
}
