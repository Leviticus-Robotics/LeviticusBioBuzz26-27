package org.firstinspires.ftc.teamcode.fusion.core;

import org.ejml.simple.SimpleMatrix;

/** Immutable mean/covariance representation of a multivariate Gaussian. */
public final class Gaussian {
    private final double[] mean;
    private final SimpleMatrix covariance;

    public Gaussian(double[] mean, SimpleMatrix covariance) {
        this.mean = MatrixOps.copyVector(mean, "mean");
        this.covariance = MatrixOps.requirePositiveDefinite(covariance, mean.length, "covariance");
    }

    public int dimension() { return mean.length; }
    public double mean(int index) { return mean[index]; }
    public double[] mean() { return mean.clone(); }
    public SimpleMatrix covariance() { return covariance.copy(); }

    public InformationGaussian toInformation() {
        SimpleMatrix information = MatrixOps.inverse(covariance);
        return new InformationGaussian(MatrixOps.vector(information.mult(MatrixOps.column(mean))), information);
    }
}
