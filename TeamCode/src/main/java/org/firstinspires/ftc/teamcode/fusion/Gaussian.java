package org.firstinspires.ftc.teamcode.fusion;

import org.ejml.simple.SimpleMatrix;

/** Immutable mean/covariance representation of a multivariate Gaussian. */
public final class Gaussian {
    private final double[] mean;
    private final SimpleMatrix covariance;

    public Gaussian(double[] mean, SimpleMatrix covariance) {
        if (mean == null || mean.length == 0 || covariance == null
                || covariance.numRows() != mean.length
                || covariance.numCols() != mean.length) {
            throw new IllegalArgumentException("Gaussian dimensions do not match");
        }
        this.mean = mean.clone();
        this.covariance = covariance.copy();
    }

    /** Performs finite-value and positive-definite checks at an external boundary. */
    public Gaussian validate() {
        MatrixOps.copyVector(mean, "mean");
        MatrixOps.requirePositiveDefinite(covariance, mean.length, "covariance");
        return this;
    }

    public int dimension() { return mean.length; }
    public double mean(int index) { return mean[index]; }
    public double[] mean() { return mean.clone(); }
    public SimpleMatrix covariance() { return covariance.copy(); }

    public static Gaussian fromInformation(double[] vector, SimpleMatrix matrix) {
        return new InformationGaussian(vector, matrix).toGaussian();
    }

    public InformationGaussian toInformation() { return InformationGaussian.from(this); }
}
