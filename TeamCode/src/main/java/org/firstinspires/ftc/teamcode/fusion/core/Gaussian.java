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

    public static Gaussian fromInformation(double[] vector, SimpleMatrix matrix) {
        return new InformationGaussian(vector, matrix).toGaussian();
    }

    public InformationGaussian toInformation() { return InformationGaussian.from(this); }
}
