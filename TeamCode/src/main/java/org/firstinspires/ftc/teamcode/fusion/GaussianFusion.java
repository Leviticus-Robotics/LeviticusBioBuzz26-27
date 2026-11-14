package org.firstinspires.ftc.teamcode.fusion;

import java.util.List;
import org.ejml.simple.SimpleMatrix;

/** Reusable policies for combining Gaussian estimates. */
public final class GaussianFusion {
    private GaussianFusion() { }

    /** Product of statistically independent Gaussian densities. */
    public static Gaussian product(List<Gaussian> gaussians) {
        requireNonempty(gaussians);
        return sumInformation(gaussians).toGaussian();
    }

    /** One-solve Kalman-equivalent product of two independent Gaussian densities. */
    public static Gaussian product(Gaussian first, Gaussian second) {
        if (first == null || second == null
                || first.dimension() != second.dimension()) {
            throw new IllegalArgumentException("Gaussian dimensions do not match");
        }
        SimpleMatrix firstCovariance = first.covariance();
        SimpleMatrix secondCovariance = second.covariance();
        SimpleMatrix innovationCovariance = firstCovariance.plus(secondCovariance);
        SimpleMatrix gain = MatrixOps.solve(
                innovationCovariance, firstCovariance.transpose()).transpose();
        SimpleMatrix residual = MatrixOps.column(second.mean())
                .minus(MatrixOps.column(first.mean()));
        double[] mean = MatrixOps.vector(
                MatrixOps.column(first.mean()).plus(gain.mult(residual)));

        SimpleMatrix identityMinusGain = SimpleMatrix.identity(first.dimension()).minus(gain);
        SimpleMatrix covariance = identityMinusGain.mult(firstCovariance)
                .mult(identityMinusGain.transpose())
                .plus(gain.mult(secondCovariance).mult(gain.transpose()));
        return new Gaussian(mean, MatrixOps.symmetrize(covariance));
    }

    /** Equal-weight logarithmic opinion pool for correlated observations. */
    public static Gaussian informationAverage(List<Gaussian> gaussians) {
        requireNonempty(gaussians);
        return sumInformation(gaussians).scale(1.0/gaussians.size()).toGaussian();
    }

    /** Empirical covariance of Gaussian means around a supplied center. */
    public static SimpleMatrix disagreementCovariance(
            List<Gaussian> gaussians, double[] center) {
        requireNonempty(gaussians);
        int dimension = gaussians.get(0).dimension();
        if (center == null || center.length != dimension) {
            throw new IllegalArgumentException("center dimension does not match Gaussians");
        }
        SimpleMatrix result = new SimpleMatrix(dimension, dimension);
        SimpleMatrix centerColumn = MatrixOps.column(center);
        for (Gaussian gaussian : gaussians) {
            if (gaussian.dimension() != dimension) {
                throw new IllegalArgumentException("Gaussian dimensions do not match");
            }
            SimpleMatrix residual = MatrixOps.column(gaussian.mean()).minus(centerColumn);
            result = result.plus(residual.mult(residual.transpose()));
        }
        return MatrixOps.symmetrize(result.scale(1.0/gaussians.size()));
    }

    private static InformationGaussian sumInformation(List<Gaussian> gaussians) {
        requireNonempty(gaussians);
        InformationGaussian result = InformationGaussian.from(gaussians.get(0));
        for (int i = 1; i < gaussians.size(); i++) {
            result = result.plus(InformationGaussian.from(gaussians.get(i)));
        }
        return result;
    }

    private static void requireNonempty(List<Gaussian> gaussians) {
        if (gaussians == null || gaussians.isEmpty()) {
            throw new IllegalArgumentException("Gaussian list is empty");
        }
    }
}
