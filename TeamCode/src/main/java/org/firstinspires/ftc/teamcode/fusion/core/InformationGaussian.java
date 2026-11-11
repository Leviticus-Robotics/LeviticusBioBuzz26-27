package org.firstinspires.ftc.teamcode.fusion.core;

import org.ejml.simple.SimpleMatrix;

/** Immutable information-vector/information-matrix Gaussian representation. */
public final class InformationGaussian {
    private final double[] vector;
    private final SimpleMatrix matrix;

    public InformationGaussian(double[] vector, SimpleMatrix matrix) {
        this.vector = MatrixOps.copyVector(vector, "information vector");
        this.matrix = MatrixOps.requirePositiveDefinite(
                matrix, vector.length, "information matrix");
    }

    public static InformationGaussian from(Gaussian gaussian) {
        SimpleMatrix matrix = MatrixOps.inverse(gaussian.covariance());
        return new InformationGaussian(MatrixOps.vector(
                matrix.mult(MatrixOps.column(gaussian.mean()))), matrix);
    }

    public InformationGaussian plus(InformationGaussian other) {
        requireDimension(other);
        return new InformationGaussian(
                MatrixOps.vector(MatrixOps.column(vector).plus(MatrixOps.column(other.vector))),
                matrix.plus(other.matrix));
    }

    public InformationGaussian scale(double weight) {
        if (!Double.isFinite(weight) || weight <= 0) {
            throw new IllegalArgumentException("information weight must be positive and finite");
        }
        return new InformationGaussian(
                MatrixOps.vector(MatrixOps.column(vector).scale(weight)),
                matrix.scale(weight));
    }

    public Gaussian toGaussian() {
        SimpleMatrix covariance = MatrixOps.inverse(matrix);
        return new Gaussian(MatrixOps.vector(MatrixOps.solve(
                matrix, MatrixOps.column(vector))), covariance);
    }

    public int dimension() { return vector.length; }
    public double[] vector() { return vector.clone(); }
    public SimpleMatrix matrix() { return matrix.copy(); }

    private void requireDimension(InformationGaussian other) {
        if (other == null || other.dimension() != dimension()) {
            throw new IllegalArgumentException("information dimensions do not match");
        }
    }
}
