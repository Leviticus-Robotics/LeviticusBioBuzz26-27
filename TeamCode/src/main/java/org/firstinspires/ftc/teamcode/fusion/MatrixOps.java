package org.firstinspires.ftc.teamcode.fusion;

import org.ejml.data.DMatrixRMaj;
import org.ejml.dense.row.factory.DecompositionFactory_DDRM;
import org.ejml.interfaces.decomposition.CholeskyDecomposition_F64;
import org.ejml.simple.SimpleMatrix;

/** Numerical validation and solve helpers for Gaussian fusion. */
public final class MatrixOps {
    private static final double SYMMETRY_TOLERANCE = 1e-9;

    private MatrixOps() { }

    public static SimpleMatrix copy(SimpleMatrix matrix) {
        if (matrix == null) throw new IllegalArgumentException("matrix is null");
        SimpleMatrix result = matrix.copy();
        requireFinite(result, "matrix");
        return result;
    }

    public static double[] copyVector(double[] vector, String name) {
        if (vector == null || vector.length == 0) throw new IllegalArgumentException(name + " is empty");
        double[] result = vector.clone();
        for (double value : result) {
            if (!Double.isFinite(value)) throw new IllegalArgumentException(name + " contains a non-finite value");
        }
        return result;
    }

    public static SimpleMatrix column(double[] values) {
        return new SimpleMatrix(values.length, 1, true, values);
    }

    public static double[] vector(SimpleMatrix column) {
        if (column.numCols() != 1) throw new IllegalArgumentException("expected a column vector");
        double[] result = new double[column.numRows()];
        for (int i = 0; i < result.length; i++) result[i] = column.get(i, 0);
        return result;
    }

    public static SimpleMatrix diagonal(double... diagonal) {
        SimpleMatrix result = new SimpleMatrix(diagonal.length, diagonal.length);
        for (int i = 0; i < diagonal.length; i++) result.set(i, i, diagonal[i]);
        return result;
    }

    public static SimpleMatrix requirePositiveDefinite(SimpleMatrix matrix, int dimension, String name) {
        SimpleMatrix result = copy(matrix);
        if (result.numRows() != dimension || result.numCols() != dimension) {
            throw new IllegalArgumentException(name + " must be " + dimension + "x" + dimension);
        }
        for (int row = 0; row < dimension; row++) {
            for (int col = row + 1; col < dimension; col++) {
                if (Math.abs(result.get(row, col) - result.get(col, row)) > SYMMETRY_TOLERANCE) {
                    throw new IllegalArgumentException(name + " must be symmetric");
                }
            }
        }
        DMatrixRMaj dense = result.getDDRM().copy();
        CholeskyDecomposition_F64<DMatrixRMaj> cholesky = DecompositionFactory_DDRM.chol(dimension, true);
        if (!cholesky.decompose(dense)) throw new IllegalArgumentException(name + " must be positive definite");
        return result;
    }

    public static SimpleMatrix solve(SimpleMatrix matrix, SimpleMatrix rightHandSide) {
        try {
            return matrix.solve(rightHandSide);
        } catch (RuntimeException exception) {
            throw new IllegalArgumentException("matrix is not solvable", exception);
        }
    }

    public static SimpleMatrix inverse(SimpleMatrix matrix) {
        if (matrix.numRows() != matrix.numCols()) throw new IllegalArgumentException("matrix must be square");
        return solve(matrix, SimpleMatrix.identity(matrix.numRows()));
    }

    public static SimpleMatrix symmetrize(SimpleMatrix matrix) {
        return matrix.plus(matrix.transpose()).scale(0.5);
    }

    private static void requireFinite(SimpleMatrix matrix, String name) {
        for (int row = 0; row < matrix.numRows(); row++) {
            for (int col = 0; col < matrix.numCols(); col++) {
                if (!Double.isFinite(matrix.get(row, col))) {
                    throw new IllegalArgumentException(name + " contains a non-finite value");
                }
            }
        }
    }
}
