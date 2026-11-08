package org.firstinspires.ftc.teamcode.fusion.core;

import org.ejml.simple.SimpleMatrix;

/** Combines a state Gaussian with a compatible linear Gaussian measurement. */
public final class GaussianFusion {
    private GaussianFusion() { }

    public static Gaussian fuse(Gaussian state, Gaussian measurement,
                                SimpleMatrix stateToMeasurement) {
        if (state == null || measurement == null) throw new IllegalArgumentException("Gaussian is null");
        if (stateToMeasurement == null
                || stateToMeasurement.numRows() != measurement.dimension()
                || stateToMeasurement.numCols() != state.dimension()) {
            throw new IllegalArgumentException(
                    "state-to-measurement dimensions do not match the Gaussians");
        }
        MatrixOps.copy(stateToMeasurement);

        InformationGaussian stateInformation = state.toInformation();
        SimpleMatrix stateMatrix = stateInformation.informationMatrix();
        SimpleMatrix stateVector = MatrixOps.column(stateInformation.informationVector());
        SimpleMatrix measurementInformation = MatrixOps.inverse(measurement.covariance());
        SimpleMatrix measurementVector = measurementInformation.mult(MatrixOps.column(measurement.mean()));
        SimpleMatrix fusedMatrix = stateMatrix.plus(
                stateToMeasurement.transpose().mult(measurementInformation).mult(stateToMeasurement));
        SimpleMatrix fusedVector = stateVector.plus(
                stateToMeasurement.transpose().mult(measurementVector));
        return new InformationGaussian(MatrixOps.vector(fusedVector),
                MatrixOps.symmetrize(fusedMatrix)).toGaussian();
    }
}
