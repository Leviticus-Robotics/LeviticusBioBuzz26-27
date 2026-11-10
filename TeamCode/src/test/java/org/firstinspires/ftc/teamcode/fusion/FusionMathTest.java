package org.firstinspires.ftc.teamcode.fusion;

import static org.junit.jupiter.api.Assertions.*;

import org.ejml.simple.SimpleMatrix;
import org.firstinspires.ftc.teamcode.fusion.core.Gaussian;
import org.firstinspires.ftc.teamcode.fusion.core.GaussianUpdater;
import org.firstinspires.ftc.teamcode.fusion.core.MatrixOps;
import org.firstinspires.ftc.teamcode.fusion.engine.FactorApplicationResult;
import org.junit.jupiter.api.Test;

class FusionMathTest {
    @Test void informationRoundTripWorksForArbitraryDimension() {
        Gaussian original = new Gaussian(new double[] {1, -2, 3, 4},
                MatrixOps.diagonal(.2, .3, .4, .5));
        Gaussian roundTrip = original.toInformation().toGaussian();
        assertArrayEquals(original.mean(), roundTrip.mean(), 1e-10);
        assertTrue(original.covariance().isIdentical(roundTrip.covariance(), 1e-10));
    }

    @Test void partialMeasurementUpdatesCorrelatedDimensions() {
        SimpleMatrix covariance = MatrixOps.diagonal(1, 1);
        covariance.set(0, 1, .5);
        covariance.set(1, 0, .5);
        Gaussian posterior = GaussianUpdater.apply(
                new Gaussian(new double[] {0, 0}, covariance),
                new double[] {-2},
                new SimpleMatrix(new double[][] {{1, 0}}),
                MatrixOps.diagonal(.1));
        assertTrue(posterior.mean(0) > 1);
        assertTrue(posterior.mean(1) > 0);
    }

    @Test void invalidDimensionsAndCovariancesAreRejected() {
        assertThrows(IllegalArgumentException.class,
                () -> GaussianUpdater.apply(
                        new Gaussian(new double[] {0, 0}, MatrixOps.diagonal(1, 1)),
                        new double[] {1}, SimpleMatrix.identity(2), MatrixOps.diagonal(1)));
        assertThrows(IllegalArgumentException.class,
                () -> new Gaussian(new double[] {0, 0}, MatrixOps.diagonal(1)));
        assertThrows(IllegalArgumentException.class,
                () -> new Gaussian(new double[] {0}, MatrixOps.diagonal(Double.NaN)));
    }

    @Test void factorApplicationResultDistinguishesAcceptanceFromRejection() {
        assertEquals(FactorApplicationResult.Status.ACCEPTED,
                FactorApplicationResult.accepted().status);
        assertTrue(FactorApplicationResult.accepted().isAccepted());
        assertEquals(FactorApplicationResult.Status.REJECTED,
                FactorApplicationResult.rejected("invalid sample").status);
        assertFalse(FactorApplicationResult.rejected("invalid sample").isAccepted());
    }

}
