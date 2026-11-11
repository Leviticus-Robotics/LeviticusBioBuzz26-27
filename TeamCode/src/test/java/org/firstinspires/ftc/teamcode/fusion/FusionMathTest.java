package org.firstinspires.ftc.teamcode.fusion;

import static org.junit.jupiter.api.Assertions.*;

import org.ejml.simple.SimpleMatrix;
import org.firstinspires.ftc.teamcode.fusion.core.Gaussian;
import org.firstinspires.ftc.teamcode.fusion.core.GaussianUpdater;
import org.firstinspires.ftc.teamcode.fusion.core.GaussianFusion;
import org.firstinspires.ftc.teamcode.fusion.core.MatrixOps;
import org.junit.jupiter.api.Test;

class FusionMathTest {
    @Test void informationRoundTripWorksForArbitraryDimension() {
        Gaussian original = new Gaussian(new double[] {1, -2, 3, 4},
                MatrixOps.diagonal(.2, .3, .4, .5));
        Gaussian roundTrip = original.toInformation().toGaussian();
        assertArrayEquals(original.mean(), roundTrip.mean(), 1e-10);
        assertTrue(original.covariance().isIdentical(roundTrip.covariance(), 1e-10));
    }

    @Test void GaussianCombinationPoliciesRemainDistinct() {
        java.util.List<Gaussian> observations = java.util.Arrays.asList(
                new Gaussian(new double[] {0}, MatrixOps.diagonal(1)),
                new Gaussian(new double[] {2}, MatrixOps.diagonal(1)));

        Gaussian product = GaussianFusion.product(observations);
        Gaussian average = GaussianFusion.informationAverage(observations);

        assertEquals(1, product.mean(0), 1e-12);
        assertEquals(.5, product.covariance().get(0), 1e-12);
        assertEquals(1, average.mean(0), 1e-12);
        assertEquals(1, average.covariance().get(0), 1e-12);
        assertEquals(1, GaussianFusion.disagreementCovariance(
                observations, average.mean()).get(0), 1e-12);
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

}
