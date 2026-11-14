package org.firstinspires.ftc.teamcode.fusion;

import static org.junit.jupiter.api.Assertions.*;

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

    @Test void invalidDimensionsAndCovariancesAreRejected() {
        assertThrows(IllegalArgumentException.class,
                () -> new Gaussian(new double[] {0, 0}, MatrixOps.diagonal(1)));
        assertThrows(IllegalArgumentException.class,
                () -> new Gaussian(
                        new double[] {0}, MatrixOps.diagonal(Double.NaN)).validate());
    }

}
