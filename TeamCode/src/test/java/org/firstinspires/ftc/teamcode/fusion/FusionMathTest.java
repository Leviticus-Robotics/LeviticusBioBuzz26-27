package org.firstinspires.ftc.teamcode.fusion;

import static org.junit.jupiter.api.Assertions.*;

import org.ejml.simple.SimpleMatrix;
import org.firstinspires.ftc.teamcode.fusion.core.Gaussian;
import org.firstinspires.ftc.teamcode.fusion.core.GaussianFusion;
import org.firstinspires.ftc.teamcode.fusion.core.MatrixOps;
import org.firstinspires.ftc.teamcode.fusion.engine.FactorApplicationResult;
import org.firstinspires.ftc.teamcode.fusion.factor.MeasurementFactor;
import org.firstinspires.ftc.teamcode.fusion.factor.RelativeFactor;
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
        Gaussian posterior = GaussianFusion.fuse(new Gaussian(new double[] {0, 0}, covariance),
                new Gaussian(new double[] {2}, MatrixOps.diagonal(.1)),
                new SimpleMatrix(new double[][] {{1, 0}}));
        assertTrue(posterior.mean(0) > 1);
        assertTrue(posterior.mean(1) > 0);
    }

    @Test void invalidDimensionsAndCovariancesAreRejected() {
        Gaussian state = new Gaussian(new double[] {0, 0}, MatrixOps.diagonal(1, 1));
        Gaussian observation = new Gaussian(new double[] {1}, MatrixOps.diagonal(1));
        assertThrows(IllegalArgumentException.class,
                () -> GaussianFusion.fuse(state, observation, SimpleMatrix.identity(2)));
        assertThrows(IllegalArgumentException.class,
                () -> new Gaussian(new double[] {0, 0}, MatrixOps.diagonal(1)));
        assertThrows(IllegalArgumentException.class,
                () -> new Gaussian(new double[] {0}, MatrixOps.diagonal(Double.NaN)));
    }

    @Test void measurementProvidesDefaultApplicationAndRelativeFactorAppliesDirectly() {
        Gaussian state = new Gaussian(new double[] {0, 0}, MatrixOps.diagonal(1, 1));
        MeasurementFactor measurement = new ScalarMeasurement(0, 3, .01);
        assertTrue(measurement.apply(state).mean(0) > 2.9);
        RelativeFactor transition = new DeltaTransition(0, 1, 2);
        assertEquals(2, transition.apply(state).mean(0), 1e-12);
    }

    @Test void factorApplicationResultDistinguishesAcceptanceFromRejection() {
        assertEquals(FactorApplicationResult.Status.ACCEPTED,
                FactorApplicationResult.accepted().status);
        assertTrue(FactorApplicationResult.accepted().isAccepted());
        assertEquals(FactorApplicationResult.Status.REJECTED,
                FactorApplicationResult.rejected("invalid sample").status);
        assertFalse(FactorApplicationResult.rejected("invalid sample").isAccepted());
    }

    private static final class ScalarMeasurement implements MeasurementFactor {
        private final long timestamp;
        private final double value;
        private final double variance;
        ScalarMeasurement(long timestamp, double value, double variance) {
            this.timestamp = timestamp; this.value = value; this.variance = variance;
        }
        @Override public String source() { return "position"; }
        @Override public long timestampNanos() { return timestamp; }
        @Override public Gaussian measurement(Gaussian state) {
            return new Gaussian(new double[] {value}, MatrixOps.diagonal(variance));
        }
        @Override public SimpleMatrix stateToMeasurement(Gaussian state) {
            return new SimpleMatrix(new double[][] {{1, 0}});
        }
    }

    private static final class DeltaTransition implements RelativeFactor {
        private final long from;
        private final long to;
        private final double delta;
        DeltaTransition(long from, long to, double delta) { this.from = from; this.to = to; this.delta = delta; }
        @Override public String source() { return "delta"; }
        @Override public long fromTimestampNanos() { return from; }
        @Override public long timestampNanos() { return to; }
        @Override public Gaussian apply(Gaussian state) {
            double dt = (to - from) * 1e-9;
            return new Gaussian(new double[] {state.mean(0) + delta, delta / dt},
                    state.covariance().plus(MatrixOps.diagonal(.01, .01)));
        }
    }
}
