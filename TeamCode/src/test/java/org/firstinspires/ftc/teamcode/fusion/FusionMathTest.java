package org.firstinspires.ftc.teamcode.fusion;

import static org.junit.jupiter.api.Assertions.*;

import org.ejml.simple.SimpleMatrix;
import org.firstinspires.ftc.teamcode.fusion.core.Gaussian;
import org.firstinspires.ftc.teamcode.fusion.core.GaussianFusion;
import org.firstinspires.ftc.teamcode.fusion.core.MatrixOps;
import org.firstinspires.ftc.teamcode.fusion.engine.FactorApplicationResult;
import org.firstinspires.ftc.teamcode.fusion.factor.BetweenFactor;
import org.firstinspires.ftc.teamcode.fusion.factor.Factor;
import org.firstinspires.ftc.teamcode.fusion.factor.StateValues;
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

    @Test void unaryAndBetweenFactorsUseTheSameResidualInterface() {
        Factor measurement = new ScalarMeasurement(0, 3, .01);
        assertEquals(0, measurement.residual(values(0, new double[] {3, 0}))[0], 1e-12);
        Factor transition = new DeltaTransition(0, 1, 2);
        assertEquals(0, transition.residual(values(
                0, new double[] {0, 0}, 1, new double[] {2, 2e9}))[0], 1e-12);
    }

    @Test void factorApplicationResultDistinguishesAcceptanceFromRejection() {
        assertEquals(FactorApplicationResult.Status.ACCEPTED,
                FactorApplicationResult.accepted().status);
        assertTrue(FactorApplicationResult.accepted().isAccepted());
        assertEquals(FactorApplicationResult.Status.REJECTED,
                FactorApplicationResult.rejected("invalid sample").status);
        assertFalse(FactorApplicationResult.rejected("invalid sample").isAccepted());
    }

    private static final class ScalarMeasurement implements Factor {
        private final long timestamp;
        private final double value;
        private final double variance;
        ScalarMeasurement(long timestamp, double value, double variance) {
            this.timestamp = timestamp; this.value = value; this.variance = variance;
        }
        @Override public String source() { return "position"; }
        @Override public long timestampNanos() { return timestamp; }
        @Override public long[] timestamps() { return new long[] {timestamp}; }
        @Override public double[] residual(StateValues values) {
            return new double[] {values.state(timestamp)[0] - value};
        }
        @Override public SimpleMatrix covariance(StateValues values) { return MatrixOps.diagonal(variance); }
    }

    private static final class DeltaTransition implements BetweenFactor {
        private final long from;
        private final long to;
        private final double delta;
        DeltaTransition(long from, long to, double delta) { this.from = from; this.to = to; this.delta = delta; }
        @Override public String source() { return "delta"; }
        @Override public long fromTimestampNanos() { return from; }
        @Override public long toTimestampNanos() { return to; }
        @Override public double[] residual(StateValues values) {
            double[] fromState = values.state(from);
            double[] toState = values.state(to);
            double dt = (to - from) * 1e-9;
            return new double[] {
                    toState[0] - fromState[0] - delta,
                    toState[1] - delta/dt
            };
        }
        @Override public SimpleMatrix covariance(StateValues values) { return MatrixOps.diagonal(.01, .01); }
    }

    private static StateValues values(long timestamp, double[] state) {
        return values(timestamp, state, Long.MIN_VALUE, null);
    }

    private static StateValues values(long firstTimestamp, double[] firstState,
                                      long secondTimestamp, double[] secondState) {
        return new StateValues() {
            @Override public double[] state(long timestamp) {
                if (timestamp == firstTimestamp) return firstState.clone();
                if (secondState != null && timestamp == secondTimestamp) return secondState.clone();
                throw new IllegalArgumentException("unknown timestamp");
            }
        };
    }
}
