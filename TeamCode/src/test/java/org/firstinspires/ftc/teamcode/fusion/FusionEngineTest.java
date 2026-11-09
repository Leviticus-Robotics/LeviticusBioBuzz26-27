package org.firstinspires.ftc.teamcode.fusion;

import static org.junit.jupiter.api.Assertions.*;

import java.util.Arrays;
import org.ejml.simple.SimpleMatrix;
import org.firstinspires.ftc.teamcode.fusion.core.Gaussian;
import org.firstinspires.ftc.teamcode.fusion.core.MatrixOps;
import org.firstinspires.ftc.teamcode.fusion.engine.FactorApplicationResult;
import org.firstinspires.ftc.teamcode.fusion.engine.FusionEngine;
import org.firstinspires.ftc.teamcode.fusion.engine.ProcessModel;
import org.firstinspires.ftc.teamcode.fusion.factor.Factor;
import org.firstinspires.ftc.teamcode.fusion.factor.BetweenFactor;
import org.firstinspires.ftc.teamcode.fusion.factor.LinearizedFactor;
import org.firstinspires.ftc.teamcode.fusion.factor.StateValues;
import org.junit.jupiter.api.Test;

/** Reuse example for a mechanism state [position, velocity], with no localization types. */
class FusionEngineTest {
    private static final long SECOND = 1_000_000_000L;

    private FusionEngine<MotorCommand> engine(long historyNanos) {
        return new FusionEngine<>(new Gaussian(new double[] {0, 0}, MatrixOps.diagonal(1, 1)), 0,
                new MechanismProcess(), new MotorCommand(0), historyNanos);
    }

    @Test void predictsWithDomainInputAndFusesScalarMeasurement() {
        FusionEngine<MotorCommand> engine = engine(10 * SECOND);
        engine.setInput(0, new MotorCommand(2));
        engine.predictTo(SECOND);
        assertEquals(1, engine.state().mean(0), 1e-12);
        assertEquals(2, engine.state().mean(1), 1e-12);
        assertTrue(engine.applyFactor(new PositionMeasurement(SECOND, 3, .001)).isAccepted());
        assertTrue(engine.state().mean(0) > 2.9);
    }

    @Test void relativeTransitionUsesRetainedStateAndDelayedReplayMatchesInOrder() {
        PositionMeasurement correction = new PositionMeasurement(SECOND, 5, .01);
        MeasuredTransition transition = new MeasuredTransition(SECOND, 2 * SECOND, 2, .01);

        FusionEngine<MotorCommand> inOrder = engine(10 * SECOND);
        inOrder.predictTo(SECOND);
        assertTrue(inOrder.applyFactor(correction).isAccepted());
        inOrder.predictTo(2 * SECOND);
        assertTrue(inOrder.applyFactor(transition).isAccepted());

        FusionEngine<MotorCommand> delayed = engine(10 * SECOND);
        delayed.predictTo(SECOND);
        delayed.predictTo(2 * SECOND);
        assertTrue(delayed.applyFactor(transition).isAccepted());
        assertTrue(delayed.applyFactor(correction).isAccepted());

        assertArrayEquals(inOrder.state().mean(), delayed.state().mean(), 1e-10);
        assertTrue(inOrder.state().covariance().isIdentical(delayed.state().covariance(), 1e-10));
    }

    @Test void orderingIsInputThenRelativeThenAbsoluteThenTick() {
        FusionEngine<MotorCommand> engine = engine(10 * SECOND);
        engine.predictTo(SECOND);
        engine.applyFactor(new PositionMeasurement(SECOND, 10, .001));
        engine.applyFactor(new MeasuredTransition(0, SECOND, 2, .001));
        engine.setInput(SECOND, new MotorCommand(4));
        assertTrue(engine.state().mean(0) > 9.9,
                "absolute factor must run after the relative factor at the same timestamp");
        engine.predictTo(2 * SECOND);
        assertTrue(engine.state().mean(0) > 11);
    }

    @Test void resetStatePreservesActiveInput() {
        FusionEngine<MotorCommand> engine = engine(10 * SECOND);
        engine.setInput(0, new MotorCommand(2));
        engine.resetState(new Gaussian(new double[] {10, 0}, MatrixOps.diagonal(1, 1)), SECOND);
        engine.predictTo(2 * SECOND);
        assertEquals(11, engine.state().mean(0), 1e-12);
        assertEquals(2, engine.state().mean(1), 1e-12);
    }

    @Test void delayedFactorDoesNotMoveLatestSourceTimestampBackward() {
        FusionEngine<MotorCommand> engine = engine(10 * SECOND);
        engine.predictTo(SECOND);
        engine.predictTo(2 * SECOND);
        assertTrue(engine.applyFactor(new PositionMeasurement(2 * SECOND, 2, .1)).isAccepted());
        assertTrue(engine.applyFactor(new PositionMeasurement(SECOND, 1, .1)).isAccepted());
        assertEquals(Long.valueOf(2 * SECOND),
                engine.diagnostics().latestSourceTimestamps.get("position-sensor"));
    }

    @Test void invalidAndStaleFactorsAreRejectedAtomicallyAndHistoryIsBounded() {
        FusionEngine<MotorCommand> engine = engine(2 * SECOND);
        engine.predictTo(SECOND);
        engine.predictTo(2 * SECOND);
        engine.predictTo(3 * SECOND);
        engine.predictTo(4 * SECOND);
        assertEquals(Arrays.asList(2 * SECOND, 3 * SECOND, 4 * SECOND), engine.retainedEventTimestamps());
        double[] before = engine.state().mean();
        FactorApplicationResult stale = engine.applyFactor(new PositionMeasurement(0, 100, .01));
        assertFalse(stale.isAccepted());
        assertArrayEquals(before, engine.state().mean(), 0);
        Factor invalid = new Factor() {
            @Override public String source() { return "invalid"; }
            @Override public long[] timestamps() { return new long[] {4 * SECOND}; }
            @Override public double[] residual(StateValues values) { return new double[] {0}; }
            @Override public SimpleMatrix covariance(StateValues values) { return MatrixOps.diagonal(1); }
            @Override public LinearizedFactor linearize(StateValues values) {
                throw new IllegalArgumentException("custom factor rejected its input");
            }
        };
        assertFalse(engine.applyFactor(invalid).isAccepted());
        assertArrayEquals(before, engine.state().mean(), 0);
    }

    @Test void customFactorOwnsItsApplicationWithoutEngineTypeDispatch() {
        FusionEngine<MotorCommand> engine = engine(10 * SECOND);
        Factor custom = new PositionMeasurement(SECOND, 7, .0001);

        assertTrue(engine.applyFactor(custom).isAccepted());
        assertEquals(7, engine.state().mean(0), .001);
    }

    @Test void chronologicalEventsAvoidRecomputationUntilDelayedDataArrives() {
        FusionEngine<MotorCommand> engine = engine(10 * SECOND);
        engine.predictTo(SECOND);
        assertTrue(engine.applyFactor(
                new PositionMeasurement(2 * SECOND, 2, .1)).isAccepted());
        assertEquals(0, engine.diagnostics().recomputationCount);

        assertTrue(engine.applyFactor(
                new PositionMeasurement(SECOND, 1, .1)).isAccepted());
        assertEquals(1, engine.diagnostics().recomputationCount);
    }

    @Test void futureRelativeFactorCreatesItsEndpointStates() {
        FusionEngine<MotorCommand> engine = engine(10 * SECOND);
        double[] before = engine.state().mean();

        FactorApplicationResult result = engine.applyFactor(
                new MeasuredTransition(SECOND / 2, SECOND, 1, .1));

        assertTrue(result.isAccepted(), result.reason);
        assertEquals(SECOND, engine.timestampNanos());
        assertFalse(Arrays.equals(before, engine.state().mean()));
    }

    private static final class MotorCommand {
        final double acceleration;
        MotorCommand(double acceleration) { this.acceleration = acceleration; }
    }

    private static final class MechanismProcess implements ProcessModel<MotorCommand> {
        @Override public double[] predictMean(double[] prior, MotorCommand input, double dt) {
            return new double[] {
                    prior[0] + prior[1]*dt + .5*input.acceleration*dt*dt,
                    prior[1] + input.acceleration*dt
            };
        }
        @Override public SimpleMatrix processNoise(double[] prior, MotorCommand input, double dt) {
            return MatrixOps.diagonal(.001*Math.max(dt, 1e-9), .001*Math.max(dt, 1e-9));
        }
    }

    private static final class PositionMeasurement implements Factor {
        final long timestamp;
        final double position;
        final double variance;
        PositionMeasurement(long timestamp, double position, double variance) {
            this.timestamp = timestamp; this.position = position; this.variance = variance;
        }
        @Override public String source() { return "position-sensor"; }
        @Override public long timestampNanos() { return timestamp; }
        @Override public long[] timestamps() { return new long[] {timestamp}; }
        @Override public double[] residual(StateValues values) {
            return new double[] {values.state(timestamp)[0] - position};
        }
        @Override public SimpleMatrix covariance(StateValues values) {
            return MatrixOps.diagonal(variance);
        }
    }

    private static final class MeasuredTransition implements BetweenFactor {
        final long from;
        final long to;
        final double delta;
        final double variance;
        MeasuredTransition(long from, long to, double delta, double variance) {
            this.from = from; this.to = to; this.delta = delta; this.variance = variance;
        }
        @Override public String source() { return "incremental-encoder"; }
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
        @Override public SimpleMatrix covariance(StateValues values) {
            double dt = (to - from)*1e-9;
            return MatrixOps.diagonal(variance, variance/(dt*dt));
        }
    }
}
