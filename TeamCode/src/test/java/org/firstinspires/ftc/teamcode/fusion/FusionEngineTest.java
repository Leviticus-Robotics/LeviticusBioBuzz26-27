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
import org.firstinspires.ftc.teamcode.fusion.factor.AbsoluteFactor;
import org.firstinspires.ftc.teamcode.fusion.factor.MeasurementFactor;
import org.firstinspires.ftc.teamcode.fusion.factor.RelativeFactor;
import org.junit.jupiter.api.Test;

/** Reuse example for a mechanism state [position, velocity], with no localization types. */
class FusionEngineTest {
    private static final long SECOND = 1_000_000_000L;

    private FusionEngine<MotorCommand> engine(long historyNanos, int maximumEvents) {
        return new FusionEngine<>(new Gaussian(new double[] {0, 0}, MatrixOps.diagonal(1, 1)), 0,
                new MechanismProcess(), new MotorCommand(0), historyNanos, maximumEvents);
    }

    @Test void predictsWithDomainInputAndFusesScalarMeasurement() {
        FusionEngine<MotorCommand> engine = engine(10 * SECOND, 100);
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

        FusionEngine<MotorCommand> inOrder = engine(10 * SECOND, 100);
        inOrder.predictTo(SECOND);
        assertTrue(inOrder.applyFactor(correction).isAccepted());
        inOrder.predictTo(2 * SECOND);
        assertTrue(inOrder.applyFactor(transition).isAccepted());

        FusionEngine<MotorCommand> delayed = engine(10 * SECOND, 100);
        delayed.predictTo(SECOND);
        delayed.predictTo(2 * SECOND);
        assertTrue(delayed.applyFactor(transition).isAccepted());
        assertTrue(delayed.applyFactor(correction).isAccepted());

        assertArrayEquals(inOrder.state().mean(), delayed.state().mean(), 1e-10);
        assertTrue(inOrder.state().covariance().isIdentical(delayed.state().covariance(), 1e-10));
    }

    @Test void orderingIsInputThenRelativeThenAbsoluteThenTick() {
        FusionEngine<MotorCommand> engine = engine(10 * SECOND, 100);
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
        FusionEngine<MotorCommand> engine = engine(10 * SECOND, 100);
        engine.setInput(0, new MotorCommand(2));
        engine.resetState(new Gaussian(new double[] {10, 0}, MatrixOps.diagonal(1, 1)), SECOND);
        engine.predictTo(2 * SECOND);
        assertEquals(11, engine.state().mean(0), 1e-12);
        assertEquals(2, engine.state().mean(1), 1e-12);
    }

    @Test void delayedFactorDoesNotMoveLatestSourceTimestampBackward() {
        FusionEngine<MotorCommand> engine = engine(10 * SECOND, 100);
        engine.predictTo(SECOND);
        engine.predictTo(2 * SECOND);
        assertTrue(engine.applyFactor(new PositionMeasurement(2 * SECOND, 2, .1)).isAccepted());
        assertTrue(engine.applyFactor(new PositionMeasurement(SECOND, 1, .1)).isAccepted());
        assertEquals(Long.valueOf(2 * SECOND),
                engine.diagnostics().latestSourceTimestamps.get("position-sensor"));
    }

    @Test void invalidAndStaleFactorsAreRejectedAtomicallyAndHistoryIsBounded() {
        FusionEngine<MotorCommand> engine = engine(2 * SECOND, 3);
        engine.predictTo(SECOND);
        engine.predictTo(2 * SECOND);
        engine.predictTo(3 * SECOND);
        engine.predictTo(4 * SECOND);
        assertEquals(Arrays.asList(2 * SECOND, 3 * SECOND, 4 * SECOND), engine.retainedEventTimestamps());
        double[] before = engine.state().mean();
        FactorApplicationResult stale = engine.applyFactor(new PositionMeasurement(0, 100, .01));
        assertFalse(stale.isAccepted());
        assertArrayEquals(before, engine.state().mean(), 0);
        Factor unsupported = new Factor() {
            @Override public String source() { return "unsupported"; }
            @Override public long timestampNanos() { return 4 * SECOND; }
        };
        assertFalse(engine.applyFactor(unsupported).isAccepted());
        assertArrayEquals(before, engine.state().mean(), 0);
        AbsoluteFactor wrongDimension = new AbsoluteFactor() {
            @Override public String source() { return "wrong-dimension"; }
            @Override public long timestampNanos() { return 4 * SECOND; }
            @Override public Gaussian apply(Gaussian state) {
                return new Gaussian(new double[] {1}, MatrixOps.diagonal(1));
            }
        };
        assertFalse(engine.applyFactor(wrongDimension).isAccepted());
        assertArrayEquals(before, engine.state().mean(), 0);
    }

    @Test void chronologicalEventsAvoidRecomputationUntilDelayedDataArrives() {
        FusionEngine<MotorCommand> engine = engine(10 * SECOND, 100);
        engine.predictTo(SECOND);
        assertTrue(engine.applyFactor(
                new PositionMeasurement(2 * SECOND, 2, .1)).isAccepted());
        assertEquals(0, engine.diagnostics().recomputationCount);
        assertEquals(2, engine.diagnostics().processModelEvaluationCount);

        assertTrue(engine.applyFactor(
                new PositionMeasurement(SECOND, 1, .1)).isAccepted());
        assertEquals(1, engine.diagnostics().recomputationCount);
    }

    @Test void rejectedFutureRelativeFactorDoesNotAdvanceStateOrHistory() {
        FusionEngine<MotorCommand> engine = engine(10 * SECOND, 100);
        double[] before = engine.state().mean();

        FactorApplicationResult result = engine.applyFactor(
                new MeasuredTransition(SECOND / 2, SECOND, 1, .1));

        assertFalse(result.isAccepted());
        assertEquals(0, engine.timestampNanos());
        assertArrayEquals(before, engine.state().mean(), 0);
        assertTrue(engine.retainedEventTimestamps().isEmpty());
    }

    private static final class MotorCommand {
        final double acceleration;
        MotorCommand(double acceleration) { this.acceleration = acceleration; }
    }

    private static final class MechanismProcess implements ProcessModel<MotorCommand> {
        @Override public Gaussian predict(Gaussian prior, MotorCommand input, double dt) {
            double position = prior.mean(0) + prior.mean(1) * dt + .5 * input.acceleration * dt * dt;
            double velocity = prior.mean(1) + input.acceleration * dt;
            SimpleMatrix f = new SimpleMatrix(new double[][] {{1, dt}, {0, 1}});
            SimpleMatrix covariance = f.mult(prior.covariance()).mult(f.transpose())
                    .plus(MatrixOps.diagonal(.001 * dt, .001 * dt));
            return new Gaussian(new double[] {position, velocity}, covariance);
        }
    }

    private static final class PositionMeasurement implements MeasurementFactor {
        final long timestamp;
        final double position;
        final double variance;
        PositionMeasurement(long timestamp, double position, double variance) {
            this.timestamp = timestamp; this.position = position; this.variance = variance;
        }
        @Override public String source() { return "position-sensor"; }
        @Override public long timestampNanos() { return timestamp; }
        @Override public Gaussian measurement(Gaussian state) {
            return new Gaussian(new double[] {position}, MatrixOps.diagonal(variance));
        }
        @Override public SimpleMatrix stateToMeasurement(Gaussian state) {
            return new SimpleMatrix(new double[][] {{1, 0}});
        }
    }

    private static final class MeasuredTransition implements RelativeFactor {
        final long from;
        final long to;
        final double delta;
        final double variance;
        MeasuredTransition(long from, long to, double delta, double variance) {
            this.from = from; this.to = to; this.delta = delta; this.variance = variance;
        }
        @Override public String source() { return "incremental-encoder"; }
        @Override public long fromTimestampNanos() { return from; }
        @Override public long timestampNanos() { return to; }
        @Override public Gaussian apply(Gaussian state) {
            double dt = (to - from) * 1e-9;
            return new Gaussian(new double[] {state.mean(0) + delta, delta / dt},
                    state.covariance().plus(MatrixOps.diagonal(variance, variance / (dt * dt))));
        }
    }
}
