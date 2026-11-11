package org.firstinspires.ftc.teamcode.fusion;

import static org.junit.jupiter.api.Assertions.assertArrayEquals;
import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;

import java.util.Collections;
import java.util.Arrays;
import org.ejml.simple.SimpleMatrix;
import org.firstinspires.ftc.teamcode.fusion.core.Gaussian;
import org.firstinspires.ftc.teamcode.fusion.core.MatrixOps;
import org.firstinspires.ftc.teamcode.fusion.engine.FusionEngine;
import org.firstinspires.ftc.teamcode.fusion.engine.ProcessModel;
import org.firstinspires.ftc.teamcode.fusion.factor.RelativeFactor;
import org.firstinspires.ftc.teamcode.fusion.factor.Factor;
import org.firstinspires.ftc.teamcode.fusion.factor.LinearizedFactor;
import org.firstinspires.ftc.teamcode.fusion.factor.StateValues;
import org.junit.jupiter.api.Test;

class FusionEngineTest {
    private static final long SECOND = 1_000_000_000L;

    @Test void engineIsGenericOverStateMeaningAndFactorType() {
        FusionEngine engine = engine();
        engine.commitTick(SECOND, Arrays.asList(
                model().factor(0, SECOND, 2.0), new DeltaFactor(0, SECOND, 3)));
        assertEquals(3, engine.state().mean(0), .01);

        assertTrue(engine.submitFactor(new PositionFactor(SECOND, 5)));
        engine.commitTick(SECOND, Collections.emptyList());
        assertTrue(engine.state().mean(0) > 4.9);
    }

    @Test void delayedPolymorphicFactorReplaysTheTickWindow() {
        FusionEngine inOrder = engine();
        inOrder.submitFactor(new PositionFactor(SECOND, 4));
        inOrder.commitTick(SECOND, Collections.singletonList(model().factor(0, SECOND, 1.0)));
        inOrder.commitTick(2*SECOND, Collections.singletonList(model().factor(SECOND, 2*SECOND, 1.0)));

        FusionEngine delayed = engine();
        delayed.commitTick(SECOND, Collections.singletonList(model().factor(0, SECOND, 1.0)));
        delayed.commitTick(2*SECOND, Collections.singletonList(model().factor(SECOND, 2*SECOND, 1.0)));
        delayed.submitFactor(new PositionFactor(SECOND, 4));
        delayed.commitTick(2*SECOND, Collections.emptyList());

        assertArrayEquals(inOrder.state().mean(), delayed.state().mean(), 1e-10);
        assertTrue(inOrder.state().covariance().isIdentical(delayed.state().covariance(), 1e-10));
    }

    @Test void delayedFactorReplaysOnlyFromItsAffectedTick() {
        FusionEngine engine = engine();
        engine.commitTick(SECOND, Collections.singletonList(model().factor(0, SECOND, 1.0)));
        engine.commitTick(2*SECOND, Collections.singletonList(model().factor(SECOND, 2*SECOND, 1.0)));
        engine.commitTick(3*SECOND, Collections.singletonList(model().factor(2*SECOND, 3*SECOND, 1.0)));

        engine.submitFactor(new PositionFactor(2*SECOND, 8));
        engine.commitTick(3*SECOND, Collections.emptyList());

        assertEquals(1, engine.diagnostics().recomputationCount);
    }

    @Test void aNewTickRequiresOrdinaryFactorsThatMakeItsStateObservable() {
        FusionEngine engine = engine();
        org.junit.jupiter.api.Assertions.assertThrows(IllegalArgumentException.class,
                () -> engine.commitTick(SECOND, Collections.emptyList()));
        org.firstinspires.ftc.teamcode.fusion.factor.ProcessFactor<Double> process =
                model().factor(0, SECOND, 1.0);
        engine.commitTick(SECOND, Collections.singletonList(process));
    }

    @Test void independentSameTickFactorsAreNotAutomaticallyDeduplicated() {
        FusionEngine one = engine();
        one.submitFactor(new PositionFactor(0, 2));
        one.commitTick(0, Collections.emptyList());

        FusionEngine two = engine();
        two.submitFactor(new PositionFactor(0, 2));
        two.submitFactor(new PositionFactor(0, 2));
        two.commitTick(0, Collections.emptyList());

        assertTrue(two.state().covariance().get(0, 0) < one.state().covariance().get(0, 0));
    }

    private static ProcessModel<Double> model() {
        return new ProcessModel<Double>() {
            @Override public double[] predictMean(double[] prior, Double input, double dt) {
                return new double[] {prior[0] + input*dt, input};
            }

            @Override public SimpleMatrix processNoise(double[] prior, Double input, double dt) {
                return MatrixOps.diagonal(.01, .01);
            }
        };
    }

    private static FusionEngine engine() {
        return new FusionEngine(new Gaussian(new double[] {0, 0}, MatrixOps.diagonal(1, 1)),
                0, state -> state.clone(), 10*SECOND);
    }

    private static final class PositionFactor implements Factor {
        private final long timestamp;
        private final double position;

        PositionFactor(long timestamp, double position) {
            this.timestamp = timestamp;
            this.position = position;
        }

        @Override public String source() { return "position"; }
        @Override public long[] timestamps() { return new long[] {timestamp}; }
        @Override public LinearizedFactor linearize(StateValues values) {
            return new LinearizedFactor(
                    new double[] {values.state(timestamp)[0] - position},
                    MatrixOps.diagonal(1e-6),
                    new SimpleMatrix(new double[][] {{1, 0}}));
        }
    }

    private static final class DeltaFactor implements RelativeFactor {
        private final long from;
        private final long to;
        private final double delta;

        DeltaFactor(long from, long to, double delta) {
            this.from = from;
            this.to = to;
            this.delta = delta;
        }

        @Override public String source() { return "delta"; }
        @Override public long fromTimestampNanos() { return from; }
        @Override public long toTimestampNanos() { return to; }
        @Override public LinearizedFactor linearize(StateValues values) {
            return new LinearizedFactor(
                    new double[] {values.state(to)[0] - values.state(from)[0] - delta},
                    MatrixOps.diagonal(1e-6),
                    new SimpleMatrix(new double[][] {{-1, 0}}),
                    new SimpleMatrix(new double[][] {{1, 0}}));
        }
    }
}
