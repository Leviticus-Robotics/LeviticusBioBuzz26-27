package org.firstinspires.ftc.teamcode.localization;

import static org.junit.jupiter.api.Assertions.*;

import java.util.Arrays;
import org.firstinspires.ftc.teamcode.fusion.core.Gaussian;
import org.firstinspires.ftc.teamcode.fusion.core.MatrixOps;
import org.firstinspires.ftc.teamcode.fusion.engine.FactorApplicationResult;
import org.firstinspires.ftc.teamcode.fusion.engine.FusionEngine;
import org.firstinspires.ftc.teamcode.fusion.factor.MeasurementFactor;
import org.firstinspires.ftc.teamcode.localization.core.ControlInput;
import org.firstinspires.ftc.teamcode.localization.core.Pose2;
import org.firstinspires.ftc.teamcode.localization.core.RobotState;
import org.firstinspires.ftc.teamcode.localization.engine.PoseVelocityProcessModel;
import org.firstinspires.ftc.teamcode.localization.factor.AbsolutePoseFactor;
import org.firstinspires.ftc.teamcode.localization.factor.RelativePoseFactor;
import org.junit.jupiter.api.Test;

class LocalizationEngineTest {
    private FusionEngine<ControlInput> engine(long historyNanos, int events) {
        RobotState initial = RobotState.create(Pose2.zero(), 0, 0, 0,
                MatrixOps.diagonal(1, 1, 1, 1, 1, 1));
        return new FusionEngine<>(initial.gaussian(), 0,
                new PoseVelocityProcessModel(10, 10, 10, 2,
                        new double[] {1e-4, 1e-4, 1e-4, 1e-3, 1e-3, 1e-3}),
                ControlInput.ZERO,
                historyNanos, events);
    }

    @Test void previousInputIsUsedUntilChangeTimestampAndIntervalsArePredictedOnce() {
        FusionEngine<ControlInput> engine = engine(10_000_000_000L, 100);
        engine.setInput(0, new ControlInput(1, 0, 0));
        engine.setInput(1_000_000_000L, ControlInput.ZERO);
        double xAtChange = state(engine).pose().x();
        assertTrue(xAtChange > 0);
        engine.predictTo(1_000_000_000L);
        assertEquals(xAtChange, state(engine).pose().x(), 1e-12);
        engine.predictTo(2_000_000_000L);
        assertTrue(state(engine).pose().x() > xAtChange);
    }

    @Test void relativeFactorAdvancesFromCorrectedTrajectory() {
        FusionEngine<ControlInput> engine = engine(10_000_000_000L, 100);
        engine.predictTo(1_000_000_000L);
        assertTrue(engine.applyFactor(new AbsolutePoseFactor("camera", 1_000_000_000L,
                new Pose2(10, 0, 0), MatrixOps.diagonal(1e-4, 1e-4, 1e-4))).isAccepted());
        engine.predictTo(2_000_000_000L);
        assertTrue(engine.applyFactor(new RelativePoseFactor("octo", 1_000_000_000L, 2_000_000_000L,
                new Pose2(1, 0, 0), MatrixOps.diagonal(1e-12, 1e-12, 1e-12))).isAccepted());
        assertEquals(11, state(engine).pose().x(), .02);
    }

    @Test void delayedFactorReplayMatchesInOrderResult() {
        AbsolutePoseFactor camera = new AbsolutePoseFactor("camera", 1_000_000_000L,
                new Pose2(5, 0, Math.PI - .01), MatrixOps.diagonal(.01, .01, .01));
        FusionEngine<ControlInput> inOrder = engine(10_000_000_000L, 100);
        inOrder.predictTo(1_000_000_000L);
        inOrder.applyFactor(camera);
        inOrder.predictTo(2_000_000_000L);
        FusionEngine<ControlInput> delayed = engine(10_000_000_000L, 100);
        delayed.predictTo(2_000_000_000L);
        delayed.applyFactor(camera);
        assertArrayEquals(inOrder.state().mean(), delayed.state().mean(), 1e-10);
        assertTrue(inOrder.state().covariance().isIdentical(delayed.state().covariance(), 1e-10));
    }

    @Test void historyIsOrderedBoundedAndStaleFactorsAreRejected() {
        FusionEngine<ControlInput> engine = engine(2_000_000_000L, 3);
        engine.predictTo(1_000_000_000L);
        engine.predictTo(3_000_000_000L);
        engine.predictTo(2_000_000_000L);
        engine.predictTo(4_000_000_000L);
        assertEquals(Arrays.asList(2_000_000_000L, 3_000_000_000L, 4_000_000_000L),
                engine.retainedEventTimestamps());
        FactorApplicationResult stale = engine.applyFactor(new AbsolutePoseFactor("old", 0,
                Pose2.zero(), MatrixOps.diagonal(1, 1, 1)));
        assertFalse(stale.isAccepted());
        assertTrue(stale.reason.contains("retained history"));
    }

    @Test void distantFactorsAreFusedWithoutAHiddenStatisticalGate() {
        FusionEngine<ControlInput> engine = engine(10_000_000_000L, 100);
        engine.predictTo(1_000_000_000L);
        FactorApplicationResult update = engine.applyFactor(new AbsolutePoseFactor("distant", 1_000_000_000L,
                new Pose2(100, 0, 0), MatrixOps.diagonal(.01, .01, .01)));
        assertTrue(update.isAccepted());
        assertTrue(state(engine).pose().x() > 90);
    }

    @Test void processModelAndEngineAreGenericOverInputType() {
        RobotState initial = RobotState.create(Pose2.zero(), 0, 0, 0,
                MatrixOps.diagonal(1, 1, 1, 1, 1, 1));
        FusionEngine<Double> scalarInputEngine = new FusionEngine<>(initial.gaussian(), 0,
                (prior, input, elapsedSeconds) -> {
                    double[] mean = prior.mean();
                    mean[RobotState.X] += input * elapsedSeconds;
                    return new Gaussian(mean, prior.covariance());
                }, 0.0, 10_000_000_000L, 100);
        scalarInputEngine.setInput(0, 2.0);
        scalarInputEngine.predictTo(1_000_000_000L);
        assertEquals(2.0, new RobotState(scalarInputEngine.state()).pose().x(), 1e-12);
    }

    @Test void genericMeasurementFactorCanObserveVelocityWithoutKnowingAboutPose() {
        FusionEngine<ControlInput> engine = engine(10_000_000_000L, 100);
        MeasurementFactor velocityMeasurement = new MeasurementFactor() {
            @Override public String source() { return "velocity-sensor"; }
            @Override public long timestampNanos() { return 0; }
            @Override public Gaussian measurement(Gaussian predictedState) {
                return new Gaussian(new double[] {7}, MatrixOps.diagonal(.001));
            }
            @Override public org.ejml.simple.SimpleMatrix stateToMeasurement(Gaussian predictedState) {
                org.ejml.simple.SimpleMatrix model = new org.ejml.simple.SimpleMatrix(1, RobotState.DIMENSION);
                model.set(0, RobotState.VX, 1);
                return model;
            }
        };
        assertTrue(engine.applyFactor(velocityMeasurement).isAccepted());
        assertEquals(7, state(engine).vx(), .01);
        assertEquals(0, state(engine).pose().x(), 1e-12);
    }

    private static RobotState state(FusionEngine<?> engine) { return new RobotState(engine.state()); }
}
