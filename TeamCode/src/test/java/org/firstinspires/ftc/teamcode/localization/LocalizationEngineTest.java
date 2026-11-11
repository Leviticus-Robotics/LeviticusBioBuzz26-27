package org.firstinspires.ftc.teamcode.localization;

import static org.junit.jupiter.api.Assertions.*;

import java.util.Arrays;
import org.firstinspires.ftc.teamcode.fusion.core.Gaussian;
import org.firstinspires.ftc.teamcode.fusion.core.MatrixOps;
import org.firstinspires.ftc.teamcode.fusion.engine.FusionEngine;
import org.firstinspires.ftc.teamcode.localization.core.ControlInput;
import org.firstinspires.ftc.teamcode.localization.core.Pose2;
import org.firstinspires.ftc.teamcode.localization.core.RobotState;
import org.firstinspires.ftc.teamcode.localization.engine.PoseVelocityProcessModel;
import org.firstinspires.ftc.teamcode.localization.factor.AbsolutePoseFactor;
import org.firstinspires.ftc.teamcode.localization.factor.PoseDeltaFactor;
import org.junit.jupiter.api.Test;

class LocalizationEngineTest {
    private static PoseVelocityProcessModel model() {
        return new PoseVelocityProcessModel(new PoseVelocityProcessModel.Config(
                10, 10, 10, 2,
                new double[] {1e-4, 1e-4, 1e-4, 1e-3, 1e-3, 1e-3}));
    }

    private FusionEngine engine(long historyNanos) {
        RobotState initial = RobotState.create(Pose2.zero(), 0, 0, 0,
                MatrixOps.diagonal(1, 1, 1, 1, 1, 1));
        return new FusionEngine(initial.gaussian(), 0, model()::normalize,
                historyNanos);
    }

    @Test void previousInputIsUsedUntilChangeTimestampAndIntervalsArePredictedOnce() {
        FusionEngine engine = engine(10_000_000_000L);
        tick(engine, 1_000_000_000L, new ControlInput(1, 0, 0));
        double xAtChange = state(engine).pose().x();
        assertTrue(xAtChange > 0);
        tick(engine, 2_000_000_000L, ControlInput.ZERO);
        assertTrue(state(engine).pose().x() > xAtChange);
    }

    @Test void relativeFactorAdvancesFromCorrectedTrajectory() {
        FusionEngine engine = engine(10_000_000_000L);
        assertTrue(engine.submitFactor(new AbsolutePoseFactor("camera", 1_000_000_000L,
                new Pose2(10, 0, 0), MatrixOps.diagonal(1e-4, 1e-4, 1e-4))));
        tick(engine, 1_000_000_000L, ControlInput.ZERO);
        engine.commitTick(2_000_000_000L, Arrays.asList(
                model().factor(1_000_000_000L, 2_000_000_000L, ControlInput.ZERO),
                new PoseDeltaFactor(
                        "octo", 1_000_000_000L, 2_000_000_000L,
                        new Pose2(1, 0, 0), MatrixOps.diagonal(1e-12, 1e-12, 1e-12))));
        assertEquals(11, state(engine).pose().x(), .02);
    }

    @Test void delayedFactorReplayMatchesInOrderResult() {
        AbsolutePoseFactor camera = new AbsolutePoseFactor("camera", 1_000_000_000L,
                new Pose2(5, 0, Math.PI - .01), MatrixOps.diagonal(.01, .01, .01));
        FusionEngine inOrder = engine(10_000_000_000L);
        inOrder.submitFactor(camera);
        tick(inOrder, 1_000_000_000L, ControlInput.ZERO);
        tick(inOrder, 2_000_000_000L, ControlInput.ZERO);
        FusionEngine delayed = engine(10_000_000_000L);
        tick(delayed, 1_000_000_000L, ControlInput.ZERO);
        tick(delayed, 2_000_000_000L, ControlInput.ZERO);
        delayed.submitFactor(camera);
        tick(delayed, 2_000_000_000L, ControlInput.ZERO);
        assertArrayEquals(inOrder.state().mean(), delayed.state().mean(), 1e-10);
        assertTrue(inOrder.state().covariance().isIdentical(delayed.state().covariance(), 1e-10));
    }

    @Test void historyIsOrderedBoundedAndStaleFactorsAreRejected() {
        FusionEngine engine = engine(2_000_000_000L);
        tick(engine, 1_000_000_000L, ControlInput.ZERO);
        tick(engine, 2_000_000_000L, ControlInput.ZERO);
        tick(engine, 3_000_000_000L, ControlInput.ZERO);
        tick(engine, 4_000_000_000L, ControlInput.ZERO);
        assertEquals(Arrays.asList(2_000_000_000L, 3_000_000_000L, 4_000_000_000L),
                engine.retainedTickTimestamps());
        assertFalse(engine.submitFactor(new AbsolutePoseFactor("old", 0,
                Pose2.zero(), MatrixOps.diagonal(1, 1, 1))));
    }

    @Test void distantFactorsAreFusedWithoutAHiddenStatisticalGate() {
        FusionEngine engine = engine(10_000_000_000L);
        tick(engine, 1_000_000_000L, ControlInput.ZERO);
        boolean update = engine.submitFactor(new AbsolutePoseFactor("distant", 1_000_000_000L,
                new Pose2(100, 0, 0), MatrixOps.diagonal(.01, .01, .01)));
        assertTrue(update);
        tick(engine, 1_000_000_000L, ControlInput.ZERO);
        assertTrue(state(engine).pose().x() > 90);
    }

    @Test void independentAbsoluteAndRelativeFactorsFuseAsOneOrderInvariantBatch() {
        PoseDeltaFactor relative = new PoseDeltaFactor(
                "octo", 0, 1_000_000_000L, new Pose2(2, 0, .1),
                MatrixOps.diagonal(.01, .01, .01));
        AbsolutePoseFactor absolute = new AbsolutePoseFactor(
                "camera", 1_000_000_000L, new Pose2(3, 1, .2),
                MatrixOps.diagonal(.02, .02, .02));
        FusionEngine first = engine(10_000_000_000L);
        FusionEngine second = engine(10_000_000_000L);
        first.commitTick(1_000_000_000L, Arrays.asList(
                model().factor(0, 1_000_000_000L, ControlInput.ZERO),
                relative, absolute));
        second.commitTick(1_000_000_000L, Arrays.asList(
                absolute, relative,
                model().factor(0, 1_000_000_000L, ControlInput.ZERO)));

        assertArrayEquals(first.state().mean(), second.state().mean(), 1e-10);
        assertTrue(first.state().covariance().isIdentical(second.state().covariance(), 1e-10));
    }

    private static RobotState state(FusionEngine engine) { return new RobotState(engine.state()); }

    private static void tick(FusionEngine engine, long timestamp, ControlInput input) {
        java.util.List<? extends org.firstinspires.ftc.teamcode.fusion.factor.Factor> factors =
                timestamp == engine.timestampNanos()
                        ? java.util.Collections.emptyList()
                        : java.util.Collections.singletonList(
                                model().factor(engine.timestampNanos(), timestamp, input));
        engine.commitTick(timestamp, factors);
    }
}
