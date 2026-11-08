package org.firstinspires.ftc.teamcode.localization;

import static org.junit.jupiter.api.Assertions.*;

import org.firstinspires.ftc.teamcode.fusion.core.MatrixOps;
import org.firstinspires.ftc.teamcode.fusion.engine.FactorApplicationResult;
import org.firstinspires.ftc.teamcode.fusion.engine.FusionEngine;
import org.firstinspires.ftc.teamcode.localization.core.ControlInput;
import org.firstinspires.ftc.teamcode.localization.core.Pose2;
import org.firstinspires.ftc.teamcode.localization.core.RobotState;
import org.firstinspires.ftc.teamcode.localization.engine.PoseVelocityProcessModel;
import org.firstinspires.ftc.teamcode.localization.factor.AbsolutePoseFactor;
import org.firstinspires.ftc.teamcode.localization.factor.RelativePoseFactor;
import org.firstinspires.ftc.teamcode.localization.sensor.octoquad.OctoQuadFactorFactory;
import org.junit.jupiter.api.Test;

class OctoQuadFusionTest {
    @Test void factoryUsesFirstSampleAsBaselineAndEmitsOneDeltaPerLaterSample() {
        OctoQuadFactorFactory factory = new OctoQuadFactorFactory(MatrixOps.diagonal(.1, .1, .1));
        assertNull(factory.update(new Pose2(5, 6, .2), 1));
        RelativePoseFactor factor = factory.update(new Pose2(6, 7, .3), 2);
        assertNotNull(factor);
        Pose2 expected = new Pose2(5, 6, .2).localDeltaTo(new Pose2(6, 7, .3));
        assertEquals(expected.x(), factor.delta().x(), 1e-12);
        assertEquals(expected.y(), factor.delta().y(), 1e-12);
        assertEquals(expected.heading(), factor.delta().heading(), 1e-12);
        assertNull(factory.update(null, 3));
    }

    @Test void octoOnlyTrajectoryReproducesAccumulatedOdometryAndSurvivesAbsoluteCorrection() {
        RobotState initial = RobotState.create(Pose2.zero(), 0, 0, 0,
                MatrixOps.diagonal(1, 1, 1, 1, 1, 1));
        FusionEngine<ControlInput> engine = new FusionEngine<>(initial.gaussian(), 0,
                new PoseVelocityProcessModel(1, 1, 1, 1, new double[] {1, 1, 1, 1, 1, 1}),
                ControlInput.ZERO,
                10_000_000_000L, 100);
        OctoQuadFactorFactory factory = new OctoQuadFactorFactory(MatrixOps.diagonal(1e-6, 1e-6, 1e-6));
        Pose2 raw0 = new Pose2(20, 30, .4);
        assertNull(factory.update(raw0, 0));
        engine.predictTo(1_000_000_000L);
        engine.applyFactor(factory.update(raw0.compose(new Pose2(1, 0, .1)), 1_000_000_000L));
        assertEquals(1, new RobotState(engine.state()).pose().x(), 1e-5);
        assertEquals(1, new RobotState(engine.state()).vx(), 1e-5);
        assertTrue(engine.applyFactor(new AbsolutePoseFactor(
                "camera", 1_000_000_000L, new Pose2(10, 5, .2),
                MatrixOps.diagonal(1e-12, 1e-12, 1e-12))).isAccepted());
        engine.predictTo(2_000_000_000L);
        FactorApplicationResult update = engine.applyFactor(
                factory.update(raw0.compose(new Pose2(2, 0, .1)), 2_000_000_000L));
        assertTrue(update.isAccepted(), update.reason);
        assertTrue(new RobotState(engine.state()).pose().x() > 10.9,
                new RobotState(engine.state()).pose().toString());
        assertTrue(new RobotState(engine.state()).pose().x() < 12.1);
    }
}
