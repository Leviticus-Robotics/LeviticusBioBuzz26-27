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
import org.firstinspires.ftc.teamcode.localization.factor.MotionDeltaFactor;
import org.firstinspires.ftc.teamcode.localization.sensor.octoquad.OctoQuadFactorFactory;
import org.junit.jupiter.api.Test;

class OctoQuadFusionTest {
    @Test void factoryUsesFirstSampleAsBaselineAndEmitsOneDeltaPerLaterSample() {
        OctoQuadFactorFactory factory = new OctoQuadFactorFactory(
                MatrixOps.diagonal(.1, .1, .1, .2, .2, .2));
        assertNull(factory.stage(new Pose2(5, 6, .2), 0, 0, 0, 1));
        factory.commit();
        MotionDeltaFactor factor = factory.stage(new Pose2(6, 7, .3), 1, 2, .1, 2);
        assertNotNull(factor);
        Pose2 expected = new Pose2(5, 6, .2).localDeltaTo(new Pose2(6, 7, .3));
        assertEquals(expected.x(), factor.delta().x(), 1e-12);
        assertEquals(expected.y(), factor.delta().y(), 1e-12);
        assertEquals(expected.heading(), factor.delta().heading(), 1e-12);
        factory.commit();
        assertNull(factory.stage(null, 0, 0, 0, 3));
        factory.commit();
    }

    @Test void rejectedSampleDoesNotAdvanceCommittedDeltaBaseline() {
        OctoQuadFactorFactory factory =
                new OctoQuadFactorFactory(MatrixOps.diagonal(.1, .1, .1, .2, .2, .2));
        assertNull(factory.stage(Pose2.zero(), 0, 0, 0, 1));
        factory.commit();
        assertNotNull(factory.stage(new Pose2(1, 0, 0), 1, 0, 0, 2));
        factory.rollback();

        MotionDeltaFactor retried = factory.stage(new Pose2(2, 0, 0), 2, 0, 0, 3);
        assertNotNull(retried);
        assertEquals(2, retried.delta().x(), 1e-12);
    }

    @Test void octoOnlyTrajectoryReproducesAccumulatedOdometryAndSurvivesAbsoluteCorrection() {
        RobotState initial = RobotState.create(Pose2.zero(), 0, 0, 0,
                MatrixOps.diagonal(1, 1, 1, 1, 1, 1));
        FusionEngine engine = new FusionEngine(initial.gaussian(), 0, state -> {
            double[] normalized = state.clone();
            normalized[RobotState.HEADING] = Pose2.wrap(normalized[RobotState.HEADING]);
            return normalized;
        },
                10_000_000_000L);
        OctoQuadFactorFactory factory = new OctoQuadFactorFactory(
                MatrixOps.diagonal(1e-6, 1e-6, 1e-6, 1e-5, 1e-5, 1e-5));
        Pose2 raw0 = new Pose2(20, 30, .4);
        factory.initializeBaseline(raw0, 0);
        double rawHeading = .5;
        MotionDeltaFactor firstDelta = factory.stage(
                raw0.compose(new Pose2(1, 0, .1)), Math.cos(rawHeading),
                Math.sin(rawHeading), .1, 1_000_000_000L);
        assertTrue(engine.commitTick(1_000_000_000L,
                java.util.Collections.singletonList(firstDelta)).isAccepted());
        factory.commit();
        assertEquals(1, new RobotState(engine.state()).pose().x(), 1e-5);
        assertEquals(Math.cos(.1), new RobotState(engine.state()).vx(), 1e-4);
        assertEquals(Math.sin(.1), new RobotState(engine.state()).vy(), 1e-4);
        assertTrue(engine.submitFactor(new AbsolutePoseFactor(
                "camera", 1_000_000_000L, new Pose2(10, 5, .2),
                MatrixOps.diagonal(1e-12, 1e-12, 1e-12))).isAccepted());
        MotionDeltaFactor secondDelta = factory.stage(
                raw0.compose(new Pose2(2, 0, .1)), Math.cos(rawHeading),
                Math.sin(rawHeading), 0, 2_000_000_000L);
        FactorApplicationResult update = engine.commitTick(
                2_000_000_000L,
                java.util.Collections.singletonList(secondDelta));
        factory.commit();
        assertTrue(update.isAccepted(), update.reason);
        assertTrue(new RobotState(engine.state()).pose().x() > 10.9,
                new RobotState(engine.state()).pose().toString());
        assertTrue(new RobotState(engine.state()).pose().x() < 12.1);
    }

    @Test void processAndOctoFactorsFuseInTheSameTick() {
        RobotState initial = RobotState.create(Pose2.zero(), 0, 0, 0,
                MatrixOps.diagonal(1, 1, 1, 1, 1, 1));
        PoseVelocityProcessModel model = new PoseVelocityProcessModel(
                1, 2, 2, 2, new double[] {.1, .1, .1, .1, .1, .1});
        FusionEngine engine = new FusionEngine(initial.gaussian(), 0, model::normalize,
                10_000_000_000L);
        MotionDeltaFactor octo = new MotionDeltaFactor(
                "octoquad", 0, 1_000_000_000L, new Pose2(1, 0, 0), 1, 0, 0,
                MatrixOps.diagonal(.01, .01, .01, .01, .01, .01));

        assertTrue(engine.commitTick(1_000_000_000L, java.util.Arrays.asList(
                model.factor(0, 1_000_000_000L, new ControlInput(.5, 0, 0)),
                octo)).isAccepted());
        assertTrue(new RobotState(engine.state()).pose().x() > 0);
    }

}
