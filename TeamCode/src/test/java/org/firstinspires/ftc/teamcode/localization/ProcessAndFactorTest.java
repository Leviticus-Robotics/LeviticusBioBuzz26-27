package org.firstinspires.ftc.teamcode.localization;

import static org.junit.jupiter.api.Assertions.*;

import org.junit.jupiter.api.Test;
import org.firstinspires.ftc.teamcode.localization.core.ControlInput;
import org.firstinspires.ftc.teamcode.fusion.core.Gaussian;
import org.firstinspires.ftc.teamcode.fusion.core.MatrixOps;
import org.firstinspires.ftc.teamcode.fusion.engine.FusionEngine;
import org.firstinspires.ftc.teamcode.localization.core.Pose2;
import org.firstinspires.ftc.teamcode.localization.core.RobotState;
import org.firstinspires.ftc.teamcode.localization.engine.PoseVelocityProcessModel;
import org.firstinspires.ftc.teamcode.localization.factor.AbsolutePoseFactor;

class ProcessAndFactorTest {
    private static PoseVelocityProcessModel model() {
        return new PoseVelocityProcessModel(new PoseVelocityProcessModel.Config(
                20, 10, 10, 2,
                new double[] {1e-6, 1e-6, 1e-6, 1e-5, 1e-5, 1e-5}));
    }

    private static Gaussian zeroState() {
        return RobotState.create(Pose2.zero(), 0, 0, 0,
                MatrixOps.diagonal(.1, .1, .1, .1, .1, .1)).gaussian();
    }

    @Test void processPredictsStationaryForwardStrafeTurnAndCombinedMotion() {
        RobotState stationary = new RobotState(predict(zeroState(), ControlInput.ZERO, .1));
        assertEquals(0, stationary.pose().x(), 1e-12);
        RobotState forward = new RobotState(predict(zeroState(), new ControlInput(1, 0, 0), .1));
        assertTrue(forward.pose().x() > 0);
        assertTrue(forward.vx() > 0);
        RobotState strafe = new RobotState(predict(zeroState(), new ControlInput(0, 1, 0), .1));
        assertTrue(strafe.pose().y() > 0);
        RobotState turn = new RobotState(predict(zeroState(), new ControlInput(0, 0, 1), .1));
        assertTrue(turn.pose().heading() > 0);
        RobotState combined = new RobotState(predict(zeroState(), new ControlInput(.5, .4, .5), .1));
        assertTrue(combined.pose().x() > 0 && combined.pose().y() > 0 && combined.pose().heading() > 0);
    }

    @Test void robotCentricInputRotatesWithRobotHeading() {
        Gaussian facingLeft = RobotState.create(new Pose2(0, 0, Math.PI / 2), 0, 0, 0,
                MatrixOps.diagonal(.1, .1, .1, .1, .1, .1)).gaussian();
        RobotState robotCentric = new RobotState(predict(
                facingLeft, new ControlInput(1, 0, 0), .1));
        assertTrue(robotCentric.pose().y() > Math.abs(robotCentric.pose().x()));
    }

    @Test void poseUpdateChangesCorrelatedVelocityButPreservesUncorrelatedDimensions() {
        org.ejml.simple.SimpleMatrix covariance = MatrixOps.diagonal(1, 1, 1, 1, 1, 1);
        covariance.set(RobotState.X, RobotState.VX, .5);
        covariance.set(RobotState.VX, RobotState.X, .5);
        Gaussian prior = RobotState.create(Pose2.zero(), 0, 0, 0, covariance).gaussian();
        FusionEngine engine = engine(prior);
        engine.submitFactor(new AbsolutePoseFactor("test", 0, new Pose2(1, 0, 0),
                MatrixOps.diagonal(.1, .1, .1)));
        engine.commitTick(0, java.util.Collections.emptyList());
        Gaussian posterior = engine.state();
        assertTrue(posterior.mean(RobotState.X) > 0);
        assertTrue(posterior.mean(RobotState.VX) > 0);
        assertEquals(0, posterior.mean(RobotState.VY), 1e-12);
    }

    @Test void GaussianUpdateDoesNotApplyAnImplicitOutlierGate() {
        Gaussian prior = zeroState();
        FusionEngine engine = engine(prior);
        engine.submitFactor(new AbsolutePoseFactor("test", 0, new Pose2(100, 0, 0),
                MatrixOps.diagonal(.01, .01, .01)));
        engine.commitTick(0, java.util.Collections.emptyList());
        Gaussian posterior = engine.state();
        assertTrue(posterior.mean(RobotState.X) > 90);
        assertTrue(posterior.covariance().get(RobotState.X, RobotState.X)
                < prior.covariance().get(RobotState.X, RobotState.X));
    }

    private static FusionEngine engine(Gaussian initial) {
        return new FusionEngine(initial, 0, model()::normalize,
                10_000_000_000L);
    }

    private static Gaussian predict(Gaussian prior, ControlInput input, double seconds) {
        long toTimestampNanos = Math.round(seconds*1e9);
        FusionEngine engine = engine(prior);
        engine.commitTick(toTimestampNanos, java.util.Collections.singletonList(
                model().factor(0, toTimestampNanos, input)));
        return engine.state();
    }

}
