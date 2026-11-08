package org.firstinspires.ftc.teamcode.localization;

import static org.junit.jupiter.api.Assertions.*;

import org.junit.jupiter.api.Test;
import org.firstinspires.ftc.teamcode.localization.core.ControlInput;
import org.firstinspires.ftc.teamcode.fusion.core.Gaussian;
import org.firstinspires.ftc.teamcode.fusion.core.MatrixOps;
import org.firstinspires.ftc.teamcode.localization.core.Pose2;
import org.firstinspires.ftc.teamcode.localization.core.RobotState;
import org.firstinspires.ftc.teamcode.localization.engine.PoseVelocityProcessModel;
import org.firstinspires.ftc.teamcode.localization.factor.AbsolutePoseFactor;

class ProcessAndFactorTest {
    private static PoseVelocityProcessModel model() {
        return new PoseVelocityProcessModel(20, 10, 10, 2,
                new double[] {1e-6, 1e-6, 1e-6, 1e-5, 1e-5, 1e-5});
    }

    private static Gaussian zeroState() {
        return RobotState.create(Pose2.zero(), 0, 0, 0,
                MatrixOps.diagonal(.1, .1, .1, .1, .1, .1)).gaussian();
    }

    @Test void processPredictsStationaryForwardStrafeTurnAndCombinedMotion() {
        RobotState stationary = new RobotState(model().predict(zeroState(), ControlInput.ZERO, .1));
        assertEquals(0, stationary.pose().x(), 1e-12);
        RobotState forward = new RobotState(model().predict(zeroState(), new ControlInput(1, 0, 0), .1));
        assertTrue(forward.pose().x() > 0);
        assertTrue(forward.vx() > 0);
        RobotState strafe = new RobotState(model().predict(zeroState(), new ControlInput(0, 1, 0), .1));
        assertTrue(strafe.pose().y() > 0);
        RobotState turn = new RobotState(model().predict(zeroState(), new ControlInput(0, 0, 1), .1));
        assertTrue(turn.pose().heading() > 0);
        RobotState combined = new RobotState(model().predict(zeroState(), new ControlInput(.5, .4, .5), .1));
        assertTrue(combined.pose().x() > 0 && combined.pose().y() > 0 && combined.pose().heading() > 0);
    }

    @Test void robotCentricInputRotatesWithRobotHeading() {
        Gaussian facingLeft = RobotState.create(new Pose2(0, 0, Math.PI / 2), 0, 0, 0,
                MatrixOps.diagonal(.1, .1, .1, .1, .1, .1)).gaussian();
        RobotState robotCentric = new RobotState(model().predict(
                facingLeft, new ControlInput(1, 0, 0), .1));
        assertTrue(robotCentric.pose().y() > Math.abs(robotCentric.pose().x()));
    }

    @Test void poseUpdateChangesCorrelatedVelocityButPreservesUncorrelatedDimensions() {
        org.ejml.simple.SimpleMatrix covariance = MatrixOps.diagonal(1, 1, 1, 1, 1, 1);
        covariance.set(RobotState.X, RobotState.VX, .5);
        covariance.set(RobotState.VX, RobotState.X, .5);
        Gaussian prior = RobotState.create(Pose2.zero(), 0, 0, 0, covariance).gaussian();
        Gaussian posterior = new AbsolutePoseFactor("test", 0, new Pose2(1, 0, 0),
                MatrixOps.diagonal(.1, .1, .1)).apply(prior);
        assertTrue(posterior.mean(RobotState.X) > 0);
        assertTrue(posterior.mean(RobotState.VX) > 0);
        assertEquals(0, posterior.mean(RobotState.VY), 1e-12);
    }

    @Test void GaussianFusionDoesNotApplyAnImplicitOutlierGate() {
        Gaussian prior = zeroState();
        Gaussian posterior = new AbsolutePoseFactor("test", 0, new Pose2(100, 0, 0),
                MatrixOps.diagonal(.01, .01, .01)).apply(prior);
        assertTrue(posterior.mean(RobotState.X) > 90);
        assertTrue(posterior.covariance().get(RobotState.X, RobotState.X)
                < prior.covariance().get(RobotState.X, RobotState.X));
    }
}
