package org.firstinspires.ftc.teamcode.localization;

import static org.junit.jupiter.api.Assertions.*;

import java.util.Arrays;
import org.firstinspires.ftc.teamcode.fusion.core.MatrixOps;
import org.firstinspires.ftc.teamcode.fusion.engine.FusionEngine;
import org.firstinspires.ftc.teamcode.localization.core.ControlInput;
import org.firstinspires.ftc.teamcode.localization.core.Pose2;
import org.firstinspires.ftc.teamcode.localization.core.RobotState;
import org.firstinspires.ftc.teamcode.localization.engine.PoseVelocityProcessModel;
import org.firstinspires.ftc.teamcode.localization.factor.AbsolutePoseFactor;
import org.firstinspires.ftc.teamcode.localization.sensor.apriltag.AprilTagFrameProcessor;
import org.firstinspires.ftc.teamcode.localization.sensor.apriltag.AprilTagPoseObservation;
import org.junit.jupiter.api.Test;

class AprilTagFusionTest {
    private AprilTagFrameProcessor processor(double inflation) {
        return new AprilTagFrameProcessor(detection -> true, MatrixOps.diagonal(1, 1, .1),
                MatrixOps.diagonal(.2, .2, .02), 1e6, inflation, Pose2.zero());
    }

    private AprilTagPoseObservation observation(Pose2 pose) {
        return new AprilTagPoseObservation(pose, MatrixOps.diagonal(1, 1, .1));
    }

    @Test void poseObservationIsImmutableAndRejectsNullPose() {
        AprilTagPoseObservation observation = observation(Pose2.zero());
        org.ejml.simple.SimpleMatrix covariance = observation.covariance();
        covariance.set(0, 0, 100);
        assertEquals(1, observation.covariance().get(0, 0), 1e-12);
        assertThrows(IllegalArgumentException.class,
                () -> new AprilTagPoseObservation(null, MatrixOps.diagonal(1, 1, 1)));
    }

    @Test void duplicateIdenticalTagsDoNotIncreaseInformation() {
        AprilTagPoseObservation observation = observation(new Pose2(3, 4, Math.PI - .01));
        AbsolutePoseFactor one = processor(1).combineFrame(1, Arrays.asList(observation),
                new Pose2(3, 4, -Math.PI + .01));
        AbsolutePoseFactor two = processor(1).combineFrame(1, Arrays.asList(observation, observation),
                new Pose2(3, 4, -Math.PI + .01));
        assertTrue(one.covariance().isIdentical(two.covariance(), 1e-12));
        assertEquals(one.pose().heading(), two.pose().heading(), 1e-12);
    }

    @Test void disagreementInflatesCovarianceAndOutliersCanBeRejected() {
        AprilTagPoseObservation a = observation(new Pose2(0, 0, 0));
        AprilTagPoseObservation b = observation(new Pose2(1, 0, 0));
        AbsolutePoseFactor agreeing = processor(2).combineFrame(1, Arrays.asList(a, a), Pose2.zero());
        AbsolutePoseFactor disagreeing = processor(2).combineFrame(1, Arrays.asList(a, b), Pose2.zero());
        assertTrue(disagreeing.covariance().get(0, 0) > agreeing.covariance().get(0, 0));
        AprilTagFrameProcessor tight = new AprilTagFrameProcessor(detection -> true,
                MatrixOps.diagonal(.1, .1, .1), MatrixOps.diagonal(.01, .01, .01),
                9, 1, Pose2.zero());
        AbsolutePoseFactor gated = tight.combineFrame(2, Arrays.asList(a, observation(new Pose2(100, 0, 0))), Pose2.zero());
        assertEquals(0, gated.pose().x(), 1e-12);
    }

    @Test void delayedCameraFactorReplaysThroughHeadingWrap() {
        FusionEngine<ControlInput> engine = new FusionEngine<>(
                RobotState.create(Pose2.zero(), 0, 0, 0,
                        MatrixOps.diagonal(1, 1, 1, 1, 1, 1)).gaussian(), 0,
                new PoseVelocityProcessModel(1, 1, 1, 1,
                        new double[] {.01, .01, .01, .01, .01, .01}),
                ControlInput.ZERO,
                10_000_000_000L, 100);
        engine.predictTo(2_000_000_000L);
        AbsolutePoseFactor factor = processor(1).combineFrame(1_000_000_000L,
                Arrays.asList(observation(new Pose2(2, 0, Math.PI - .01))),
                new Pose2(0, 0, -Math.PI + .01));
        assertTrue(engine.applyFactor(factor).isAccepted());
        assertTrue(Math.abs(new RobotState(engine.state()).pose().heading()) > 2);
    }

    @Test void aCameraFrameIsSubmittedOnlyOnce() {
        AprilTagFrameProcessor processor = processor(1);
        java.util.List<AprilTagPoseObservation> observations = Arrays.asList(observation(Pose2.zero()));
        assertNotNull(processor.fromFrame(100, observations, Pose2.zero()));
        assertNull(processor.fromFrame(100, observations, Pose2.zero()));
        assertNull(processor.fromFrame(99, observations, Pose2.zero()));
        assertNotNull(processor.fromFrame(101, observations, Pose2.zero()));
    }
}
