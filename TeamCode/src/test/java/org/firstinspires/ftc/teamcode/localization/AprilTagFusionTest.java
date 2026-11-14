package org.firstinspires.ftc.teamcode.localization;

import static org.junit.jupiter.api.Assertions.*;

import com.pedropathing.math.Pose;
import java.util.Arrays;
import org.firstinspires.ftc.teamcode.fusion.MatrixOps;
import org.firstinspires.ftc.teamcode.localization.apriltag.AprilTagFrameProcessor;
import org.firstinspires.ftc.teamcode.localization.apriltag.AprilTagPoseObservation;
import org.junit.jupiter.api.Test;

class AprilTagFusionTest {
    private AprilTagFrameProcessor processor(double inflation) {
        return new AprilTagFrameProcessor(detection -> true,
                MatrixOps.diagonal(1, 1, .1), MatrixOps.diagonal(.2, .2, .02),
                1e6, inflation, Pose.zero());
    }

    private AprilTagPoseObservation observation(Pose pose) {
        return new AprilTagPoseObservation(1, pose, MatrixOps.diagonal(1, 1, .1));
    }

    @Test void observationIsImmutableAndRejectsNullPose() {
        AprilTagPoseObservation observation = observation(Pose.zero());
        org.ejml.simple.SimpleMatrix covariance = observation.covariance();
        covariance.set(0, 0, 100);
        assertEquals(1, observation.covariance().get(0, 0), 1e-12);
        assertThrows(IllegalArgumentException.class,
                () -> new AprilTagPoseObservation(1, null, MatrixOps.diagonal(1, 1, 1)));
    }

    @Test void duplicateTagsDoNotPretendToBeIndependentCameras() {
        AprilTagPoseObservation observation = observation(new Pose(3, 4, Math.PI - .01));
        AprilTagPoseObservation one = processor(1).combineFrame(1, Arrays.asList(observation));
        AprilTagPoseObservation two = processor(1).combineFrame(
                1, Arrays.asList(observation, observation));
        assertTrue(one.covariance().isIdentical(two.covariance(), 1e-12));
        assertEquals(one.robotFieldPose().heading(), two.robotFieldPose().heading(), 1e-12);
    }

    @Test void disagreementInflatesCovarianceAndOutliersAreRejected() {
        AprilTagPoseObservation a = observation(Pose.zero());
        AprilTagPoseObservation b = observation(new Pose(1, 0, 0));
        AprilTagPoseObservation agreeing = processor(2).combineFrame(1, Arrays.asList(a, a));
        AprilTagPoseObservation disagreeing = processor(2).combineFrame(1, Arrays.asList(a, b));
        assertTrue(disagreeing.covariance().get(0, 0) > agreeing.covariance().get(0, 0));

        AprilTagFrameProcessor tight = new AprilTagFrameProcessor(detection -> true,
                MatrixOps.diagonal(.1, .1, .1), MatrixOps.diagonal(.01, .01, .01),
                9, 1, Pose.zero());
        AprilTagPoseObservation gated = tight.combineFrame(
                2, Arrays.asList(a, observation(new Pose(100, 0, 0))));
        assertEquals(0, gated.robotFieldPose().x(), 1e-12);
    }

    @Test void aCameraFrameIsSubmittedOnlyOnce() {
        AprilTagFrameProcessor processor = processor(1);
        java.util.List<AprilTagPoseObservation> observations =
                Arrays.asList(observation(Pose.zero()));
        assertNotNull(processor.fromFrame(100, observations));
        assertNull(processor.fromFrame(100, observations));
        assertNull(processor.fromFrame(99, observations));
        assertNotNull(processor.fromFrame(101, observations));
    }

    @Test void headingsAcrossWrapUseFrameConsensus() {
        AprilTagPoseObservation combined = processor(1).combineFrame(1, Arrays.asList(
                observation(new Pose(0, 0, Math.toRadians(179))),
                observation(new Pose(0, 0, Math.toRadians(-179)))));
        assertNotNull(combined);
        assertTrue(Math.abs(combined.robotFieldPose().heading()) > Math.toRadians(170));
    }
}
