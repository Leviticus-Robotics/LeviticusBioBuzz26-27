package org.firstinspires.ftc.teamcode.localization.sensor.apriltag;

import org.ejml.simple.SimpleMatrix;
import org.firstinspires.ftc.teamcode.fusion.core.MatrixOps;
import org.firstinspires.ftc.teamcode.localization.core.Pose2;

/** One eligible tag or already-solved tag-cluster robot field-pose estimate. */
public final class AprilTagPoseObservation {
    private final Pose2 robotFieldPose;
    private final SimpleMatrix covariance;

    public AprilTagPoseObservation(Pose2 robotFieldPose, SimpleMatrix covariance) {
        if (robotFieldPose == null) throw new IllegalArgumentException("robot field pose is null");
        this.robotFieldPose = robotFieldPose;
        this.covariance = MatrixOps.requirePositiveDefinite(covariance, 3, "AprilTag covariance");
    }

    public Pose2 robotFieldPose() { return robotFieldPose; }
    public SimpleMatrix covariance() { return covariance.copy(); }
}
