package org.firstinspires.ftc.teamcode.localization.sensor.apriltag;

import org.ejml.simple.SimpleMatrix;
import org.firstinspires.ftc.teamcode.fusion.core.MatrixOps;
import org.firstinspires.ftc.teamcode.fusion.core.Gaussian;
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

    /** Represents this pose in the heading chart nearest the supplied reference. */
    public Gaussian gaussianNear(Pose2 reference) {
        double heading = reference.heading() + Pose2.angularResidual(
                robotFieldPose.heading(), reference.heading());
        return new Gaussian(new double[] {
                robotFieldPose.x(), robotFieldPose.y(), heading
        }, covariance);
    }
}
