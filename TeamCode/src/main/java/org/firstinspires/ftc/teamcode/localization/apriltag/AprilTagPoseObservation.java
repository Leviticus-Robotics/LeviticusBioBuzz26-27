package org.firstinspires.ftc.teamcode.localization.apriltag;

import com.pedropathing.math.Pose;
import com.pedropathing.utils.Angle;
import org.ejml.simple.SimpleMatrix;
import org.firstinspires.ftc.teamcode.fusion.Gaussian;
import org.firstinspires.ftc.teamcode.fusion.MatrixOps;

/** One tag-derived or combined camera-frame absolute robot pose. */
public final class AprilTagPoseObservation {
    private final long timestampNanos;
    private final Pose robotFieldPose;
    private final SimpleMatrix covariance;

    public AprilTagPoseObservation(long timestampNanos, Pose robotFieldPose,
                                   SimpleMatrix covariance) {
        if (timestampNanos < 0) throw new IllegalArgumentException("timestamp is negative");
        if (robotFieldPose == null) throw new IllegalArgumentException("robot field pose is null");
        this.timestampNanos = timestampNanos;
        this.robotFieldPose = robotFieldPose;
        this.covariance = MatrixOps.requirePositiveDefinite(
                covariance, 3, "AprilTag covariance");
    }

    public long timestampNanos() { return timestampNanos; }
    public Pose robotFieldPose() { return robotFieldPose; }
    public SimpleMatrix covariance() { return covariance.copy(); }

    public Gaussian gaussianNear(Pose reference) {
        double heading = reference.heading() + Angle.normalizeSigned(
                robotFieldPose.heading() - reference.heading());
        return new Gaussian(new double[] {
                robotFieldPose.x(), robotFieldPose.y(), heading
        }, covariance);
    }

    public Gaussian gaussian() {
        return new Gaussian(new double[] {
                robotFieldPose.x(), robotFieldPose.y(), robotFieldPose.heading()
        }, covariance);
    }
}
