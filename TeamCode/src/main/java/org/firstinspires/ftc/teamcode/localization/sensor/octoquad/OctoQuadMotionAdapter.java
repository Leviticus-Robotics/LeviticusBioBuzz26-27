package org.firstinspires.ftc.teamcode.localization.sensor.octoquad;

import org.ejml.simple.SimpleMatrix;
import org.firstinspires.ftc.teamcode.fusion.core.MatrixOps;
import org.firstinspires.ftc.teamcode.localization.core.Pose2;
import org.firstinspires.ftc.teamcode.localization.factor.MotionDeltaFactor;

/** Converts consecutive integrated OctoQuad poses into local motion factors. */
public final class OctoQuadMotionAdapter {
    private final SimpleMatrix covariance;
    private Pose2 previousPose;
    private long previousTimestamp;
    private Pose2 stagedPose;
    private long stagedTimestamp;
    private boolean staged;
    private boolean stagedReset;

    public OctoQuadMotionAdapter(SimpleMatrix covariance) {
        this.covariance = MatrixOps.requirePositiveDefinite(
                covariance, 6, "OctoQuad motion covariance");
    }

    public MotionDeltaFactor stage(Pose2 pose, double vx, double vy, double omega,
                                   long timestamp) {
        rollback();
        if (pose == null) {
            stagedReset = true;
            return null;
        }
        if (timestamp < 0) return null;
        staged = true;
        stagedPose = pose;
        stagedTimestamp = timestamp;
        if (previousPose == null) return null;
        if (timestamp <= previousTimestamp) {
            rollback();
            return null;
        }
        Pose2 delta = previousPose.localDeltaTo(pose);
        double cosine = Math.cos(pose.heading());
        double sine = Math.sin(pose.heading());
        return new MotionDeltaFactor("octoquad", previousTimestamp, timestamp, delta,
                cosine*vx + sine*vy, -sine*vx + cosine*vy, omega, covariance);
    }

    public void initializeBaseline(Pose2 pose, long timestamp) {
        if (pose == null || timestamp < 0) {
            throw new IllegalArgumentException("OctoQuad baseline is invalid");
        }
        previousPose = pose;
        previousTimestamp = timestamp;
        rollback();
    }

    public void commit() {
        if (stagedReset) {
            previousPose = null;
            previousTimestamp = 0;
        } else if (staged) {
            previousPose = stagedPose;
            previousTimestamp = stagedTimestamp;
        }
        rollback();
    }

    public void rollback() {
        stagedPose = null;
        stagedTimestamp = 0;
        staged = false;
        stagedReset = false;
    }
}
