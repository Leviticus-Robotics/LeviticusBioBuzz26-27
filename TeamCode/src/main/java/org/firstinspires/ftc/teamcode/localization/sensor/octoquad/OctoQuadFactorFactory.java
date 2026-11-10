package org.firstinspires.ftc.teamcode.localization.sensor.octoquad;

import org.ejml.simple.SimpleMatrix;
import org.firstinspires.ftc.teamcode.fusion.core.MatrixOps;
import org.firstinspires.ftc.teamcode.localization.core.Pose2;
import org.firstinspires.ftc.teamcode.localization.factor.MotionDeltaFactor;

/** Converts consecutive OctoQuad motion states into one joint motion factor per new sample. */
public final class OctoQuadFactorFactory {
    private final SimpleMatrix motionCovariance;
    private Pose2 previousPose;
    private long previousTimestampNanos;
    private Pose2 stagedPose;
    private long stagedTimestampNanos;
    private boolean staged;
    private boolean stagedReset;

    public OctoQuadFactorFactory(SimpleMatrix motionCovariance) {
        this.motionCovariance = MatrixOps.requirePositiveDefinite(
                motionCovariance, 6, "OctoQuad motion covariance");
    }

    /** Stages one sample; call {@link #commit()} or {@link #rollback()} after the tick result. */
    public MotionDeltaFactor stage(Pose2 rawPose, double vx, double vy, double omega,
                                   long timestampNanos) {
        rollback();
        if (rawPose == null) {
            stagedReset = true;
            return null;
        }
        if (timestampNanos < 0) return null;
        staged = true;
        stagedPose = rawPose;
        stagedTimestampNanos = timestampNanos;
        if (previousPose == null) {
            return null;
        }
        if (timestampNanos <= previousTimestampNanos) {
            rollback();
            return null;
        }
        Pose2 delta = previousPose.localDeltaTo(rawPose);
        double cosine = Math.cos(rawPose.heading());
        double sine = Math.sin(rawPose.heading());
        double forwardVelocity = cosine*vx + sine*vy;
        double strafeVelocity = -sine*vx + cosine*vy;
        return new MotionDeltaFactor("octoquad", previousTimestampNanos, timestampNanos,
                delta, forwardVelocity, strafeVelocity, omega, motionCovariance);
    }

    /** Establishes the committed integrated-pose baseline without creating a measurement. */
    public void initializeBaseline(Pose2 rawPose, long timestampNanos) {
        if (rawPose == null || timestampNanos < 0) {
            throw new IllegalArgumentException("OctoQuad baseline is invalid");
        }
        previousPose = rawPose;
        previousTimestampNanos = timestampNanos;
        rollback();
    }

    public void commit() {
        if (stagedReset) {
            previousPose = null;
            previousTimestampNanos = 0;
        } else if (staged) {
            previousPose = stagedPose;
            previousTimestampNanos = stagedTimestampNanos;
        }
        rollback();
    }

    public void rollback() {
        stagedPose = null;
        stagedTimestampNanos = 0;
        staged = false;
        stagedReset = false;
    }

    public void reset() {
        previousPose = null;
        previousTimestampNanos = 0;
        rollback();
    }
}
