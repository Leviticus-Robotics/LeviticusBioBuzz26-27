package org.firstinspires.ftc.teamcode.localization.sensor.octoquad;

import org.ejml.simple.SimpleMatrix;
import org.firstinspires.ftc.teamcode.fusion.core.MatrixOps;
import org.firstinspires.ftc.teamcode.localization.core.Pose2;
import org.firstinspires.ftc.teamcode.localization.factor.RelativePoseFactor;

/** Converts consecutive integrated OctoQuad poses into one relative-pose factor per new sample. */
public final class OctoQuadFactorFactory {
    private final SimpleMatrix deltaCovariance;
    private Pose2 previousPose;
    private long previousTimestampNanos;

    public OctoQuadFactorFactory(SimpleMatrix deltaCovariance) {
        this.deltaCovariance = MatrixOps.requirePositiveDefinite(deltaCovariance, 3, "OctoQuad delta covariance");
    }

    public RelativePoseFactor update(Pose2 rawPose, long timestampNanos) {
        if (rawPose == null || timestampNanos < 0) return null;
        if (previousPose == null) {
            previousPose = rawPose;
            previousTimestampNanos = timestampNanos;
            return null;
        }
        if (timestampNanos <= previousTimestampNanos) return null;
        Pose2 delta = previousPose.localDeltaTo(rawPose);
        RelativePoseFactor factor = new RelativePoseFactor(
                "octoquad", previousTimestampNanos, timestampNanos, delta, deltaCovariance);
        previousPose = rawPose;
        previousTimestampNanos = timestampNanos;
        return factor;
    }

    public void reset() {
        previousPose = null;
        previousTimestampNanos = 0;
    }
}
