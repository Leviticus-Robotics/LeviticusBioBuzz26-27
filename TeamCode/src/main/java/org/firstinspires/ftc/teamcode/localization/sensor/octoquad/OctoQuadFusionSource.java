package org.firstinspires.ftc.teamcode.localization.sensor.octoquad;

import com.pedropathing.localization.MotionState;
import com.pedropathing.math.Pose;
import com.pedropathing.math.Velocity;
import com.pedropathing.revhub.localizers.OctoQuadLocalizer;
import org.firstinspires.ftc.teamcode.localization.core.Pose2;
import org.firstinspires.ftc.teamcode.localization.factor.MotionDeltaFactor;
import org.firstinspires.ftc.teamcode.localization.sensor.AutomaticFusionSource;

/** Hardware boundary: polls Pedro's OctoQuad wrapper and produces joint motion factors. */
public final class OctoQuadFusionSource implements AutomaticFusionSource {
    private final OctoQuadLocalizer localizer;
    private final OctoQuadMotionAdapter motion;

    public OctoQuadFusionSource(OctoQuadLocalizer localizer, OctoQuadMotionAdapter motion) {
        if (localizer == null) throw new IllegalArgumentException("localizer is null");
        if (motion == null) throw new IllegalArgumentException("motion adapter is null");
        this.localizer = localizer;
        this.motion = motion;
    }

    public OctoQuadFusionSource(OctoQuadLocalizer localizer, OctoQuadMotionAdapter motion,
                                long initialTimestampNanos) {
        this(localizer, motion);
        establishBaseline(initialTimestampNanos);
    }

    @Override public MotionDeltaFactor poll(long fromTimestampNanos,
                                            long tickTimestampNanos) {
        localizer.update();
        MotionState state = localizer.state();
        if (state == null) return motion.stage(null, 0, 0, 0, tickTimestampNanos);
        Pose pose = state.pose();
        Velocity velocity = state.velocity();
        return motion.stage(new Pose2(pose.x(), pose.y(), pose.heading()),
                velocity.vx, velocity.vy, velocity.omega, tickTimestampNanos);
    }

    @Override public void commit() { motion.commit(); }
    @Override public void rollback() { motion.rollback(); }
    @Override public void reset(long timestampNanos) { establishBaseline(timestampNanos); }
    public OctoQuadLocalizer rawLocalizer() { return localizer; }

    private void establishBaseline(long timestampNanos) {
        localizer.update();
        MotionState state = localizer.state();
        if (state == null) {
            throw new IllegalStateException("OctoQuad has no valid state for fusion baseline");
        }
        Pose pose = state.pose();
        motion.initializeBaseline(
                new Pose2(pose.x(), pose.y(), pose.heading()), timestampNanos);
    }
}
