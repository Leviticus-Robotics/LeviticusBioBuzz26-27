package org.firstinspires.ftc.teamcode.localization.sensor.octoquad;

import com.pedropathing.math.Pose;
import com.pedropathing.revhub.localizers.OctoQuadLocalizer;
import org.firstinspires.ftc.teamcode.localization.core.Pose2;
import org.firstinspires.ftc.teamcode.fusion.engine.FusionEngine;
import org.firstinspires.ftc.teamcode.localization.factor.RelativePoseFactor;

/** Hardware boundary: polls Pedro's OctoQuad wrapper and submits only a relative pose factor. */
public final class OctoQuadFusionSource {
    private final OctoQuadLocalizer localizer;
    private final OctoQuadFactorFactory factors;

    public OctoQuadFusionSource(OctoQuadLocalizer localizer, OctoQuadFactorFactory factors) {
        this.localizer = localizer;
        this.factors = factors;
    }

    public void poll(FusionEngine<?> engine, long timestampNanos) {
        localizer.update();
        Pose pose = localizer.pose();
        if (pose == null) return;
        RelativePoseFactor factor = factors.update(new Pose2(pose.x(), pose.y(), pose.heading()), timestampNanos);
        if (factor == null) engine.predictTo(timestampNanos);
        else engine.applyFactor(factor);
    }

    public void reset() { factors.reset(); }
    public OctoQuadLocalizer rawLocalizer() { return localizer; }
}
