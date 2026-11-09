package org.firstinspires.ftc.teamcode.localization.sensor.octoquad;

import com.pedropathing.math.Pose;
import com.pedropathing.revhub.localizers.OctoQuadLocalizer;
import org.firstinspires.ftc.teamcode.NanoClock;
import org.firstinspires.ftc.teamcode.localization.core.Pose2;
import org.firstinspires.ftc.teamcode.localization.factor.RelativePoseFactor;
import org.firstinspires.ftc.teamcode.localization.sensor.AutomaticFusionSource;

/** Hardware boundary: polls Pedro's OctoQuad wrapper and produces relative pose factors. */
public final class OctoQuadFusionSource implements AutomaticFusionSource {
    private final OctoQuadLocalizer localizer;
    private final OctoQuadFactorFactory factors;
    private final NanoClock clock;

    public OctoQuadFusionSource(OctoQuadLocalizer localizer, OctoQuadFactorFactory factors,
                               NanoClock clock) {
        if (localizer == null) throw new IllegalArgumentException("localizer is null");
        if (factors == null) throw new IllegalArgumentException("factor factory is null");
        if (clock == null) throw new IllegalArgumentException("clock is null");
        this.localizer = localizer;
        this.factors = factors;
        this.clock = clock;
    }

    @Override public RelativePoseFactor poll() {
        localizer.update();
        Pose pose = localizer.pose();
        if (pose == null) return null;
        return factors.update(
                new Pose2(pose.x(), pose.y(), pose.heading()), clock.now());
    }

    @Override public void reset() { factors.reset(); }
    public OctoQuadLocalizer rawLocalizer() { return localizer; }
}
