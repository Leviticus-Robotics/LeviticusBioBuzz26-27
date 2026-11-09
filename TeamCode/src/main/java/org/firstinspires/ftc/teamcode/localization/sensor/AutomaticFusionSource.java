package org.firstinspires.ftc.teamcode.localization.sensor;

import org.firstinspires.ftc.teamcode.fusion.factor.Factor;

/** A localization source polled as part of {@code Localizer.update()}. */
public interface AutomaticFusionSource {
    /**
     * Returns a new factor timestamped by the source at measurement acquisition, or {@code null}
     * when no measurement is available.
     */
    Factor poll();

    /** Clears relative measurement history after the fused estimate is reset. */
    void reset();
}
