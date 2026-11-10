package org.firstinspires.ftc.teamcode.localization.sensor;

import org.firstinspires.ftc.teamcode.fusion.factor.Factor;

/** A localization source polled as part of {@code Localizer.update()}. */
public interface AutomaticFusionSource {
    /**
     * Samples once for the supplied loop interval and returns a factor for that tick, or
     * {@code null} when no measurement is available.
     */
    Factor poll(long fromTimestampNanos, long tickTimestampNanos);

    /** Publishes any source state staged by the last poll after the engine accepts the tick. */
    default void commit() { }

    /** Discards any source state staged by the last poll after the engine rejects the tick. */
    default void rollback() { }

    /** Reinitializes source history at the same timestamp as the reset fused estimate. */
    void reset(long timestampNanos);
}
