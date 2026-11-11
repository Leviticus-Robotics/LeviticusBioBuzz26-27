package org.firstinspires.ftc.teamcode.fusion.factor;

/** Immutable polymorphic Gaussian constraint over timestamped states. */
public interface Factor {
    String source();
    long[] timestamps();
    LinearizedFactor linearize(StateValues values);

    /** Null means every instance is an independent observation. */
    default String observationId() { return null; }

    default long timestampNanos() {
        long latest = Long.MIN_VALUE;
        for (long timestamp : timestamps()) latest = Math.max(latest, timestamp);
        return latest;
    }
}
