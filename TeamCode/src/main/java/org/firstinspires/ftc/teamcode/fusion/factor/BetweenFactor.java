package org.firstinspires.ftc.teamcode.fusion.factor;

/** A factor connecting two timestamped states. */
public interface BetweenFactor extends Factor {
    long fromTimestampNanos();
    long toTimestampNanos();

    @Override default long[] timestamps() {
        return new long[] {fromTimestampNanos(), toTimestampNanos()};
    }
}
