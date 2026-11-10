package org.firstinspires.ftc.teamcode.fusion.factor;

/** A factor constraining the relationship between two timestamped states. */
public interface RelativeFactor extends Factor {
    long fromTimestampNanos();
    long toTimestampNanos();

    @Override default long[] timestamps() {
        return new long[] {fromTimestampNanos(), toTimestampNanos()};
    }
}
