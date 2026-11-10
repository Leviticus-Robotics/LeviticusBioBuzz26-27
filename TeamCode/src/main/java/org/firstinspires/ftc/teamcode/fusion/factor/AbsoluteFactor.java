package org.firstinspires.ftc.teamcode.fusion.factor;

/** A factor constraining one timestamped state. */
public interface AbsoluteFactor extends Factor {
    long timestampNanos();

    @Override default long[] timestamps() { return new long[] {timestampNanos()}; }
}
