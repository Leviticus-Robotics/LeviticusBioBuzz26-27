package org.firstinspires.ftc.teamcode.fusion.factor;

/** Timestamped, source-independent inference input metadata. */
public interface Factor {
    String source();
    long timestampNanos();
}
