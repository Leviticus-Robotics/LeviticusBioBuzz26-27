package org.firstinspires.ftc.teamcode.fusion.factor;

import org.firstinspires.ftc.teamcode.fusion.core.Gaussian;

/** A factor applied from a retained Gaussian at an earlier timestamp. */
public interface RelativeFactor extends Factor {
    long fromTimestampNanos();
    Gaussian apply(Gaussian stateAtFromTimestamp);
}
