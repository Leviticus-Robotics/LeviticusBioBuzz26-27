package org.firstinspires.ftc.teamcode.fusion.factor;

import org.firstinspires.ftc.teamcode.fusion.core.Gaussian;

/** A factor applied to the current Gaussian at its timestamp. */
public interface AbsoluteFactor extends Factor {
    Gaussian apply(Gaussian currentState);
}
