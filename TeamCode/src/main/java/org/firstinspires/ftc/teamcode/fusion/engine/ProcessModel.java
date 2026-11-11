package org.firstinspires.ftc.teamcode.fusion.engine;

import org.firstinspires.ftc.teamcode.fusion.core.Gaussian;

public interface ProcessModel<I> {
    Gaussian predict(Gaussian prior, I input, double elapsedSeconds);
}
