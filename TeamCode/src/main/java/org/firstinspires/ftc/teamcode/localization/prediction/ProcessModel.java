package org.firstinspires.ftc.teamcode.localization.prediction;

import com.pedropathing.localization.MotionState;

/** Stateless short-horizon predictor driven by an external input. */
public interface ProcessModel<I> {
    MotionState predict(MotionState initial, I input, double elapsedSeconds);
}
