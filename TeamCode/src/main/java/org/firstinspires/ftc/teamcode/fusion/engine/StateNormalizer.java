package org.firstinspires.ftc.teamcode.fusion.engine;

/** Normalizes state coordinates such as wrapped angles independently of factor semantics. */
@FunctionalInterface
public interface StateNormalizer {
    StateNormalizer IDENTITY = state -> state.clone();
    double[] normalize(double[] state);
}
