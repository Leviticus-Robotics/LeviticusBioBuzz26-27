package org.firstinspires.ftc.teamcode.fusion.factor;

/** Read-only access to state means by timestamp during factor linearization. */
@FunctionalInterface
public interface StateValues {
    double[] state(long timestampNanos);
}
