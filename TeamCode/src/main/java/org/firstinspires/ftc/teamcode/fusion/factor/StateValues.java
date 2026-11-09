package org.firstinspires.ftc.teamcode.fusion.factor;

/** Read-only access to timestamped state linearization points. */
public interface StateValues {
    double[] state(long timestampNanos);
}
