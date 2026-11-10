package org.firstinspires.ftc.teamcode.fusion.factor;

import org.ejml.simple.SimpleMatrix;

/** Immutable polymorphic Gaussian constraint over timestamped states. */
public interface Factor {
    String source();
    long[] timestamps();
    double[] residual(StateValues values);
    SimpleMatrix covariance(StateValues values);
    SimpleMatrix[] jacobians(StateValues values);

    /** Null means every instance is an independent observation. */
    default String observationId() { return null; }

    /** Packages the factor's independently defined analytic behavior for the engine. */
    default LinearizedFactor linearize(StateValues values) {
        return new LinearizedFactor(timestamps(), residual(values), covariance(values),
                jacobians(values));
    }

    default long timestampNanos() {
        long latest = Long.MIN_VALUE;
        for (long timestamp : timestamps()) latest = Math.max(latest, timestamp);
        return latest;
    }
}
