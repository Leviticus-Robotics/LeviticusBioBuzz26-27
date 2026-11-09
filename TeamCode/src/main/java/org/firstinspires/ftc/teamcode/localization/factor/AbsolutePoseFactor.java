package org.firstinspires.ftc.teamcode.localization.factor;

import org.ejml.simple.SimpleMatrix;
import org.firstinspires.ftc.teamcode.fusion.core.MatrixOps;
import org.firstinspires.ftc.teamcode.fusion.factor.Factor;
import org.firstinspires.ftc.teamcode.fusion.factor.LinearizedFactor;
import org.firstinspires.ftc.teamcode.fusion.factor.StateValues;
import org.firstinspires.ftc.teamcode.localization.core.Pose2;
import org.firstinspires.ftc.teamcode.localization.core.RobotState;

public final class AbsolutePoseFactor implements Factor {
    private final String source;
    private final long timestampNanos;
    private final Pose2 pose;
    private final SimpleMatrix covariance;

    public AbsolutePoseFactor(String source, long timestampNanos, Pose2 pose, SimpleMatrix covariance) {
        if (source == null || source.isEmpty()) throw new IllegalArgumentException("source is empty");
        if (timestampNanos < 0) throw new IllegalArgumentException("timestamp is negative");
        if (pose == null) throw new IllegalArgumentException("pose is null");
        this.source = source;
        this.timestampNanos = timestampNanos;
        this.pose = pose;
        this.covariance = MatrixOps.requirePositiveDefinite(covariance, 3, "factor covariance");
    }

    @Override public String source() { return source; }
    @Override public long timestampNanos() { return timestampNanos; }
    @Override public long[] timestamps() { return new long[] {timestampNanos}; }
    public Pose2 pose() { return pose; }
    public SimpleMatrix measurementCovariance() { return covariance.copy(); }
    @Override public SimpleMatrix covariance(StateValues values) { return covariance.copy(); }

    @Override public double[] residual(StateValues values) {
        double[] state = values.state(timestampNanos);
        if (state.length != RobotState.DIMENSION) {
            throw new IllegalArgumentException("predicted state must have 6 dimensions");
        }
        return new double[] {
                state[RobotState.X] - pose.x(),
                state[RobotState.Y] - pose.y(),
                Pose2.angularResidual(state[RobotState.HEADING], pose.heading())
        };
    }

    @Override public double residualDifference(int row, double high, double low) {
        return row == 2 ? Pose2.angularResidual(high, low) : high - low;
    }

    @Override public LinearizedFactor linearize(StateValues values) {
        return new LinearizedFactor(timestamps(), residual(values), covariance(values),
                analyticJacobians(values));
    }

    SimpleMatrix[] analyticJacobians(StateValues values) {
        double[] state = values.state(timestampNanos);
        if (state.length != RobotState.DIMENSION) {
            throw new IllegalArgumentException("predicted state must have 6 dimensions");
        }
        SimpleMatrix jacobian = new SimpleMatrix(3, RobotState.DIMENSION);
        jacobian.set(0, RobotState.X, 1);
        jacobian.set(1, RobotState.Y, 1);
        jacobian.set(2, RobotState.HEADING, 1);
        return new SimpleMatrix[] {jacobian};
    }
}
