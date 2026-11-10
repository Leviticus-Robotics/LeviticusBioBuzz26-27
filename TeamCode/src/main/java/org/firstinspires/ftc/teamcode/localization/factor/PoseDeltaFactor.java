package org.firstinspires.ftc.teamcode.localization.factor;

import org.ejml.simple.SimpleMatrix;
import org.firstinspires.ftc.teamcode.fusion.core.MatrixOps;
import org.firstinspires.ftc.teamcode.fusion.factor.RelativeFactor;
import org.firstinspires.ftc.teamcode.fusion.factor.StateValues;
import org.firstinspires.ftc.teamcode.localization.core.Pose2;
import org.firstinspires.ftc.teamcode.localization.core.RobotState;

/** One relative pose-delta measurement between consecutive loop ticks. */
public final class PoseDeltaFactor implements RelativeFactor {
    private final String source;
    private final long fromTimestampNanos;
    private final long timestampNanos;
    private final Pose2 delta;
    private final SimpleMatrix covariance;

    public PoseDeltaFactor(String source, long fromTimestampNanos, long timestampNanos,
                           Pose2 delta, SimpleMatrix covariance) {
        if (source == null || source.isEmpty()) throw new IllegalArgumentException("source is empty");
        if (fromTimestampNanos < 0 || timestampNanos <= fromTimestampNanos) {
            throw new IllegalArgumentException("relative factor timestamps are invalid");
        }
        if (delta == null) throw new IllegalArgumentException("delta is null");
        this.source = source;
        this.fromTimestampNanos = fromTimestampNanos;
        this.timestampNanos = timestampNanos;
        this.delta = delta;
        this.covariance = MatrixOps.requirePositiveDefinite(covariance, 3, "factor covariance");
    }

    @Override public String source() { return source; }
    @Override public long fromTimestampNanos() { return fromTimestampNanos; }
    @Override public long toTimestampNanos() { return timestampNanos; }
    @Override public long timestampNanos() { return timestampNanos; }
    @Override public String observationId() {
        return source + ":" + fromTimestampNanos + ":" + timestampNanos;
    }
    public Pose2 delta() { return delta; }
    public SimpleMatrix covariance() { return covariance.copy(); }

    public double[] residual(double[] fromState, double[] toState) {
        requireState(fromState);
        requireState(toState);
        return delta.inverse().compose(pose(fromState).localDeltaTo(pose(toState))).toArray();
    }

    public SimpleMatrix[] jacobians(double[] fromState, double[] toState) {
        requireState(fromState);
        requireState(toState);
        double heading = fromState[RobotState.HEADING];
        double cos = Math.cos(heading);
        double sin = Math.sin(heading);
        double globalDx = toState[RobotState.X] - fromState[RobotState.X];
        double globalDy = toState[RobotState.Y] - fromState[RobotState.Y];
        double predictedDx = cos*globalDx + sin*globalDy;
        double predictedDy = -sin*globalDx + cos*globalDy;
        double measuredCos = Math.cos(delta.heading());
        double measuredSin = Math.sin(delta.heading());
        SimpleMatrix rotation = new SimpleMatrix(new double[][] {
                {measuredCos, measuredSin}, {-measuredSin, measuredCos}
        });
        SimpleMatrix fromTranslation = rotation.mult(new SimpleMatrix(new double[][] {
                {-cos, -sin, predictedDy}, {sin, -cos, -predictedDx}
        }));
        SimpleMatrix toTranslation = rotation.mult(new SimpleMatrix(new double[][] {
                {cos, sin, 0}, {-sin, cos, 0}
        }));

        SimpleMatrix fromJacobian = new SimpleMatrix(3, RobotState.DIMENSION);
        SimpleMatrix toJacobian = new SimpleMatrix(3, RobotState.DIMENSION);
        for (int row = 0; row < 2; row++) {
            fromJacobian.set(row, RobotState.X, fromTranslation.get(row, 0));
            fromJacobian.set(row, RobotState.Y, fromTranslation.get(row, 1));
            fromJacobian.set(row, RobotState.HEADING, fromTranslation.get(row, 2));
            toJacobian.set(row, RobotState.X, toTranslation.get(row, 0));
            toJacobian.set(row, RobotState.Y, toTranslation.get(row, 1));
        }
        fromJacobian.set(2, RobotState.HEADING, -1);
        toJacobian.set(2, RobotState.HEADING, 1);
        return new SimpleMatrix[] {fromJacobian, toJacobian};
    }

    @Override public double[] residual(StateValues values) {
        return residual(values.state(fromTimestampNanos), values.state(timestampNanos));
    }

    @Override public SimpleMatrix covariance(StateValues values) { return covariance(); }

    @Override public SimpleMatrix[] jacobians(StateValues values) {
        return jacobians(values.state(fromTimestampNanos), values.state(timestampNanos));
    }

    private static Pose2 pose(double[] state) {
        return new Pose2(state[RobotState.X], state[RobotState.Y], state[RobotState.HEADING]);
    }

    private static void requireState(double[] state) {
        if (state == null || state.length != RobotState.DIMENSION) {
            throw new IllegalArgumentException("pose-delta states must have 6 dimensions");
        }
    }
}
