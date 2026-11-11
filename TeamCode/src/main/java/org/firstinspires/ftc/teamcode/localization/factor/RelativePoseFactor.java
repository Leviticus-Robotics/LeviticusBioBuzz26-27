package org.firstinspires.ftc.teamcode.localization.factor;

import org.ejml.simple.SimpleMatrix;
import org.firstinspires.ftc.teamcode.fusion.core.Gaussian;
import org.firstinspires.ftc.teamcode.fusion.core.MatrixOps;
import org.firstinspires.ftc.teamcode.fusion.factor.RelativeFactor;
import org.firstinspires.ftc.teamcode.localization.core.Pose2;
import org.firstinspires.ftc.teamcode.localization.core.RobotState;

public final class RelativePoseFactor implements RelativeFactor {
    private final String source;
    private final long fromTimestampNanos;
    private final long timestampNanos;
    private final Pose2 delta;
    private final SimpleMatrix covariance;

    public RelativePoseFactor(String source, long fromTimestampNanos, long timestampNanos,
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
    @Override public long timestampNanos() { return timestampNanos; }
    public Pose2 delta() { return delta; }
    public SimpleMatrix covariance() { return covariance.copy(); }

    @Override public Gaussian apply(Gaussian stateAtFromTimestamp) {
        if (stateAtFromTimestamp.dimension() != RobotState.DIMENSION) {
            throw new IllegalArgumentException("transition state must have 6 dimensions");
        }
        double elapsedSeconds = (timestampNanos - fromTimestampNanos) * 1e-9;
        double[] state = stateAtFromTimestamp.mean();
        double[] deltaVector = delta.toArray();
        double[] transitioned = transition(state, deltaVector, elapsedSeconds);
        SimpleMatrix stateJacobian = numericalJacobian(state, deltaVector, elapsedSeconds, true);
        SimpleMatrix deltaJacobian = numericalJacobian(state, deltaVector, elapsedSeconds, false);
        SimpleMatrix transitionedCovariance = stateJacobian.mult(stateAtFromTimestamp.covariance())
                .mult(stateJacobian.transpose())
                .plus(deltaJacobian.mult(covariance).mult(deltaJacobian.transpose()));
        return new Gaussian(transitioned, MatrixOps.symmetrize(transitionedCovariance));
    }

    private static double[] transition(double[] state, double[] delta, double elapsedSeconds) {
        Pose2 previous = new Pose2(state[RobotState.X], state[RobotState.Y], state[RobotState.HEADING]);
        Pose2 next = previous.compose(new Pose2(delta[0], delta[1], delta[2]));
        return new double[] {
                next.x(), next.y(), next.heading(),
                (next.x() - previous.x()) / elapsedSeconds,
                (next.y() - previous.y()) / elapsedSeconds,
                Pose2.angularResidual(next.heading(), previous.heading()) / elapsedSeconds
        };
    }

    private static SimpleMatrix numericalJacobian(double[] state, double[] delta, double elapsedSeconds,
                                                   boolean withRespectToState) {
        int columns = withRespectToState ? RobotState.DIMENSION : 3;
        SimpleMatrix jacobian = new SimpleMatrix(RobotState.DIMENSION, columns);
        double epsilon = 1e-6;
        for (int column = 0; column < columns; column++) {
            double[] highState = state.clone();
            double[] lowState = state.clone();
            double[] highDelta = delta.clone();
            double[] lowDelta = delta.clone();
            if (withRespectToState) {
                highState[column] += epsilon;
                lowState[column] -= epsilon;
            } else {
                highDelta[column] += epsilon;
                lowDelta[column] -= epsilon;
            }
            double[] high = transition(highState, highDelta, elapsedSeconds);
            double[] low = transition(lowState, lowDelta, elapsedSeconds);
            for (int row = 0; row < RobotState.DIMENSION; row++) {
                double difference = row == RobotState.HEADING
                        ? Pose2.angularResidual(high[row], low[row]) : high[row] - low[row];
                jacobian.set(row, column, difference / (2 * epsilon));
            }
        }
        return jacobian;
    }
}
