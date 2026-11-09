package org.firstinspires.ftc.teamcode.localization.factor;

import org.ejml.simple.SimpleMatrix;
import org.firstinspires.ftc.teamcode.fusion.core.MatrixOps;
import org.firstinspires.ftc.teamcode.fusion.factor.BetweenFactor;
import org.firstinspires.ftc.teamcode.fusion.factor.StateValues;
import org.firstinspires.ftc.teamcode.localization.core.Pose2;
import org.firstinspires.ftc.teamcode.localization.core.RobotState;

public final class RelativePoseFactor implements BetweenFactor {
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
    @Override public long toTimestampNanos() { return timestampNanos; }
    @Override public long timestampNanos() { return timestampNanos; }
    public Pose2 delta() { return delta; }
    @Override public SimpleMatrix covariance(StateValues values) { return covariance.copy(); }

    @Override public double[] residual(StateValues values) {
        double[] fromState = values.state(fromTimestampNanos);
        double[] toState = values.state(timestampNanos);
        if (fromState.length != RobotState.DIMENSION || toState.length != RobotState.DIMENSION) {
            throw new IllegalArgumentException("relative-pose states must have 6 dimensions");
        }
        Pose2 from = new Pose2(fromState[RobotState.X], fromState[RobotState.Y],
                fromState[RobotState.HEADING]);
        Pose2 to = new Pose2(toState[RobotState.X], toState[RobotState.Y],
                toState[RobotState.HEADING]);
        Pose2 predictedDelta = from.localDeltaTo(to);
        Pose2 error = delta.inverse().compose(predictedDelta);
        return error.toArray();
    }

    @Override public double residualDifference(int row, double high, double low) {
        return row == 2 ? Pose2.angularResidual(high, low) : high - low;
    }

    /** Analytic Jacobian candidate; kept separate until validated and selected for production. */
    SimpleMatrix[] analyticJacobians(StateValues values) {
        double[] fromState = values.state(fromTimestampNanos);
        double[] toState = values.state(timestampNanos);
        if (fromState.length != RobotState.DIMENSION || toState.length != RobotState.DIMENSION) {
            throw new IllegalArgumentException("relative-pose states must have 6 dimensions");
        }

        double fromHeading = fromState[RobotState.HEADING];
        double cosFrom = Math.cos(fromHeading);
        double sinFrom = Math.sin(fromHeading);
        double globalDx = toState[RobotState.X] - fromState[RobotState.X];
        double globalDy = toState[RobotState.Y] - fromState[RobotState.Y];
        double predictedDx = cosFrom*globalDx + sinFrom*globalDy;
        double predictedDy = -sinFrom*globalDx + cosFrom*globalDy;

        double cosMeasured = Math.cos(delta.heading());
        double sinMeasured = Math.sin(delta.heading());
        SimpleMatrix rotateIntoError = new SimpleMatrix(new double[][] {
                {cosMeasured, sinMeasured},
                {-sinMeasured, cosMeasured}
        });

        SimpleMatrix fromTranslation = new SimpleMatrix(new double[][] {
                {-cosFrom, -sinFrom, predictedDy},
                { sinFrom, -cosFrom, -predictedDx}
        });
        SimpleMatrix toTranslation = new SimpleMatrix(new double[][] {
                {cosFrom, sinFrom, 0},
                {-sinFrom, cosFrom, 0}
        });
        fromTranslation = rotateIntoError.mult(fromTranslation);
        toTranslation = rotateIntoError.mult(toTranslation);

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

}
