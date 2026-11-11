package org.firstinspires.ftc.teamcode.localization.factor;

import org.ejml.simple.SimpleMatrix;
import org.firstinspires.ftc.teamcode.fusion.core.MatrixOps;
import org.firstinspires.ftc.teamcode.fusion.factor.RelativeFactor;
import org.firstinspires.ftc.teamcode.fusion.factor.LinearizedFactor;
import org.firstinspires.ftc.teamcode.fusion.factor.StateValues;
import org.firstinspires.ftc.teamcode.localization.core.Pose2;
import org.firstinspires.ftc.teamcode.localization.core.RobotState;

/** Joint observation of a relative pose delta and current robot-frame velocity. */
public final class MotionDeltaFactor implements RelativeFactor {
    private final String source;
    private final PoseDeltaFactor poseFactor;
    private final long fromTimestampNanos;
    private final long toTimestampNanos;
    private final double forwardVelocity;
    private final double strafeVelocity;
    private final double omega;
    private final SimpleMatrix covariance;

    public MotionDeltaFactor(String source, long fromTimestampNanos, long toTimestampNanos,
                             Pose2 delta, double forwardVelocity, double strafeVelocity,
                             double omega, SimpleMatrix covariance) {
        if (source == null || source.isEmpty()) throw new IllegalArgumentException("source is empty");
        if (!Double.isFinite(forwardVelocity) || !Double.isFinite(strafeVelocity)
                || !Double.isFinite(omega)) {
            throw new IllegalArgumentException("motion velocity is not finite");
        }
        this.source = source;
        poseFactor = new PoseDeltaFactor(source, fromTimestampNanos, toTimestampNanos, delta,
                MatrixOps.diagonal(1, 1, 1));
        this.fromTimestampNanos = fromTimestampNanos;
        this.toTimestampNanos = toTimestampNanos;
        this.forwardVelocity = forwardVelocity;
        this.strafeVelocity = strafeVelocity;
        this.omega = omega;
        this.covariance = MatrixOps.requirePositiveDefinite(
                covariance, RobotState.DIMENSION, "motion-delta covariance");
    }

    @Override public String source() { return source; }
    @Override public long fromTimestampNanos() { return fromTimestampNanos; }
    @Override public long toTimestampNanos() { return toTimestampNanos; }
    @Override public String observationId() {
        return source + ":" + fromTimestampNanos + ":" + toTimestampNanos;
    }

    public Pose2 delta() { return poseFactor.delta(); }
    public double forwardVelocity() { return forwardVelocity; }
    public double strafeVelocity() { return strafeVelocity; }
    public double omega() { return omega; }
    public SimpleMatrix measurementCovariance() { return covariance.copy(); }

    public double[] residual(StateValues values) {
        double[] poseResidual = poseFactor.residual(values);
        double[] previous = values.state(fromTimestampNanos);
        double[] current = values.state(toTimestampNanos);
        double heading = previous[RobotState.HEADING] + delta().heading();
        double cosine = Math.cos(heading);
        double sine = Math.sin(heading);
        double measuredVx = cosine*forwardVelocity - sine*strafeVelocity;
        double measuredVy = sine*forwardVelocity + cosine*strafeVelocity;
        return new double[] {
                poseResidual[0], poseResidual[1], poseResidual[2],
                current[RobotState.VX] - measuredVx,
                current[RobotState.VY] - measuredVy,
                current[RobotState.OMEGA] - omega
        };
    }

    public SimpleMatrix[] jacobians(StateValues values) {
        SimpleMatrix[] poseJacobians = poseFactor.jacobians(values);
        SimpleMatrix from = new SimpleMatrix(RobotState.DIMENSION, RobotState.DIMENSION);
        SimpleMatrix to = new SimpleMatrix(RobotState.DIMENSION, RobotState.DIMENSION);
        from.insertIntoThis(0, 0, poseJacobians[0]);
        to.insertIntoThis(0, 0, poseJacobians[1]);
        double[] previous = values.state(fromTimestampNanos);
        double heading = previous[RobotState.HEADING] + delta().heading();
        double cosine = Math.cos(heading);
        double sine = Math.sin(heading);
        double measuredVx = cosine*forwardVelocity - sine*strafeVelocity;
        double measuredVy = sine*forwardVelocity + cosine*strafeVelocity;
        from.set(RobotState.VX, RobotState.HEADING, measuredVy);
        from.set(RobotState.VY, RobotState.HEADING, -measuredVx);
        to.set(RobotState.VX, RobotState.VX, 1);
        to.set(RobotState.VY, RobotState.VY, 1);
        to.set(RobotState.OMEGA, RobotState.OMEGA, 1);
        return new SimpleMatrix[] {from, to};
    }

    @Override public LinearizedFactor linearize(StateValues values) {
        return new LinearizedFactor(residual(values), covariance, jacobians(values));
    }
}
