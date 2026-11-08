package org.firstinspires.ftc.teamcode.localization.factor;

import org.ejml.simple.SimpleMatrix;
import org.firstinspires.ftc.teamcode.fusion.core.Gaussian;
import org.firstinspires.ftc.teamcode.fusion.core.MatrixOps;
import org.firstinspires.ftc.teamcode.fusion.factor.MeasurementFactor;
import org.firstinspires.ftc.teamcode.localization.core.Pose2;
import org.firstinspires.ftc.teamcode.localization.core.RobotState;

public final class AbsolutePoseFactor implements MeasurementFactor {
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
    public Pose2 pose() { return pose; }
    public SimpleMatrix covariance() { return covariance.copy(); }

    @Override public Gaussian measurement(Gaussian predictedState) {
        if (predictedState.dimension() != RobotState.DIMENSION) {
            throw new IllegalArgumentException("predicted state must have 6 dimensions");
        }
        double predictedHeading = predictedState.mean(RobotState.HEADING);
        double unwrappedHeading = predictedHeading
                + Pose2.angularResidual(pose.heading(), predictedHeading);
        return new Gaussian(new double[] {pose.x(), pose.y(), unwrappedHeading}, covariance);
    }

    @Override public SimpleMatrix stateToMeasurement(Gaussian predictedState) {
        SimpleMatrix model = new SimpleMatrix(3, predictedState.dimension());
        model.set(0, RobotState.X, 1);
        model.set(1, RobotState.Y, 1);
        model.set(2, RobotState.HEADING, 1);
        return model;
    }
}
