package org.firstinspires.ftc.teamcode.localization.prediction;

import com.pedropathing.localization.MotionState;
import com.pedropathing.math.Pose;
import com.pedropathing.math.Velocity;
import com.pedropathing.utils.Angle;

/** First-order command-response model used only for temporary future prediction. */
public final class PoseVelocityProcessModel implements ProcessModel<ControlInput> {
    private final double velocityResponse;
    private final double maximumForwardVelocity;
    private final double maximumStrafeVelocity;
    private final double maximumAngularVelocity;

    public PoseVelocityProcessModel(Config config) {
        if (config == null) throw new IllegalArgumentException("process config is null");
        velocityResponse = config.velocityResponse();
        maximumForwardVelocity = config.maximumForwardVelocity();
        maximumStrafeVelocity = config.maximumStrafeVelocity();
        maximumAngularVelocity = config.maximumAngularVelocity();
    }

    @Override public MotionState predict(MotionState initial, ControlInput input,
                                         double elapsedSeconds) {
        if (initial == null) throw new IllegalArgumentException("initial state is null");
        if (input == null) throw new IllegalArgumentException("input is null");
        if (!Double.isFinite(elapsedSeconds) || elapsedSeconds < 0) {
            throw new IllegalArgumentException("elapsed time is invalid");
        }
        Pose pose = initial.pose();
        Velocity previous = initial.velocity();
        double heading = pose.heading();
        double commandedForward = input.forwardPower * maximumForwardVelocity;
        double commandedStrafe = input.strafePower * maximumStrafeVelocity;
        double targetVx = Math.cos(heading)*commandedForward
                - Math.sin(heading)*commandedStrafe;
        double targetVy = Math.sin(heading)*commandedForward
                + Math.cos(heading)*commandedStrafe;
        double targetOmega = input.turnPower * maximumAngularVelocity;
        double blend = 1.0 - Math.exp(-velocityResponse*elapsedSeconds);
        double vx = previous.vx + blend*(targetVx - previous.vx);
        double vy = previous.vy + blend*(targetVy - previous.vy);
        double omega = previous.omega + blend*(targetOmega - previous.omega);
        Pose predictedPose = new Pose(
                pose.x() + .5*(previous.vx + vx)*elapsedSeconds,
                pose.y() + .5*(previous.vy + vy)*elapsedSeconds,
                Angle.normalize(pose.heading()
                        + .5*(previous.omega + omega)*elapsedSeconds));
        return MotionState.ofVelocity(predictedPose, new Velocity(vx, vy, omega));
    }

    /** Immutable command-response configuration. */
    public static final class Config {
        private final double velocityResponse;
        private final double maximumForwardVelocity;
        private final double maximumStrafeVelocity;
        private final double maximumAngularVelocity;

        public Config(double velocityResponse, double maximumForwardVelocity,
                      double maximumStrafeVelocity, double maximumAngularVelocity) {
            if (!Double.isFinite(velocityResponse) || velocityResponse < 0) {
                throw new IllegalArgumentException(
                        "velocity response must be finite and nonnegative");
            }
            if (!Double.isFinite(maximumForwardVelocity) || maximumForwardVelocity <= 0
                    || !Double.isFinite(maximumStrafeVelocity) || maximumStrafeVelocity <= 0
                    || !Double.isFinite(maximumAngularVelocity)
                    || maximumAngularVelocity <= 0) {
                throw new IllegalArgumentException(
                        "maximum model velocities must be positive and finite");
            }
            this.velocityResponse = velocityResponse;
            this.maximumForwardVelocity = maximumForwardVelocity;
            this.maximumStrafeVelocity = maximumStrafeVelocity;
            this.maximumAngularVelocity = maximumAngularVelocity;
        }

        public static Config defaults() { return new Config(8, 80, 80, 4); }
        public double velocityResponse() { return velocityResponse; }
        public double maximumForwardVelocity() { return maximumForwardVelocity; }
        public double maximumStrafeVelocity() { return maximumStrafeVelocity; }
        public double maximumAngularVelocity() { return maximumAngularVelocity; }
    }
}
