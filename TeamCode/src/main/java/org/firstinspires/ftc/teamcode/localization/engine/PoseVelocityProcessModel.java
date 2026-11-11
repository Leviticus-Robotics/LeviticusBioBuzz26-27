package org.firstinspires.ftc.teamcode.localization.engine;

import org.ejml.simple.SimpleMatrix;
import org.firstinspires.ftc.teamcode.fusion.core.MatrixOps;
import org.firstinspires.ftc.teamcode.fusion.engine.ProcessModel;
import org.firstinspires.ftc.teamcode.localization.core.ControlInput;
import org.firstinspires.ftc.teamcode.localization.core.Pose2;
import org.firstinspires.ftc.teamcode.localization.core.RobotState;

/** First-order command response model using ProcessModel's numerical transition Jacobian. */
public final class PoseVelocityProcessModel implements ProcessModel<ControlInput> {
    private final double velocityResponse;
    private final double maximumForwardVelocity;
    private final double maximumStrafeVelocity;
    private final double maximumAngularVelocity;
    private final double[] noisePerSecond;

    public PoseVelocityProcessModel(Config config) {
        if (config == null) throw new IllegalArgumentException("process config is null");
        velocityResponse = config.velocityResponse();
        maximumForwardVelocity = config.maximumForwardVelocity();
        maximumStrafeVelocity = config.maximumStrafeVelocity();
        maximumAngularVelocity = config.maximumAngularVelocity();
        noisePerSecond = config.noisePerSecond();
    }

    @Override public double[] predictMean(double[] prior, ControlInput input, double elapsedSeconds) {
        if (prior.length != RobotState.DIMENSION) throw new IllegalArgumentException("prior must have 6 dimensions");
        if (!Double.isFinite(elapsedSeconds) || elapsedSeconds < 0) throw new IllegalArgumentException("elapsed time is invalid");
        return transition(prior, input, elapsedSeconds);
    }

    @Override public SimpleMatrix processNoise(double[] prior, ControlInput input,
                                               double elapsedSeconds) {
        SimpleMatrix q = new SimpleMatrix(RobotState.DIMENSION, RobotState.DIMENSION);
        for (int i = 0; i < RobotState.DIMENSION; i++) q.set(i, i, noisePerSecond[i] * elapsedSeconds);
        return q;
    }

    @Override public double[] residual(double[] predicted, double[] actual) {
        double[] result = ProcessModel.super.residual(predicted, actual);
        result[RobotState.HEADING] = Pose2.angularResidual(
                actual[RobotState.HEADING], predicted[RobotState.HEADING]);
        return result;
    }

    @Override public double[] normalize(double[] state) {
        double[] result = state.clone();
        result[RobotState.HEADING] = Pose2.wrap(result[RobotState.HEADING]);
        return result;
    }

    private double[] transition(double[] state, ControlInput input, double dt) {
        double heading = state[RobotState.HEADING];
        double commandedForwardVelocity = input.forwardPower * maximumForwardVelocity;
        double commandedStrafeVelocity = input.strafePower * maximumStrafeVelocity;
        double targetVx = Math.cos(heading) * commandedForwardVelocity
                - Math.sin(heading) * commandedStrafeVelocity;
        double targetVy = Math.sin(heading) * commandedForwardVelocity
                + Math.cos(heading) * commandedStrafeVelocity;
        double targetOmega = input.turnPower * maximumAngularVelocity;
        double blend = 1.0 - Math.exp(-velocityResponse * dt);
        double vx = state[RobotState.VX] + blend * (targetVx - state[RobotState.VX]);
        double vy = state[RobotState.VY] + blend * (targetVy - state[RobotState.VY]);
        double omega = state[RobotState.OMEGA] + blend * (targetOmega - state[RobotState.OMEGA]);
        return new double[] {
                state[RobotState.X] + 0.5 * (state[RobotState.VX] + vx) * dt,
                state[RobotState.Y] + 0.5 * (state[RobotState.VY] + vy) * dt,
                Pose2.wrap(heading + 0.5 * (state[RobotState.OMEGA] + omega) * dt),
                vx, vy, omega
        };
    }

    /** Immutable process-dynamics and process-noise configuration. */
    public static final class Config {
        private final double velocityResponse;
        private final double maximumForwardVelocity;
        private final double maximumStrafeVelocity;
        private final double maximumAngularVelocity;
        private final double[] noisePerSecond;

        public Config(double velocityResponse, double maximumForwardVelocity,
                      double maximumStrafeVelocity, double maximumAngularVelocity,
                      double[] noisePerSecond) {
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
            this.noisePerSecond = MatrixOps.copyVector(noisePerSecond, "process noise");
            if (this.noisePerSecond.length != RobotState.DIMENSION) {
                throw new IllegalArgumentException("process noise must have 6 values");
            }
            for (double noise : this.noisePerSecond) {
                if (noise <= 0) throw new IllegalArgumentException("process noise must be positive");
            }
            this.velocityResponse = velocityResponse;
            this.maximumForwardVelocity = maximumForwardVelocity;
            this.maximumStrafeVelocity = maximumStrafeVelocity;
            this.maximumAngularVelocity = maximumAngularVelocity;
        }

        public static Config defaults() {
            return new Config(8, 80, 80, 4,
                    // Variance accumulated per second for x, y, heading, vx, vy, omega.
                    // At 50 Hz this becomes [.1, .1, .004, 1, 1, .06], keeping the
                    // command model substantially weaker than measured OctoQuad motion.
                    new double[] {5, 5, .2, 50, 50, 3});
        }

        public Config withMaximumVelocities(double forward, double strafe, double angular) {
            return new Config(velocityResponse, forward, strafe, angular, noisePerSecond);
        }

        public double velocityResponse() { return velocityResponse; }
        public double maximumForwardVelocity() { return maximumForwardVelocity; }
        public double maximumStrafeVelocity() { return maximumStrafeVelocity; }
        public double maximumAngularVelocity() { return maximumAngularVelocity; }
        public double[] noisePerSecond() { return noisePerSecond.clone(); }
    }

}
