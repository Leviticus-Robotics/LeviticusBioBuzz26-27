package org.firstinspires.ftc.teamcode.localization.engine;

import org.ejml.simple.SimpleMatrix;
import org.firstinspires.ftc.teamcode.fusion.core.Gaussian;
import org.firstinspires.ftc.teamcode.fusion.core.MatrixOps;
import org.firstinspires.ftc.teamcode.fusion.engine.ProcessModel;
import org.firstinspires.ftc.teamcode.localization.core.ControlInput;
import org.firstinspires.ftc.teamcode.localization.core.Pose2;
import org.firstinspires.ftc.teamcode.localization.core.RobotState;

/** First-order command response model with a full numerically-linearized covariance transition. */
public final class PoseVelocityProcessModel implements ProcessModel<ControlInput> {
    private final double velocityResponse;
    private final double maximumForwardVelocity;
    private final double maximumStrafeVelocity;
    private final double maximumAngularVelocity;
    private final double[] noisePerSecond;

    public PoseVelocityProcessModel(double velocityResponse, double maximumForwardVelocity,
                                    double maximumStrafeVelocity, double maximumAngularVelocity,
                                    double[] noisePerSecond) {
        if (!Double.isFinite(velocityResponse) || velocityResponse < 0) {
            throw new IllegalArgumentException("velocity response must be finite and nonnegative");
        }
        if (!Double.isFinite(maximumForwardVelocity) || maximumForwardVelocity <= 0
                || !Double.isFinite(maximumStrafeVelocity) || maximumStrafeVelocity <= 0
                || !Double.isFinite(maximumAngularVelocity) || maximumAngularVelocity <= 0) {
            throw new IllegalArgumentException("maximum model velocities must be positive and finite");
        }
        this.velocityResponse = velocityResponse;
        this.maximumForwardVelocity = maximumForwardVelocity;
        this.maximumStrafeVelocity = maximumStrafeVelocity;
        this.maximumAngularVelocity = maximumAngularVelocity;
        this.noisePerSecond = MatrixOps.copyVector(noisePerSecond, "process noise");
        if (noisePerSecond.length != RobotState.DIMENSION) throw new IllegalArgumentException("process noise must have 6 values");
        for (double noise : noisePerSecond) if (noise <= 0) throw new IllegalArgumentException("process noise must be positive");
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

    @Override public double residualDifference(int row, double high, double low) {
        return row == RobotState.HEADING ? Pose2.angularResidual(high, low) : high - low;
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

    private SimpleMatrix numericalJacobian(double[] state, ControlInput input, double dt) {
        SimpleMatrix result = new SimpleMatrix(RobotState.DIMENSION, RobotState.DIMENSION);
        double epsilon = 1e-6;
        for (int column = 0; column < RobotState.DIMENSION; column++) {
            double[] plus = state.clone();
            double[] minus = state.clone();
            plus[column] += epsilon;
            minus[column] -= epsilon;
            double[] high = transition(plus, input, dt);
            double[] low = transition(minus, input, dt);
            for (int row = 0; row < RobotState.DIMENSION; row++) {
                double difference = row == RobotState.HEADING
                        ? Pose2.angularResidual(high[row], low[row]) : high[row] - low[row];
                result.set(row, column, difference / (2.0 * epsilon));
            }
        }
        return result;
    }
}
