package org.firstinspires.ftc.teamcode.localization;

import org.ejml.simple.SimpleMatrix;
import org.firstinspires.ftc.teamcode.fusion.core.MatrixOps;
import org.firstinspires.ftc.teamcode.localization.core.Pose2;
import org.firstinspires.ftc.teamcode.localization.core.RobotState;

/** Immutable covariance and sensor-fusion tuning used to construct localization. */
public final class LocalizationConfig {
    private final SimpleMatrix initialCovariance;
    private final double[] processNoisePerSecond;
    private final SimpleMatrix octoQuadMotionCovariance;
    private final SimpleMatrix aprilTagCovariance;
    private final SimpleMatrix sharedCameraCovarianceFloor;
    private final double aprilTagConsensusGateSquared;
    private final double aprilTagDisagreementInflation;
    private final Pose2 fieldConventionTransform;

    public LocalizationConfig(SimpleMatrix initialCovariance, double[] processNoisePerSecond,
                              SimpleMatrix octoQuadMotionCovariance, SimpleMatrix aprilTagCovariance,
                              SimpleMatrix sharedCameraCovarianceFloor,
                              double aprilTagConsensusGateSquared,
                              double aprilTagDisagreementInflation,
                              Pose2 fieldConventionTransform) {
        this.initialCovariance = MatrixOps.requirePositiveDefinite(
                initialCovariance, RobotState.DIMENSION, "initial covariance");
        this.processNoisePerSecond = MatrixOps.copyVector(processNoisePerSecond, "process noise");
        if (this.processNoisePerSecond.length != RobotState.DIMENSION) {
            throw new IllegalArgumentException("process noise must have 6 values");
        }
        for (double noise : this.processNoisePerSecond) {
            if (noise <= 0) throw new IllegalArgumentException("process noise must be positive");
        }
        this.octoQuadMotionCovariance = MatrixOps.requirePositiveDefinite(
                octoQuadMotionCovariance, 6, "OctoQuad motion covariance");
        this.aprilTagCovariance = MatrixOps.requirePositiveDefinite(
                aprilTagCovariance, 3, "AprilTag covariance");
        this.sharedCameraCovarianceFloor = MatrixOps.requirePositiveDefinite(
                sharedCameraCovarianceFloor, 3, "shared camera covariance floor");
        if (!Double.isFinite(aprilTagConsensusGateSquared) || aprilTagConsensusGateSquared <= 0
                || !Double.isFinite(aprilTagDisagreementInflation)
                || aprilTagDisagreementInflation < 0) {
            throw new IllegalArgumentException("AprilTag gate and inflation are invalid");
        }
        if (fieldConventionTransform == null) {
            throw new IllegalArgumentException("field convention transform is null");
        }
        this.aprilTagConsensusGateSquared = aprilTagConsensusGateSquared;
        this.aprilTagDisagreementInflation = aprilTagDisagreementInflation;
        this.fieldConventionTransform = fieldConventionTransform;
    }

    public static LocalizationConfig defaults() {
        return new LocalizationConfig(
                MatrixOps.diagonal(4, 4, .25, 25, 25, 4),
                new double[] {.5, .5, .03, 8, 8, .5},
                MatrixOps.diagonal(.02, .02, .002, 4, 4, .1),
                MatrixOps.diagonal(2.25, 2.25, .04),
                MatrixOps.diagonal(.25, .25, .01),
                16.27, 1.0, Pose2.zero());
    }

    public SimpleMatrix initialCovariance() { return initialCovariance.copy(); }
    public double[] processNoisePerSecond() { return processNoisePerSecond.clone(); }
    public SimpleMatrix octoQuadMotionCovariance() { return octoQuadMotionCovariance.copy(); }
    public SimpleMatrix aprilTagCovariance() { return aprilTagCovariance.copy(); }
    public SimpleMatrix sharedCameraCovarianceFloor() { return sharedCameraCovarianceFloor.copy(); }
    public double aprilTagConsensusGateSquared() { return aprilTagConsensusGateSquared; }
    public double aprilTagDisagreementInflation() { return aprilTagDisagreementInflation; }
    public Pose2 fieldConventionTransform() { return fieldConventionTransform; }
}
