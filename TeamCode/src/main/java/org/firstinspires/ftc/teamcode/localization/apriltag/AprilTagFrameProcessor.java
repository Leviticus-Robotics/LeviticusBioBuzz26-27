package org.firstinspires.ftc.teamcode.localization.apriltag;

import com.pedropathing.math.Pose;
import com.pedropathing.utils.Angle;
import java.util.ArrayList;
import java.util.List;
import java.util.function.Predicate;
import org.ejml.simple.SimpleMatrix;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.teamcode.fusion.Gaussian;
import org.firstinspires.ftc.teamcode.fusion.GaussianFusion;
import org.firstinspires.ftc.teamcode.fusion.MatrixOps;

/** FTC Vision boundary and conservative same-frame AprilTag combiner. */
public final class AprilTagFrameProcessor {
    private final Predicate<AprilTagDetection> stationaryReference;
    private final SimpleMatrix perDetectionCovariance;
    private final SimpleMatrix sharedCameraCovarianceFloor;
    private final double consensusGateSquared;
    private final double disagreementInflation;
    private final Pose fieldConventionTransform;
    private long lastFrameTimestampNanos = Long.MIN_VALUE;

    public AprilTagFrameProcessor(Predicate<AprilTagDetection> stationaryReference,
                                  SimpleMatrix perDetectionCovariance,
                                  SimpleMatrix sharedCameraCovarianceFloor,
                                  double consensusGateSquared,
                                  double disagreementInflation,
                                  Pose fieldConventionTransform) {
        if (stationaryReference == null) throw new IllegalArgumentException("tag predicate is null");
        if (fieldConventionTransform == null) throw new IllegalArgumentException("field transform is null");
        this.stationaryReference = stationaryReference;
        this.perDetectionCovariance = MatrixOps.requirePositiveDefinite(
                perDetectionCovariance, 3, "per-detection covariance");
        this.sharedCameraCovarianceFloor = MatrixOps.requirePositiveDefinite(
                sharedCameraCovarianceFloor, 3, "shared camera covariance floor");
        if (!Double.isFinite(consensusGateSquared) || consensusGateSquared <= 0
                || !Double.isFinite(disagreementInflation) || disagreementInflation < 0) {
            throw new IllegalArgumentException("AprilTag gate and inflation are invalid");
        }
        this.consensusGateSquared = consensusGateSquared;
        this.disagreementInflation = disagreementInflation;
        this.fieldConventionTransform = fieldConventionTransform;
    }

    public AprilTagPoseObservation fromDetections(List<AprilTagDetection> detections) {
        if (detections == null || detections.isEmpty()) return null;
        AprilTagDetection first = null;
        for (AprilTagDetection detection : detections) {
            if (detection != null) { first = detection; break; }
        }
        if (first == null) return null;
        long frameTimestamp = first.frameAcquisitionNanoTime;
        if (frameTimestamp <= lastFrameTimestampNanos) return null;
        List<AprilTagPoseObservation> observations = new ArrayList<>();
        for (AprilTagDetection detection : detections) {
            if (detection == null || detection.frameAcquisitionNanoTime != frameTimestamp
                    || detection.robotPose == null || !stationaryReference.test(detection)) continue;
            Pose3D pose = detection.robotPose;
            Position position = pose.getPosition().toUnit(DistanceUnit.INCH);
            Pose converted = fieldConventionTransform.compose(new Pose(
                    position.x, position.y,
                    pose.getOrientation().getYaw(AngleUnit.RADIANS)));
            observations.add(new AprilTagPoseObservation(
                    frameTimestamp, converted, perDetectionCovariance));
        }
        return fromFrame(frameTimestamp, observations);
    }

    public AprilTagPoseObservation fromFrame(
            long frameTimestampNanos, List<AprilTagPoseObservation> observations) {
        if (frameTimestampNanos <= lastFrameTimestampNanos) return null;
        lastFrameTimestampNanos = frameTimestampNanos;
        return combineFrame(frameTimestampNanos, observations);
    }

    public AprilTagPoseObservation combineFrame(
            long frameTimestampNanos, List<AprilTagPoseObservation> observations) {
        if (observations == null || observations.isEmpty()) return null;
        Pose consensusReference = selectConsensusReference(observations);
        List<Gaussian> accepted = new ArrayList<>();
        for (AprilTagPoseObservation observation : observations) {
            Gaussian gaussian = observation.gaussianNear(consensusReference);
            SimpleMatrix residual = MatrixOps.column(gaussian.mean()).minus(
                    MatrixOps.column(new double[] {
                            consensusReference.x(), consensusReference.y(),
                            consensusReference.heading()
                    }));
            SimpleMatrix innovation = gaussian.covariance().plus(sharedCameraCovarianceFloor);
            double distance = residual.transpose().mult(
                    MatrixOps.solve(innovation, residual)).get(0);
            if (distance <= consensusGateSquared) accepted.add(gaussian);
        }
        if (accepted.isEmpty()) return null;
        Gaussian combined = GaussianFusion.informationAverage(accepted);
        SimpleMatrix disagreement = GaussianFusion.disagreementCovariance(
                accepted, combined.mean());
        SimpleMatrix covariance = combined.covariance().plus(sharedCameraCovarianceFloor)
                .plus(disagreement.scale(disagreementInflation));
        double[] mean = combined.mean();
        return new AprilTagPoseObservation(frameTimestampNanos,
                new Pose(mean[0], mean[1], Angle.normalize(mean[2])),
                MatrixOps.symmetrize(covariance));
    }

    private Pose selectConsensusReference(List<AprilTagPoseObservation> observations) {
        Pose best = observations.get(0).robotFieldPose();
        double bestCost = Double.POSITIVE_INFINITY;
        for (AprilTagPoseObservation candidate : observations) {
            double cost = 0;
            for (AprilTagPoseObservation other : observations) {
                SimpleMatrix residual = MatrixOps.column(new double[] {
                        other.robotFieldPose().x() - candidate.robotFieldPose().x(),
                        other.robotFieldPose().y() - candidate.robotFieldPose().y(),
                        Angle.normalizeSigned(other.robotFieldPose().heading()
                                - candidate.robotFieldPose().heading())
                });
                cost += residual.transpose().mult(MatrixOps.solve(
                        other.covariance().plus(sharedCameraCovarianceFloor), residual)).get(0);
            }
            if (cost < bestCost) { bestCost = cost; best = candidate.robotFieldPose(); }
        }
        return best;
    }

    public long lastFrameTimestampNanos() { return lastFrameTimestampNanos; }
}
