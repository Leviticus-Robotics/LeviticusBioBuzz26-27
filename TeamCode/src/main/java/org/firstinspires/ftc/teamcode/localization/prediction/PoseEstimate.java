package org.firstinspires.ftc.teamcode.localization.prediction;

import com.pedropathing.math.Pose;
import com.pedropathing.utils.Angle;
import org.ejml.simple.SimpleMatrix;
import org.firstinspires.ftc.teamcode.fusion.Gaussian;
import org.firstinspires.ftc.teamcode.fusion.GaussianFusion;

/** Pose-specific interpretation and correction of a three-dimensional Gaussian. */
public class PoseEstimate {
    private final Gaussian gaussian;

    public PoseEstimate(Gaussian gaussian) {
        if (gaussian == null || gaussian.dimension() != 3) {
            throw new IllegalArgumentException("pose estimate must be a 3D Gaussian");
        }
        this.gaussian = gaussian;
    }

    public static PoseEstimate at(Pose pose, SimpleMatrix covariance) {
        return new PoseEstimate(new Gaussian(new double[] {
                pose.x(), pose.y(), Angle.normalize(pose.heading())
        }, covariance));
    }

    public Pose pose() {
        return new Pose(
                gaussian.mean(0), gaussian.mean(1),
                Angle.normalize(gaussian.mean(2)));
    }

    public PoseEstimate correct(Gaussian measurement) {
        Pose reference = pose();
        double[] measurementMean = measurement.mean();
        measurementMean[2] = reference.heading() + Angle.normalizeSigned(
                measurementMean[2] - reference.heading());
        Gaussian fused = GaussianFusion.product(
                gaussian, new Gaussian(measurementMean, measurement.covariance()));
        double[] mean = fused.mean();
        mean[2] = Angle.normalize(mean[2]);
        return new PoseEstimate(new Gaussian(mean, fused.covariance()));
    }

    public Gaussian gaussian() { return gaussian; }
}
