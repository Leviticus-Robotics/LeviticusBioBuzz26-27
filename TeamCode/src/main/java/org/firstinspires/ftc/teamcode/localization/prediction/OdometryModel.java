package org.firstinspires.ftc.teamcode.localization.prediction;

import com.pedropathing.math.Pose;
import com.pedropathing.math.Twist;
import com.pedropathing.math.Vector;
import org.ejml.simple.SimpleMatrix;
import org.firstinspires.ftc.teamcode.fusion.Gaussian;
import org.firstinspires.ftc.teamcode.fusion.MatrixOps;

/** Probabilistic transition model for a measured robot-relative odometry transform. */
public class OdometryModel {
    private final SimpleMatrix variancePerUnit;

    public OdometryModel(SimpleMatrix variancePerUnit) {
        this.variancePerUnit = MatrixOps.copy(variancePerUnit);
    }

    public PoseEstimate propagate(PoseEstimate previous, Pose relativeTransform) {
        Pose previousPose = previous.pose();
        Pose nextPose = previousPose.compose(relativeTransform);
        Vector distance = Twist.riemannianLog(Pose.zero(), relativeTransform)
                .toVector().abs();

        SimpleMatrix distanceScale = MatrixOps.diagonal(
                Math.sqrt(distance.get(0)),
                Math.sqrt(distance.get(1)),
                Math.sqrt(distance.get(2)));
        SimpleMatrix bodyNoise = distanceScale.mult(variancePerUnit)
                .mult(distanceScale);

        double cosine = Math.cos(previousPose.heading());
        double sine = Math.sin(previousPose.heading());
        double dx = relativeTransform.x();
        double dy = relativeTransform.y();
        SimpleMatrix stateJacobian = new SimpleMatrix(new double[][] {
                {1, 0, -sine*dx - cosine*dy},
                {0, 1, cosine*dx - sine*dy},
                {0, 0, 1}
        });
        SimpleMatrix noiseJacobian = new SimpleMatrix(new double[][] {
                {cosine, -sine, 0},
                {sine, cosine, 0},
                {0, 0, 1}
        });

        Gaussian previousGaussian = previous.gaussian();
        SimpleMatrix covariance = stateJacobian.mult(previousGaussian.covariance())
                .mult(stateJacobian.transpose())
                .plus(noiseJacobian.mult(bodyNoise).mult(noiseJacobian.transpose()));
        return PoseEstimate.at(nextPose, MatrixOps.symmetrize(covariance));
    }
}
