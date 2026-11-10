package org.firstinspires.ftc.teamcode.localization.core;

import org.ejml.simple.SimpleMatrix;
import org.firstinspires.ftc.teamcode.fusion.core.Gaussian;

/** Six-dimensional [x,y,heading,vx,vy,omega] Gaussian state. */
public final class RobotState {
    public static final int DIMENSION = 6;
    public static final int X = 0, Y = 1, HEADING = 2, VX = 3, VY = 4, OMEGA = 5;
    private final Gaussian gaussian;

    public RobotState(Gaussian gaussian) {
        if (gaussian.dimension() != DIMENSION) throw new IllegalArgumentException("robot state must have 6 dimensions");
        double wrappedHeading = Pose2.wrap(gaussian.mean(HEADING));
        if (wrappedHeading == gaussian.mean(HEADING)) {
            this.gaussian = gaussian;
        } else {
            double[] mean = gaussian.mean();
            mean[HEADING] = wrappedHeading;
            this.gaussian = new Gaussian(mean, gaussian.covariance());
        }
    }

    public static RobotState create(Pose2 pose, double vx, double vy, double omega, SimpleMatrix covariance) {
        return new RobotState(new Gaussian(new double[] {
                pose.x(), pose.y(), pose.heading(), vx, vy, omega
        }, covariance));
    }

    /** Canonicalizes coordinates that have equivalent representations. */
    public static double[] normalize(double[] state) {
        if (state == null || state.length != DIMENSION) {
            throw new IllegalArgumentException("robot state must have 6 dimensions");
        }
        double[] result = state.clone();
        result[HEADING] = Pose2.wrap(result[HEADING]);
        return result;
    }

    public Gaussian gaussian() { return gaussian; }
    public Pose2 pose() { return new Pose2(gaussian.mean(X), gaussian.mean(Y), gaussian.mean(HEADING)); }
    public double vx() { return gaussian.mean(VX); }
    public double vy() { return gaussian.mean(VY); }
    public double omega() { return gaussian.mean(OMEGA); }
    public SimpleMatrix covariance() { return gaussian.covariance(); }
}
