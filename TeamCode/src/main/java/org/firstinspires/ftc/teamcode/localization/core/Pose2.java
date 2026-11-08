package org.firstinspires.ftc.teamcode.localization.core;

import java.util.Objects;

/** Immutable planar pose owned by localization, independent of sensors and consumers. */
public final class Pose2 {
    private final double x;
    private final double y;
    private final double heading;

    public Pose2(double x, double y, double heading) {
        if (!Double.isFinite(x) || !Double.isFinite(y) || !Double.isFinite(heading)) {
            throw new IllegalArgumentException("pose components must be finite");
        }
        this.x = x;
        this.y = y;
        this.heading = wrap(heading);
    }

    public static Pose2 zero() { return new Pose2(0, 0, 0); }
    public double x() { return x; }
    public double y() { return y; }
    public double heading() { return heading; }

    public Pose2 compose(Pose2 localTransform) {
        double cosine = Math.cos(heading);
        double sine = Math.sin(heading);
        return new Pose2(
                x + cosine * localTransform.x - sine * localTransform.y,
                y + sine * localTransform.x + cosine * localTransform.y,
                heading + localTransform.heading);
    }

    public Pose2 inverse() {
        double cosine = Math.cos(heading);
        double sine = Math.sin(heading);
        return new Pose2(-cosine * x - sine * y, sine * x - cosine * y, -heading);
    }

    public Pose2 localDeltaTo(Pose2 next) { return inverse().compose(next); }

    public Pose2 correctedBy(double dx, double dy, double dHeading) {
        return new Pose2(x + dx, y + dy, heading + dHeading);
    }

    public double[] toArray() { return new double[] {x, y, heading}; }

    public static double wrap(double angle) {
        double result = angle % (2.0 * Math.PI);
        if (result >= Math.PI) result -= 2.0 * Math.PI;
        if (result < -Math.PI) result += 2.0 * Math.PI;
        return result;
    }

    public static double angularResidual(double measured, double predicted) { return wrap(measured - predicted); }

    @Override public boolean equals(Object other) {
        if (this == other) return true;
        if (!(other instanceof Pose2)) return false;
        Pose2 pose = (Pose2) other;
        return Double.compare(x, pose.x) == 0 && Double.compare(y, pose.y) == 0
                && Double.compare(heading, pose.heading) == 0;
    }

    @Override public int hashCode() { return Objects.hash(x, y, heading); }
    @Override public String toString() { return "Pose2(" + x + ", " + y + ", " + heading + ")"; }
}
