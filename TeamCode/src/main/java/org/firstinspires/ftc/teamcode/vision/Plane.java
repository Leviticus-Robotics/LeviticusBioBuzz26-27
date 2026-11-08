package org.firstinspires.ftc.teamcode.vision;

import org.ejml.simple.SimpleMatrix;

public final class Plane {
    private final SimpleMatrix normal;
    private final double offset;

    public Plane(double normalX, double normalY, double normalZ, double offset) {
        SimpleMatrix normal = new SimpleMatrix(
                new double[] {normalX, normalY, normalZ}
        );

        double length = normal.normF();

        if (length == 0.0 || !Double.isFinite(length)
                || !Double.isFinite(offset)) {
            throw new IllegalArgumentException(
                    "Plane requires a finite, nonzero normal and finite offset"
            );
        }

        this.normal = normal.divide(length);
        this.offset = offset / length;
    }

    /**
     * Creates the world plane z = worldZ.
     */
    public static Plane horizontal(double worldZ) {
        return new Plane(0.0, 0.0, 1.0, -worldZ);
    }

    public SimpleMatrix getNormal() {
        return new SimpleMatrix(normal);
    }

    public double getOffset() {
        return offset;
    }
}