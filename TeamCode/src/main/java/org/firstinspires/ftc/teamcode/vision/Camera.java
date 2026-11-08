package org.firstinspires.ftc.teamcode.vision;

import org.ejml.simple.SimpleMatrix;

public class Camera {
    private SimpleMatrix K;
    private SimpleMatrix R;
    private SimpleMatrix C;

    public Camera setIntrinsics(
            double focalLengthX,
            double focalLengthY,
            double imageWidth,
            double imageHeight
    ) {
        K = new SimpleMatrix(new double[][] {
                {focalLengthX, 0.0, imageWidth / 2},
                {0.0, focalLengthY, imageHeight / 2},
                {0.0, 0.0, 1.0}
        });
        return this;
    }

    /**
     * Sets the camera intrinsics from its horizontal and vertical field of view.
     *
     * @param xFovRad horizontal field of view, in radians
     * @param yFovRad vertical field of view, in radians
     * @param imageWidth image width, in pixels
     * @param imageHeight image height, in pixels
     * @return this camera, for chaining
     */
    public Camera setFieldOfView(
            double xFovRad,
            double yFovRad,
            double imageWidth,
            double imageHeight
    ) {
        if (!(xFovRad > 0.0 && xFovRad < Math.PI)
                || !(yFovRad > 0.0 && yFovRad < Math.PI)) {
            throw new IllegalArgumentException(
                    "Field of view must be between 0 and PI radians"
            );
        }

        double focalLengthX =
                imageWidth / (2.0 * Math.tan(xFovRad / 2.0));
        double focalLengthY =
                imageHeight / (2.0 * Math.tan(yFovRad / 2.0));

        return setIntrinsics(
                focalLengthX,
                focalLengthY,
                imageWidth,
                imageHeight
        );
    }

    /**
     * Sets R, which maps camera-frame vectors into world coordinates.
     *
     * <p>The camera's local +X points right in the image, +Y points down
     * in the image, and +Z points forward along the optical axis.
     * At zero angles, the camera axes align with the world axes.
     *
     * <p>Angles are right-handed. The rotation is R = Rz * Ry * Rx.
     * For a column vector, this applies X, then Y, then Z about the
     * fixed world axes, starting from the zero-angle alignment.
     *
     * @param xAngleRad rotation about the fixed +X axis, in radians
     * @param yAngleRad rotation about the fixed +Y axis, in radians
     * @param zAngleRad rotation about the fixed +Z axis, in radians
     * @return this camera, for chaining
     */
    public Camera setCameraToWorldRotation(
            double xAngleRad,
            double yAngleRad,
            double zAngleRad
    ) {
        double cosX = Math.cos(xAngleRad);
        double sinX = Math.sin(xAngleRad);
        double cosY = Math.cos(yAngleRad);
        double sinY = Math.sin(yAngleRad);
        double cosZ = Math.cos(zAngleRad);
        double sinZ = Math.sin(zAngleRad);

        SimpleMatrix rotationX = new SimpleMatrix(new double[][] {
                {1.0, 0.0, 0.0},
                {0.0, cosX, -sinX},
                {0.0, sinX, cosX}
        });

        SimpleMatrix rotationY = new SimpleMatrix(new double[][] {
                {cosY, 0.0, sinY},
                {0.0, 1.0, 0.0},
                {-sinY, 0.0, cosY}
        });

        SimpleMatrix rotationZ = new SimpleMatrix(new double[][] {
                {cosZ, -sinZ, 0.0},
                {sinZ, cosZ, 0.0},
                {0.0, 0.0, 1.0}
        });

        R = rotationZ.mult(rotationY).mult(rotationX);
        return this;
    }

    public Camera setCameraCenter(double worldX, double worldY, double worldZ) {
        C = new SimpleMatrix(new double[] {worldX, worldY, worldZ});
        return this;
    }

    public SimpleMatrix getK() {
        return K;
    }
    public SimpleMatrix getR() {
        return R;
    }
    public SimpleMatrix getC() {
        return C;
    }
}
