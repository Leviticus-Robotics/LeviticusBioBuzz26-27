package org.firstinspires.ftc.teamcode.vision;

import org.ejml.simple.SimpleMatrix;

public final class Homography {
    private final SimpleMatrix matrix;
    private final SimpleMatrix reverseMatrix;

    private Homography(
            SimpleMatrix matrix,
            SimpleMatrix reverseMatrix
    ) {
        this.matrix = new SimpleMatrix(matrix);
        this.reverseMatrix = new SimpleMatrix(reverseMatrix);
    }

    /**
     * Maps undistorted camera pixels to world XYZ points on the plane.
     * <br>
     * Camera R must map camera coordinates into world coordinates.
     */
    public static Homography cameraToPlane(Camera camera, Plane plane) {
        SimpleMatrix center = camera.getC();
        SimpleMatrix normal = plane.getNormal();

        double planeOffsetAtCamera =
                normal.dot(center) + plane.getOffset();

        if (planeOffsetAtCamera == 0.0) {
            throw new IllegalArgumentException(
                    "Camera center lies on the plane"
            );
        }

        SimpleMatrix pixelToRay =
                camera.getR().mult(camera.getK().invert());

        // Homogeneous world point:
        // [C * n^T - (n^T * C + d) * I] * ray
        // [                  n^T     ] * ray
        SimpleMatrix rayToWorld = new SimpleMatrix(4, 3);

        SimpleMatrix numerator = center.mult(normal.transpose())
                .minus(SimpleMatrix.identity(3).scale(planeOffsetAtCamera));

        for (int column = 0; column < 3; column++) {
            for (int row = 0; row < 3; row++) {
                rayToWorld.set(row, column, numerator.get(row, column));
            }

            rayToWorld.set(3, column, normal.get(column, 0));
        }

        SimpleMatrix cameraToWorld = rayToWorld.mult(pixelToRay);

        // Projection: K * R^T * [I | -C].
        SimpleMatrix worldToCamera = camera.getR().transpose();
        SimpleMatrix translation = worldToCamera.mult(center).negative();
        SimpleMatrix extrinsic = new SimpleMatrix(3, 4);

        for (int row = 0; row < 3; row++) {
            for (int column = 0; column < 3; column++) {
                extrinsic.set(row, column, worldToCamera.get(row, column));
            }

            extrinsic.set(row, 3, translation.get(row, 0));
        }

        SimpleMatrix worldToPixel = camera.getK().mult(extrinsic);

        return new Homography(cameraToWorld, worldToPixel);
    }

    /**
     * Maps world XYZ points on the plane to undistorted camera pixels.
     */
    public static Homography planeToCamera(Camera camera, Plane plane) {
        return cameraToPlane(camera, plane).inverse();
    }

    /**
     * Maps source pixels to destination pixels for points on the plane.
     */
    public static Homography betweenCameras(
            Camera source,
            Camera destination,
            Plane plane
    ) {
        Homography sourceMapping = cameraToPlane(source, plane);
        Homography destinationMapping = cameraToPlane(destination, plane);

        SimpleMatrix forward = destinationMapping.reverseMatrix
                .mult(sourceMapping.matrix);

        SimpleMatrix reverse = sourceMapping.reverseMatrix
                .mult(destinationMapping.matrix);

        return new Homography(forward, reverse);
    }

    /**
     * Transforms a pixel coordinate.
     * Returns a 2x1 destination pixel for camera-to-camera mappings,
     * or a 3x1 world point for camera-to-plane mappings.
     */
    public SimpleMatrix transform(double pixelX, double pixelY) {
        return transformHomogeneous(
                new SimpleMatrix(new double[] {pixelX, pixelY, 1.0})
        );
    }

    /**
     * Transforms a world XYZ point on the mapping's plane.
     * Returns a 2x1 pixel coordinate.
     */
    public SimpleMatrix transform(double worldX, double worldY, double worldZ) {
        return transformHomogeneous(
                new SimpleMatrix(new double[] {worldX, worldY, worldZ, 1.0})
        );
    }

    /**
     * Returns the reverse mapping.
     *
     * For world inputs, the inverse relationship applies to points
     * on the plane used to construct this mapping.
     */
    public Homography inverse() {
        return new Homography(reverseMatrix, matrix);
    }

    public SimpleMatrix getMatrix() {
        return new SimpleMatrix(matrix);
    }

    private SimpleMatrix transformHomogeneous(SimpleMatrix input) {
        if (input.getNumRows() != matrix.getNumCols()) {
            throw new IllegalArgumentException(
                    "Input coordinate count does not match this mapping"
            );
        }

        SimpleMatrix output = matrix.mult(input);
        int coordinateCount = output.getNumRows() - 1;
        double w = output.get(coordinateCount, 0);

        if (w == 0.0 || output.hasUncountable()) {
            throw new ArithmeticException("Point has no finite mapping");
        }

        SimpleMatrix result = new SimpleMatrix(coordinateCount, 1);

        for (int row = 0; row < coordinateCount; row++) {
            result.set(row, 0, output.get(row, 0) / w);
        }

        if (result.hasUncountable()) {
            throw new ArithmeticException("Point has no finite mapping");
        }

        return result;
    }
}