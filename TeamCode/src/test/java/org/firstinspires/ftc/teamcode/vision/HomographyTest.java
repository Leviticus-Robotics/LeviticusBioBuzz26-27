package org.firstinspires.ftc.teamcode.vision;

import org.ejml.simple.SimpleMatrix;
import org.junit.jupiter.api.Test;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertThrows;

class HomographyTest {
    private static final double EPSILON = 1e-9;

    private static Camera camera(double x, double y, double z) {
        return new Camera()
                .setIntrinsics(100.0, 200.0, 640.0, 480.0)
                .setCameraToWorldRotation(0.0, 0.0, 0.0)
                .setCameraCenter(x, y, z);
    }

    private static void assertVector(
            SimpleMatrix actual,
            double... expected
    ) {
        assertEquals(expected.length, actual.getNumRows());
        assertEquals(1, actual.getNumCols());

        for (int row = 0; row < expected.length; row++) {
            assertEquals(
                    expected[row],
                    actual.get(row, 0),
                    EPSILON,
                    "Coordinate " + row
            );
        }
    }

    @Test
    void principalPointMapsAlongOpticalAxis() {
        Homography mapping = Homography.cameraToPlane(
                camera(0.0, 0.0, 0.0),
                Plane.horizontal(10.0)
        );

        assertVector(mapping.transform(320.0, 240.0), 0.0, 0.0, 10.0);
    }

    @Test
    void cameraToPlaneAccountsForIntrinsicsAndCameraPosition() {
        Homography mapping = Homography.cameraToPlane(
                camera(3.0, -4.0, 2.0),
                Plane.horizontal(12.0)
        );

        // Normalized ray is (0.2, 0.2, 1).
        // The plane is 10 units ahead of the camera.
        assertVector(mapping.transform(340.0, 280.0), 5.0, -2.0, 12.0);
    }

    @Test
    void cameraToPlaneAccountsForCameraRotation() {
        Camera camera = camera(0.0, 0.0, 0.0)
                .setCameraToWorldRotation(0.0, 0.0, Math.PI / 2.0);

        Homography mapping = Homography.cameraToPlane(
                camera,
                Plane.horizontal(10.0)
        );

        // Rotating (2, 2, 10) about +Z by 90 degrees gives (-2, 2, 10).
        assertVector(mapping.transform(340.0, 280.0), -2.0, 2.0, 10.0);
    }

    @Test
    void cameraToPlaneSupportsTiltedPlanes() {
        // World plane x + z = 10.
        Plane plane = new Plane(1.0, 0.0, 1.0, -10.0);

        Homography mapping = Homography.cameraToPlane(
                camera(0.0, 0.0, 0.0),
                plane
        );

        // Pixel gives ray (1, 0, 1), intersecting the plane at (5, 0, 5).
        assertVector(mapping.transform(420.0, 240.0), 5.0, 0.0, 5.0);
    }

    @Test
    void planeToCameraProjectsKnownWorldPoint() {
        Homography mapping = Homography.planeToCamera(
                camera(1.0, -2.0, 0.0),
                Plane.horizontal(10.0)
        );

        // Camera-frame point is (1, 4, 10).
        assertVector(mapping.transform(2.0, 2.0, 10.0), 330.0, 320.0);
    }

    @Test
    void betweenCamerasMapsKnownPoint() {
        Homography mapping = Homography.betweenCameras(
                camera(0.0, 0.0, 0.0),
                camera(1.0, -2.0, 0.0),
                Plane.horizontal(10.0)
        );

        // Source pixel observes world point (2, 2, 10).
        assertVector(mapping.transform(340.0, 280.0), 330.0, 320.0);
    }

    @Test
    void betweenCamerasAccountsForDestinationRotation() {
        Camera destination = camera(0.0, 0.0, 0.0)
                .setCameraToWorldRotation(0.0, 0.0, Math.PI / 2.0);

        Homography mapping = Homography.betweenCameras(
                camera(0.0, 0.0, 0.0),
                destination,
                Plane.horizontal(10.0)
        );

        // World (2, 2, 10) becomes destination-camera (2, -2, 10).
        assertVector(mapping.transform(340.0, 280.0), 340.0, 200.0);
    }

    @Test
    void cameraToPlaneInverseProjectsBackToOriginalPixel() {
        Homography mapping = Homography.cameraToPlane(
                camera(3.0, -4.0, 2.0),
                Plane.horizontal(12.0)
        );

        SimpleMatrix world = mapping.transform(340.0, 280.0);

        assertVector(
                mapping.inverse().transform(
                        world.get(0, 0),
                        world.get(1, 0),
                        world.get(2, 0)
                ),
                340.0,
                280.0
        );
    }

    @Test
    void cameraToCameraInverseMapsBackToSourcePixel() {
        Homography mapping = Homography.betweenCameras(
                camera(0.0, 0.0, 0.0),
                camera(1.0, -2.0, 0.0),
                Plane.horizontal(10.0)
        );

        assertVector(
                mapping.inverse().transform(330.0, 320.0),
                340.0,
                280.0
        );
    }

    @Test
    void mappingRetainsPoseFromConstructionTime() {
        Camera camera = camera(0.0, 0.0, 0.0);
        Homography mapping = Homography.cameraToPlane(
                camera,
                Plane.horizontal(10.0)
        );

        camera.setCameraCenter(100.0, 200.0, 300.0);
        camera.setCameraToWorldRotation(0.0, 0.0, Math.PI / 2.0);
        camera.setIntrinsics(500.0, 500.0, 0.0, 0.0);

        assertVector(mapping.transform(340.0, 280.0), 2.0, 2.0, 10.0);
    }

    @Test
    void modifyingReturnedMatrixDoesNotChangeMapping() {
        Homography mapping = Homography.cameraToPlane(
                camera(0.0, 0.0, 0.0),
                Plane.horizontal(10.0)
        );

        mapping.getMatrix().zero();

        assertVector(mapping.transform(340.0, 280.0), 2.0, 2.0, 10.0);
    }

    @Test
    void cameraOnPlaneIsRejected() {
        Camera camera = camera(0.0, 0.0, 10.0);
        Plane plane = Plane.horizontal(10.0);

        assertThrows(
                IllegalArgumentException.class,
                () -> Homography.cameraToPlane(camera, plane)
        );
    }

    @Test
    void rayParallelToPlaneHasNoFiniteIntersection() {
        Homography mapping = Homography.cameraToPlane(
                camera(0.0, 0.0, 0.0),
                new Plane(1.0, 0.0, 0.0, -5.0)
        );

        // The center ray points along +Z and never reaches x = 5.
        assertThrows(
                ArithmeticException.class,
                () -> mapping.transform(320.0, 240.0)
        );
    }

    @Test
    void wrongInputCoordinateCountIsRejected() {
        Homography cameraToPlane = Homography.cameraToPlane(
                camera(0.0, 0.0, 0.0),
                Plane.horizontal(10.0)
        );

        Homography planeToCamera = cameraToPlane.inverse();

        assertThrows(
                IllegalArgumentException.class,
                () -> cameraToPlane.transform(1.0, 2.0, 10.0)
        );

        assertThrows(
                IllegalArgumentException.class,
                () -> planeToCamera.transform(320.0, 240.0)
        );
    }
}