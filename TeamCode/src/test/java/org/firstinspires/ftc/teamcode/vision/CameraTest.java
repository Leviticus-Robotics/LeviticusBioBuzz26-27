package org.firstinspires.ftc.teamcode.vision;

import org.ejml.simple.SimpleMatrix;
import org.junit.jupiter.api.Test;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertThrows;

class CameraTest {
    private static final double EPSILON = 1e-9;

    @Test
    void fieldOfViewIsConvertedToPixelFocalLengths() {
        Camera camera = new Camera().setFieldOfView(
                Math.PI / 2.0,
                Math.toRadians(60.0),
                640.0,
                480.0
        );

        SimpleMatrix intrinsics = camera.getK();

        assertEquals(320.0, intrinsics.get(0, 0), EPSILON);
        assertEquals(240.0 / Math.tan(Math.toRadians(30.0)),
                intrinsics.get(1, 1), EPSILON);
        assertEquals(320.0, intrinsics.get(0, 2), EPSILON);
        assertEquals(240.0, intrinsics.get(1, 2), EPSILON);
        assertEquals(1.0, intrinsics.get(2, 2), EPSILON);
    }

    @Test
    void invalidFieldOfViewIsRejected() {
        assertThrows(
                IllegalArgumentException.class,
                () -> new Camera().setFieldOfView(0.0, 1.0, 640.0, 480.0)
        );
        assertThrows(
                IllegalArgumentException.class,
                () -> new Camera().setFieldOfView(1.0, Math.PI, 640.0, 480.0)
        );
        assertThrows(
                IllegalArgumentException.class,
                () -> new Camera().setFieldOfView(
                        Double.NaN,
                        1.0,
                        640.0,
                        480.0
                )
        );
    }
}
