package org.firstinspires.ftc.teamcode.localization;

import static org.junit.jupiter.api.Assertions.*;

import org.ejml.simple.SimpleMatrix;
import org.firstinspires.ftc.teamcode.fusion.core.Gaussian;
import org.firstinspires.ftc.teamcode.fusion.core.MatrixOps;
import org.firstinspires.ftc.teamcode.localization.core.Pose2;
import org.firstinspires.ftc.teamcode.localization.core.RobotState;
import org.junit.jupiter.api.Test;

class LocalizationMathTest {
    @Test void solvesRepresentativeSixBySixSystem() {
        SimpleMatrix matrix = MatrixOps.diagonal(2, 3, 4, 5, 6, 7);
        SimpleMatrix solution = MatrixOps.solve(matrix, MatrixOps.column(new double[] {2, 6, 12, 20, 30, 42}));
        for (int i = 0; i < 6; i++) assertEquals(i + 1, solution.get(i), 1e-12);
    }

    @Test void gaussianInformationRoundTripPreservesDistribution() {
        Gaussian original = new Gaussian(new double[] {1, -2, .3}, MatrixOps.diagonal(.2, .3, .4));
        Gaussian roundTrip = original.toInformation().toGaussian();
        assertArrayEquals(original.mean(), roundTrip.mean(), 1e-10);
        assertTrue(original.covariance().isIdentical(roundTrip.covariance(), 1e-10));
    }

    @Test void invalidCovarianceIsRejected() {
        assertThrows(IllegalArgumentException.class,
                () -> new Gaussian(new double[] {0, 0}, new SimpleMatrix(new double[][] {{1, 2}, {0, 1}})));
        assertThrows(IllegalArgumentException.class,
                () -> new Gaussian(new double[] {0}, MatrixOps.diagonal(Double.NaN)));
        assertThrows(IllegalArgumentException.class,
                () -> new Gaussian(new double[] {0}, MatrixOps.diagonal(0)));
    }

    @Test void poseRoundTripAndHeadingWrapWork() {
        Pose2 start = new Pose2(2, -1, Math.PI - .01);
        Pose2 delta = new Pose2(.5, -.2, .04);
        Pose2 end = start.compose(delta);
        Pose2 recovered = start.localDeltaTo(end);
        assertEquals(delta.x(), recovered.x(), 1e-10);
        assertEquals(delta.y(), recovered.y(), 1e-10);
        assertEquals(delta.heading(), recovered.heading(), 1e-10);
        assertEquals(.02, Pose2.angularResidual(-Math.PI + .01, Math.PI - .01), 1e-10);
    }

    @Test void statePreservesPoseAndVelocity() {
        RobotState state = RobotState.create(new Pose2(1, 2, 3), 4, 5, 6,
                MatrixOps.diagonal(1, 1, 1, 1, 1, 1));
        assertEquals(1, state.pose().x());
        assertEquals(4, state.vx());
        assertEquals(6, state.omega());
        assertThrows(IllegalArgumentException.class,
                () -> new RobotState(new Gaussian(new double[] {1, 2, 3}, MatrixOps.diagonal(1, 1, 1))));
    }
}
