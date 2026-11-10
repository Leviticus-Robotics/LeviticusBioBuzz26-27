package org.firstinspires.ftc.teamcode.localization.factor;

import static org.junit.jupiter.api.Assertions.assertTrue;

import java.util.Random;
import org.ejml.simple.SimpleMatrix;
import org.firstinspires.ftc.teamcode.fusion.core.MatrixOps;
import org.firstinspires.ftc.teamcode.fusion.factor.StateValues;
import org.firstinspires.ftc.teamcode.localization.core.Pose2;
import org.firstinspires.ftc.teamcode.localization.core.RobotState;
import org.junit.jupiter.api.Test;

class PoseFactorJacobianTest {
    private static final int RANDOM_CASES = 1_000;
    private static final double TOLERANCE = 2e-7;
    private static final double EPSILON = 1e-6;

    @Test void absoluteAnalyticJacobianMatchesNumericalReference() {
        Random random = new Random(0xA8501L);
        double maximumError = 0;
        for (int i = 0; i < RANDOM_CASES; i++) {
            double[] state = randomState(random);
            AbsolutePoseFactor factor = new AbsolutePoseFactor("absolute", 10, randomPose(random),
                    MatrixOps.diagonal(.1, .2, .03));
            maximumError = Math.max(maximumError,
                    difference(numericalAbsolute(factor, state), factor.jacobian()));
        }
        assertTrue(maximumError < TOLERANCE, "maximum error=" + maximumError);
    }

    @Test void relativeAnalyticJacobiansMatchNumericalReference() {
        Random random = new Random(0xBE7EEEL);
        double maximumError = 0;
        for (int i = 0; i < RANDOM_CASES; i++) {
            double[] from = randomState(random);
            double[] to = randomState(random);
            PoseDeltaFactor factor = new PoseDeltaFactor("relative", 10, 20,
                    randomPose(random), MatrixOps.diagonal(.1, .2, .03));
            SimpleMatrix[] analytic = factor.jacobians(from, to);
            maximumError = Math.max(maximumError,
                    difference(numericalRelative(factor, from, to, true), analytic[0]));
            maximumError = Math.max(maximumError,
                    difference(numericalRelative(factor, from, to, false), analytic[1]));
        }
        assertTrue(maximumError < TOLERANCE, "maximum error=" + maximumError);
    }

    @Test void octoMotionAnalyticJacobiansMatchNumericalReference() {
        Random random = new Random(0x0C70L);
        double maximumError = 0;
        for (int i = 0; i < RANDOM_CASES; i++) {
            double[] from = randomState(random);
            double[] to = randomState(random);
            MotionDeltaFactor factor = new MotionDeltaFactor(
                    "motion", 10, 20, randomPose(random), between(random, -20, 20),
                    between(random, -20, 20), between(random, -5, 5),
                    MatrixOps.diagonal(.1, .2, .03, .4, .5, .06));
            SimpleMatrix[] analytic = factor.jacobians(values(from, to));
            maximumError = Math.max(maximumError,
                    difference(numericalOcto(factor, from, to, true), analytic[0]));
            maximumError = Math.max(maximumError,
                    difference(numericalOcto(factor, from, to, false), analytic[1]));
        }
        assertTrue(maximumError < TOLERANCE, "maximum error=" + maximumError);
    }

    private static SimpleMatrix numericalAbsolute(AbsolutePoseFactor factor, double[] state) {
        SimpleMatrix result = new SimpleMatrix(3, RobotState.DIMENSION);
        for (int column = 0; column < RobotState.DIMENSION; column++) {
            double[] high = state.clone();
            double[] low = state.clone();
            high[column] += EPSILON;
            low[column] -= EPSILON;
            setColumn(result, column, factor.residual(high), factor.residual(low));
        }
        return result;
    }

    private static SimpleMatrix numericalRelative(PoseDeltaFactor factor, double[] from,
                                                   double[] to, boolean perturbFrom) {
        SimpleMatrix result = new SimpleMatrix(3, RobotState.DIMENSION);
        for (int column = 0; column < RobotState.DIMENSION; column++) {
            double[] highFrom = from.clone();
            double[] lowFrom = from.clone();
            double[] highTo = to.clone();
            double[] lowTo = to.clone();
            (perturbFrom ? highFrom : highTo)[column] += EPSILON;
            (perturbFrom ? lowFrom : lowTo)[column] -= EPSILON;
            setColumn(result, column, factor.residual(highFrom, highTo),
                    factor.residual(lowFrom, lowTo));
        }
        return result;
    }

    private static SimpleMatrix numericalOcto(MotionDeltaFactor factor, double[] from,
                                               double[] to, boolean perturbFrom) {
        SimpleMatrix result = new SimpleMatrix(RobotState.DIMENSION, RobotState.DIMENSION);
        for (int column = 0; column < RobotState.DIMENSION; column++) {
            double[] highFrom = from.clone();
            double[] lowFrom = from.clone();
            double[] highTo = to.clone();
            double[] lowTo = to.clone();
            (perturbFrom ? highFrom : highTo)[column] += EPSILON;
            (perturbFrom ? lowFrom : lowTo)[column] -= EPSILON;
            setColumn(result, column, factor.residual(values(highFrom, highTo)),
                    factor.residual(values(lowFrom, lowTo)));
        }
        return result;
    }

    private static StateValues values(double[] from, double[] to) {
        return timestamp -> timestamp == 10 ? from.clone() : to.clone();
    }

    private static void setColumn(SimpleMatrix matrix, int column, double[] high, double[] low) {
        for (int row = 0; row < high.length; row++) {
            double delta = row == 2 ? Pose2.angularResidual(high[row], low[row]) : high[row] - low[row];
            matrix.set(row, column, delta/(2*EPSILON));
        }
    }

    private static double difference(SimpleMatrix expected, SimpleMatrix actual) {
        double maximum = 0;
        for (int row = 0; row < expected.numRows(); row++) {
            for (int column = 0; column < expected.numCols(); column++) {
                maximum = Math.max(maximum, Math.abs(expected.get(row, column) - actual.get(row, column)));
            }
        }
        return maximum;
    }

    private static double[] randomState(Random random) {
        return new double[] {
                between(random, -100, 100), between(random, -100, 100),
                between(random, -Math.PI, Math.PI), between(random, -20, 20),
                between(random, -20, 20), between(random, -5, 5)
        };
    }

    private static Pose2 randomPose(Random random) {
        return new Pose2(between(random, -100, 100), between(random, -100, 100),
                between(random, -Math.PI, Math.PI));
    }

    private static double between(Random random, double minimum, double maximum) {
        return minimum + random.nextDouble()*(maximum - minimum);
    }
}
