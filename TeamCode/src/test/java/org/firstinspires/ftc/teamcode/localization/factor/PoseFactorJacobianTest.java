package org.firstinspires.ftc.teamcode.localization.factor;

import static org.junit.jupiter.api.Assertions.assertTrue;

import java.util.LinkedHashMap;
import java.util.Map;
import java.util.Random;
import org.ejml.simple.SimpleMatrix;
import org.firstinspires.ftc.teamcode.fusion.core.MatrixOps;
import org.firstinspires.ftc.teamcode.fusion.factor.Factor;
import org.firstinspires.ftc.teamcode.fusion.factor.LinearizedFactor;
import org.firstinspires.ftc.teamcode.fusion.factor.StateValues;
import org.firstinspires.ftc.teamcode.localization.core.Pose2;
import org.firstinspires.ftc.teamcode.localization.core.RobotState;
import org.junit.jupiter.api.Test;

class PoseFactorJacobianTest {
    private static final int RANDOM_CASES = 1_000;
    private static final double TOLERANCE = 2e-7;

    @Test void absoluteAnalyticJacobianMatchesNumericalReference() {
        Random random = new Random(0xA8501L);
        double maximumError = 0;
        for (int i = 0; i < RANDOM_CASES; i++) {
            long timestamp = 10;
            double[] state = randomState(random);
            Pose2 measurement = randomPose(random);
            if (i < 4) {
                state[RobotState.HEADING] = i % 2 == 0 ? Math.PI - 1e-8 : -Math.PI + 1e-8;
                measurement = new Pose2(measurement.x(), measurement.y(), -state[RobotState.HEADING]);
            }
            AbsolutePoseFactor factor = new AbsolutePoseFactor("absolute", timestamp, measurement,
                    MatrixOps.diagonal(.1, .2, .03));
            StateValues values = values(timestamp, state);
            maximumError = Math.max(maximumError,
                    maximumDifference(numericalLinearization(factor, values).jacobians,
                            factor.analyticJacobians(values)));
        }
        System.out.println("absolute pose maximum Jacobian error=" + maximumError);
        assertTrue(maximumError < TOLERANCE, "maximum error=" + maximumError);
    }

    @Test void relativeAnalyticJacobiansMatchNumericalReference() {
        Random random = new Random(0xBE7EEEL);
        double maximumError = 0;
        for (int i = 0; i < RANDOM_CASES; i++) {
            long fromTimestamp = 10;
            long toTimestamp = 20;
            double[] from = randomState(random);
            double[] to = randomState(random);
            Pose2 measurement = randomPose(random);
            if (i < 8) {
                from[RobotState.HEADING] = i % 2 == 0 ? Math.PI - 1e-8 : -Math.PI + 1e-8;
                to[RobotState.HEADING] = -from[RobotState.HEADING] + (i - 4)*1e-9;
                measurement = new Pose2(measurement.x(), measurement.y(),
                        i % 2 == 0 ? Math.PI - 1e-8 : -Math.PI + 1e-8);
            }
            RelativePoseFactor factor = new RelativePoseFactor(
                    "relative", fromTimestamp, toTimestamp, measurement,
                    MatrixOps.diagonal(.1, .2, .03));
            StateValues values = values(fromTimestamp, from, toTimestamp, to);
            LinearizedFactor numerical = numericalLinearization(factor, values);
            maximumError = Math.max(maximumError,
                    maximumDifference(numerical.jacobians, factor.analyticJacobians(values)));
        }
        System.out.println("relative pose maximum Jacobian error=" + maximumError);
        assertTrue(maximumError < TOLERANCE, "maximum error=" + maximumError);
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

    private static double maximumDifference(SimpleMatrix[] expected, SimpleMatrix[] actual) {
        double maximum = 0;
        for (int matrix = 0; matrix < expected.length; matrix++) {
            for (int row = 0; row < expected[matrix].numRows(); row++) {
                for (int column = 0; column < expected[matrix].numCols(); column++) {
                    maximum = Math.max(maximum, Math.abs(
                            expected[matrix].get(row, column) - actual[matrix].get(row, column)));
                }
            }
        }
        return maximum;
    }

    private static LinearizedFactor numericalLinearization(Factor factor, StateValues values) {
        Factor numericalReference = new Factor() {
            @Override public String source() { return factor.source(); }
            @Override public long[] timestamps() { return factor.timestamps(); }
            @Override public double[] residual(StateValues states) { return factor.residual(states); }
            @Override public SimpleMatrix covariance(StateValues states) {
                return factor.covariance(states);
            }
            @Override public double residualDifference(int row, double high, double low) {
                return factor.residualDifference(row, high, low);
            }
        };
        return numericalReference.linearize(values);
    }

    private static StateValues values(long timestamp, double[] state) {
        return values(timestamp, state, Long.MIN_VALUE, null);
    }

    private static StateValues values(long firstTimestamp, double[] firstState,
                                      long secondTimestamp, double[] secondState) {
        Map<Long, double[]> states = new LinkedHashMap<>();
        states.put(firstTimestamp, firstState.clone());
        if (secondState != null) states.put(secondTimestamp, secondState.clone());
        return timestamp -> {
            double[] state = states.get(timestamp);
            if (state == null) throw new IllegalArgumentException("unknown timestamp");
            return state.clone();
        };
    }
}
