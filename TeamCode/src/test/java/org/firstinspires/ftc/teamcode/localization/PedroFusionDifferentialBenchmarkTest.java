package org.firstinspires.ftc.teamcode.localization;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;

import com.pedropathing.localization.FusionLocalizer;
import com.pedropathing.localization.Localizer;
import com.pedropathing.localization.MotionState;
import com.pedropathing.math.Pose;
import com.pedropathing.math.Velocity;
import java.lang.reflect.Field;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.Collections;
import java.util.List;
import java.util.Map;
import java.util.NavigableMap;
import java.util.concurrent.atomic.AtomicLong;
import org.firstinspires.ftc.teamcode.fusion.Gaussian;
import org.firstinspires.ftc.teamcode.fusion.MatrixOps;
import org.firstinspires.ftc.teamcode.localization.pedro.FusedLocalizer;
import org.junit.jupiter.api.Test;

/** Differential simulation and microbenchmark against Pedro's FusionLocalizer. */
class PedroFusionDifferentialBenchmarkTest {
    private static final long LOOP_NANOS = 20_000_000L;
    private static final double LOOP_SECONDS = LOOP_NANOS*1e-9;
    private static final Pose INITIAL_VARIANCE = new Pose(4, 4, .25);
    private static final Pose ODOMETRY_VARIANCE = new Pose(.0025, .0025, .0001);
    private static final Pose CAMERA_VARIANCE = new Pose(.25, .25, .01);
    @Test void compareIdenticalSimulationAndBenchmarkSpeed() throws Exception {
        runScenario(2_000, 25, 7);
        Result odometryOnly = runTrials("odometry-only", 5, 8_000, 0, 0);
        Result corrected = runTrials("delayed-camera", 5, 2_000, 40, 7);

        assertEquals(0, odometryOnly.maximumEstimatorTranslationDifference, 1e-9);
        assertEquals(0, odometryOnly.maximumEstimatorHeadingDifference, 1e-9);
        assertTrue(Double.isFinite(corrected.maximumEstimatorTranslationDifference));
        assertTrue(Double.isFinite(corrected.maximumEstimatorHeadingDifference));
    }

    private static Result runTrials(String name, int trials, int steps,
                                    int cameraPeriod, int cameraDelayTicks) throws Exception {
        Result[] results = new Result[trials];
        for (int trial = 0; trial < trials; trial++) {
            results[trial] = runScenario(steps, cameraPeriod, cameraDelayTicks);
        }
        Result result = new Result(
                median(results, value -> value.maximumEstimatorTranslationDifference),
                median(results, value -> value.maximumEstimatorHeadingDifference),
                median(results, value -> value.pedroFinalTruthError),
                median(results, value -> value.ourFinalTruthError),
                median(results, value -> value.pedroUpdateNanos),
                median(results, value -> value.ourUpdateNanos),
                median(results, value -> value.pedroCorrectionNanos),
                median(results, value -> value.ourCorrectionNanos),
                results[0].corrections);
        printResult(name, result);
        return result;
    }

    private static Result runScenario(int steps, int cameraPeriod,
                                      int cameraDelayTicks) throws Exception {
        FakeLocalizer oursRaw = new FakeLocalizer();
        PedroReference pedro = new PedroReference(steps + 2);
        AtomicLong ourClock = new AtomicLong();
        FusedLocalizer ours = new FusedLocalizer(
                oursRaw,
                new FusedLocalizer.Config(
                        MatrixOps.diagonal(4, 4, .25),
                        MatrixOps.diagonal(.0025, .0025, .0001),
                        Long.MAX_VALUE),
                ourClock::get);

        List<Pose> truths = new ArrayList<>();
        List<Long> timestamps = new ArrayList<>();
        Pose truth = Pose.zero();
        Pose raw = Pose.zero();
        truths.add(truth);
        timestamps.add(0L);
        long pedroUpdateNanos = 0;
        long ourUpdateNanos = 0;
        long pedroCorrectionNanos = 0;
        long ourCorrectionNanos = 0;
        int corrections = 0;
        double maximumTranslationDifference = 0;
        double maximumHeadingDifference = 0;

        for (int step = 1; step <= steps; step++) {
            double forward = 30 + 8*Math.sin(step*.031);
            double strafe = 5*Math.cos(step*.047);
            double omega = .45*Math.sin(step*.019);
            truth = integrate(truth, forward, strafe, omega, LOOP_SECONDS);
            raw = integrate(raw, forward*1.004, strafe*.997, omega*1.002, LOOP_SECONDS);
            Velocity rawVelocity = fieldVelocity(raw.heading(),
                    forward*1.004, strafe*.997, omega*1.002);
            MotionState rawState = MotionState.ofVelocity(raw, rawVelocity);
            pedro.setState(rawState);
            oursRaw.state = rawState;
            truths.add(truth);

            pedro.prepareUpdate();
            long started = System.nanoTime();
            pedro.update();
            pedroUpdateNanos += System.nanoTime() - started;
            long timestamp = pedro.latestTimestamp();
            timestamps.add(timestamp);

            ourClock.set(timestamp);
            started = System.nanoTime();
            ours.update();
            ourUpdateNanos += System.nanoTime() - started;

            if (cameraPeriod > 0 && step >= cameraDelayTicks
                    && step % cameraPeriod == 0) {
                int capture = step - cameraDelayTicks;
                Pose capturedTruth = truths.get(capture);
                Pose camera = new Pose(
                        capturedTruth.x() + .12*Math.sin(step*.13),
                        capturedTruth.y() + .12*Math.cos(step*.17),
                        capturedTruth.heading() + .008*Math.sin(step*.11));
                long captureTimestamp = timestamps.get(capture);
                Gaussian cameraGaussian = new Gaussian(new double[] {
                        camera.x(), camera.y(), camera.heading()
                }, MatrixOps.diagonal(.25, .25, .01));

                started = System.nanoTime();
                pedro.addMeasurement(camera, captureTimestamp);
                pedroCorrectionNanos += System.nanoTime() - started;
                started = System.nanoTime();
                ours.addAbsoluteMeasurement("camera", captureTimestamp,
                        cameraGaussian);
                ourCorrectionNanos += System.nanoTime() - started;
                corrections++;
            }

            Pose pedroPose = pedro.state().pose();
            Pose ourPose = ours.state().pose();
            maximumTranslationDifference = Math.max(maximumTranslationDifference,
                    Math.hypot(pedroPose.x() - ourPose.x(), pedroPose.y() - ourPose.y()));
            maximumHeadingDifference = Math.max(maximumHeadingDifference,
                    Math.abs(normalizeSigned(pedroPose.heading() - ourPose.heading())));
        }

        Pose pedroPose = pedro.state().pose();
        Pose ourPose = ours.state().pose();
        Result result = new Result(
                maximumTranslationDifference,
                maximumHeadingDifference,
                poseError(pedroPose, truth),
                poseError(ourPose, truth),
                pedroUpdateNanos/(double)steps,
                ourUpdateNanos/(double)steps,
                corrections == 0 ? 0 : pedroCorrectionNanos/(double)corrections,
                corrections == 0 ? 0 : ourCorrectionNanos/(double)corrections,
                corrections);
        return result;
    }

    private static void printResult(String name, Result result) {
        System.out.printf(
                "%s: max difference %.9f in / %.9f rad; final truth error "
                        + "Pedro %.6f, ours %.6f; update ns Pedro %.1f, ours %.1f; "
                        + "correction ns Pedro %.1f, ours %.1f (%d corrections)%n",
                name,
                result.maximumEstimatorTranslationDifference,
                result.maximumEstimatorHeadingDifference,
                result.pedroFinalTruthError,
                result.ourFinalTruthError,
                result.pedroUpdateNanos,
                result.ourUpdateNanos,
                result.pedroCorrectionNanos,
                result.ourCorrectionNanos,
                result.corrections);
    }

    private static double median(Result[] results, ResultValue value) {
        double[] values = new double[results.length];
        for (int index = 0; index < results.length; index++) {
            values[index] = value.get(results[index]);
        }
        Arrays.sort(values);
        return values[values.length/2];
    }

    private static Pose integrate(Pose pose, double forward, double strafe,
                                  double omega, double dt) {
        double cosine = Math.cos(pose.heading());
        double sine = Math.sin(pose.heading());
        return new Pose(
                pose.x() + (cosine*forward - sine*strafe)*dt,
                pose.y() + (sine*forward + cosine*strafe)*dt,
                pose.heading() + omega*dt);
    }

    private static Velocity fieldVelocity(double heading, double forward,
                                          double strafe, double omega) {
        double cosine = Math.cos(heading);
        double sine = Math.sin(heading);
        return new Velocity(
                cosine*forward - sine*strafe,
                sine*forward + cosine*strafe,
                omega);
    }

    private static double poseError(Pose estimate, Pose truth) {
        return Math.hypot(estimate.x() - truth.x(), estimate.y() - truth.y())
                + Math.abs(normalizeSigned(estimate.heading() - truth.heading()));
    }

    private static double normalizeSigned(double angle) {
        return Math.atan2(Math.sin(angle), Math.cos(angle));
    }

    /** Isolates the reflection needed because Pedro does not expose its tick clock. */
    private static final class PedroReference {
        private static final Field HISTORY = field("history");
        private static final Field LAST_UPDATE = field("lastUpdateTime");
        private final FakeLocalizer raw = new FakeLocalizer();
        private final FusionLocalizer localizer;

        PedroReference(int bufferSize) {
            localizer = new FusionLocalizer(
                    raw, INITIAL_VARIANCE, ODOMETRY_VARIANCE,
                    CAMERA_VARIANCE, bufferSize);
        }

        void setState(MotionState state) { raw.state = state; }

        void prepareUpdate() throws IllegalAccessException {
            LAST_UPDATE.setLong(localizer, System.nanoTime() - LOOP_NANOS);
        }

        void update() { localizer.update(); }

        void addMeasurement(Pose pose, long timestampNanos) {
            localizer.addMeasurement(pose, timestampNanos, CAMERA_VARIANCE);
        }

        MotionState state() { return localizer.state(); }

        @SuppressWarnings("unchecked")
        long latestTimestamp() throws IllegalAccessException {
            NavigableMap<Long, ?> history =
                    (NavigableMap<Long, ?>) HISTORY.get(localizer);
            return history.lastKey();
        }

        private static Field field(String name) {
            try {
                Field field = FusionLocalizer.class.getDeclaredField(name);
                field.setAccessible(true);
                return field;
            } catch (ReflectiveOperationException exception) {
                throw new ExceptionInInitializerError(exception);
            }
        }
    }

    private static final class FakeLocalizer implements Localizer {
        MotionState state = MotionState.zero();
        @Override public MotionState state() { return state; }
        @Override public void update() { }
        @Override public void setPose(Pose pose) {
            state = MotionState.ofVelocity(pose, state.velocity());
        }
        @Override public void reset() { state = MotionState.zero(); }
        @Override public Map<String, Object> debug() { return Collections.emptyMap(); }
    }

    private static final class Result {
        final double maximumEstimatorTranslationDifference;
        final double maximumEstimatorHeadingDifference;
        final double pedroFinalTruthError;
        final double ourFinalTruthError;
        final double pedroUpdateNanos;
        final double ourUpdateNanos;
        final double pedroCorrectionNanos;
        final double ourCorrectionNanos;
        final int corrections;

        Result(double maximumEstimatorTranslationDifference,
               double maximumEstimatorHeadingDifference,
               double pedroFinalTruthError, double ourFinalTruthError,
               double pedroUpdateNanos, double ourUpdateNanos,
               double pedroCorrectionNanos, double ourCorrectionNanos,
               int corrections) {
            this.maximumEstimatorTranslationDifference = maximumEstimatorTranslationDifference;
            this.maximumEstimatorHeadingDifference = maximumEstimatorHeadingDifference;
            this.pedroFinalTruthError = pedroFinalTruthError;
            this.ourFinalTruthError = ourFinalTruthError;
            this.pedroUpdateNanos = pedroUpdateNanos;
            this.ourUpdateNanos = ourUpdateNanos;
            this.pedroCorrectionNanos = pedroCorrectionNanos;
            this.ourCorrectionNanos = ourCorrectionNanos;
            this.corrections = corrections;
        }
    }

    private interface ResultValue {
        double get(Result result);
    }
}
