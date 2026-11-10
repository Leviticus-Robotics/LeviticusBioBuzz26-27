package org.firstinspires.ftc.teamcode.localization;

import java.util.ArrayList;
import java.util.Collections;
import java.util.List;
import org.firstinspires.ftc.teamcode.fusion.core.MatrixOps;
import org.firstinspires.ftc.teamcode.fusion.engine.FusionEngine;
import org.firstinspires.ftc.teamcode.fusion.engine.ProcessModel;
import org.firstinspires.ftc.teamcode.fusion.factor.Factor;
import org.firstinspires.ftc.teamcode.localization.core.ControlInput;
import org.firstinspires.ftc.teamcode.localization.core.Pose2;
import org.firstinspires.ftc.teamcode.localization.core.RobotState;
import org.firstinspires.ftc.teamcode.localization.engine.PoseVelocityProcessModel;
import org.firstinspires.ftc.teamcode.localization.factor.AbsolutePoseFactor;
import org.firstinspires.ftc.teamcode.localization.factor.MotionDeltaFactor;
import org.junit.jupiter.api.Tag;
import org.junit.jupiter.api.Test;

/** Manual host benchmark for the complete fusion workload executed by one robot loop. */
@Tag("benchmark")
class FusionLoopBenchmarkTest {
    private static final long STEP_NANOS = 20_000_000L; // 50 Hz robot loop
    private static final int TOTAL_STEPS = 600;
    private static final int MEASURE_FROM_STEP = 100; // measure only after the 2 s window is full
    private static final int TAG_PERIOD_STEPS = 5; // 10 Hz camera
    private static final int TAG_LATENCY_STEPS = 5; // 100 ms processing latency

    @Test void completeLoopWithNumericalProcessJacobian() {
        BenchmarkResult result = result(model());
        for (int step = 1; step <= TOTAL_STEPS; step++) executeStep(result, step);

        print("all 50 Hz loops", result.allLoops);
        print("process + OctoQuad loops", result.loopsWithoutTag);
        print("process + OctoQuad + delayed AprilTag loops", result.loopsWithTag);
    }

    private static BenchmarkResult result(ProcessModel<ControlInput> model) {
        return new BenchmarkResult(model, engine(model));
    }

    private static void executeStep(BenchmarkResult result, int step) {
        long previousTimestamp = (step - 1L)*STEP_NANOS;
        long timestamp = step*STEP_NANOS;
        double seconds = timestamp*1e-9;
        Pose2 previous = truth(previousTimestamp*1e-9);
        Pose2 current = truth(seconds);
        ControlInput input = new ControlInput(
                .45 + .12*Math.sin(seconds), .15*Math.cos(.6*seconds), .08);
        boolean tagAvailable = step % TAG_PERIOD_STEPS == 0 && step > TAG_LATENCY_STEPS;
        long loopStart = System.nanoTime();

        List<Factor> tickFactors = new ArrayList<>();
        tickFactors.add(result.model.factor(previousTimestamp, timestamp, input));
        double heading = current.heading();
        double fieldVx = 24;
        double fieldVy = 5.6*Math.cos(.7*seconds);
        double forwardVelocity = Math.cos(heading)*fieldVx + Math.sin(heading)*fieldVy;
        double strafeVelocity = -Math.sin(heading)*fieldVx + Math.cos(heading)*fieldVy;
        tickFactors.add(new MotionDeltaFactor(
                "octoquad", previousTimestamp, timestamp,
                previous.localDeltaTo(current), forwardVelocity, strafeVelocity, .3,
                MatrixOps.diagonal(.0025, .0025, .0001, .04, .04, .0025)));
        if (tagAvailable) {
            long captureTimestamp = timestamp - TAG_LATENCY_STEPS*STEP_NANOS;
            result.engine.submitFactor(new AbsolutePoseFactor(
                    "apriltag-camera", captureTimestamp, truth(captureTimestamp*1e-9),
                    MatrixOps.diagonal(.04, .04, .0025)));
        }
        if (!result.engine.commitTick(timestamp, tickFactors).isAccepted()) {
            throw new AssertionError("fusion tick was rejected");
        }

        if (step > MEASURE_FROM_STEP) {
            long elapsed = System.nanoTime() - loopStart;
            result.allLoops.add(elapsed);
            (tagAvailable ? result.loopsWithTag : result.loopsWithoutTag).add(elapsed);
        }
    }

    private static PoseVelocityProcessModel model() {
        return new PoseVelocityProcessModel(new PoseVelocityProcessModel.Config(
                8, 60, 60, 4,
                new double[] {.02, .02, .002, .2, .2, .02}));
    }

    private static FusionEngine engine(ProcessModel<ControlInput> model) {
        return new FusionEngine(
                RobotState.create(Pose2.zero(), 0, 0, 0,
                        MatrixOps.diagonal(1, 1, .1, 10, 10, 1)).gaussian(),
                0, model::normalize,
                2_000_000_000L);
    }

    private static Pose2 truth(double seconds) {
        return new Pose2(24*seconds, 8*Math.sin(.7*seconds), .3*seconds);
    }

    private static void print(String label, List<Long> samples) {
        Collections.sort(samples);
        System.out.printf("%s: n=%d mean=%.3f ms p50=%.3f ms p95=%.3f ms max=%.3f ms%n",
                label, samples.size(), mean(samples)/1e6,
                percentile(samples, .50)/1e6, percentile(samples, .95)/1e6,
                samples.get(samples.size() - 1)/1e6);
    }

    private static double mean(List<Long> samples) {
        double total = 0;
        for (long sample : samples) total += sample;
        return total/samples.size();
    }

    private static long percentile(List<Long> samples, double percentile) {
        return samples.get(Math.min(samples.size() - 1,
                (int)Math.ceil(percentile*samples.size()) - 1));
    }

    private static final class BenchmarkResult {
        final ProcessModel<ControlInput> model;
        final FusionEngine engine;
        final List<Long> allLoops = new ArrayList<>();
        final List<Long> loopsWithoutTag = new ArrayList<>();
        final List<Long> loopsWithTag = new ArrayList<>();

        BenchmarkResult(ProcessModel<ControlInput> model, FusionEngine engine) {
            this.model = model;
            this.engine = engine;
        }
    }
}
