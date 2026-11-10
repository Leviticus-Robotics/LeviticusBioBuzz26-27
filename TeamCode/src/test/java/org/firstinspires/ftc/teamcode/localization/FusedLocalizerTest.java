package org.firstinspires.ftc.teamcode.localization;

import static org.junit.jupiter.api.Assertions.*;

import com.pedropathing.math.Pose;
import java.io.IOException;
import java.nio.file.Files;
import java.nio.file.Path;
import java.nio.file.Paths;
import java.util.concurrent.atomic.AtomicLong;
import java.util.concurrent.atomic.AtomicInteger;
import java.util.Collections;
import java.util.Arrays;
import org.firstinspires.ftc.teamcode.fusion.core.MatrixOps;
import org.firstinspires.ftc.teamcode.fusion.engine.FusionEngine;
import org.firstinspires.ftc.teamcode.localization.core.Pose2;
import org.firstinspires.ftc.teamcode.localization.core.RobotState;
import org.firstinspires.ftc.teamcode.localization.engine.PoseVelocityProcessModel;
import org.firstinspires.ftc.teamcode.localization.factor.AbsolutePoseFactor;
import org.firstinspires.ftc.teamcode.localization.factor.PoseDeltaFactor;
import org.firstinspires.ftc.teamcode.localization.pedro.FusedLocalizer;
import org.firstinspires.ftc.teamcode.localization.sensor.AutomaticFusionSource;
import org.firstinspires.ftc.teamcode.localization.sensor.CommandProcessSource;
import org.junit.jupiter.api.Test;

class FusedLocalizerTest {
    private static PoseVelocityProcessModel model() {
        return new PoseVelocityProcessModel(new PoseVelocityProcessModel.Config(
                1, 1, 1, 1, new double[] {.1, .1, .1, .1, .1, .1}));
    }

    private static FusionEngine engine() {
        return new FusionEngine(
                RobotState.create(Pose2.zero(), 1, 2, 3,
                        MatrixOps.diagonal(1, 1, 1, 1, 1, 1)).gaussian(), 0,
                model()::normalize,
                10_000_000_000L);
    }

    private static FusedLocalizer.Config config() {
        return FusedLocalizer.Config.defaults().withInitialCovariance(
                MatrixOps.diagonal(1, 1, 1, 1, 1, 1));
    }

    @Test void adaptsPoseVelocityUpdateResetAndDebugWithInjectedSources() {
        AtomicLong time = new AtomicLong(0);
        FakeSource source = new FakeSource();
        FusedLocalizer localizer = new FusedLocalizer(
                engine(), time::get, config(), sources(source));
        assertEquals(1, localizer.velocity().vx, 1e-12);
        localizer.setPose(new Pose(4, 5, .2));
        assertEquals(4, localizer.pose().x(), 1e-12);
        time.set(1_000_000_000L);
        localizer.update();
        assertEquals(1_000_000_000L, localizer.engine().timestampNanos());
        assertTrue(localizer.debug().containsKey("covarianceDiagonal"));
        localizer.reset();
        assertEquals(0, localizer.pose().x(), 1e-12);
        assertEquals(3, source.resets.get());
        assertEquals(1, source.polls.get());
        assertEquals(1, source.commits.get());
    }

    @Test void updateAppliesAutomaticFactorBeforePublishingCurrentState() {
        AtomicLong time = new AtomicLong(1_000_000_000L);
        FakeSource source = new FakeSource();
        source.next = new PoseDeltaFactor("automatic", 0, time.get(),
                new Pose2(5, 0, 0), MatrixOps.diagonal(.001, .001, .001));
        FusedLocalizer localizer = new FusedLocalizer(
                engine(), time::get, config(), sources(source));

        localizer.update();

        assertEquals(1, source.polls.get());
        assertTrue(localizer.pose().x() > 1, "x=" + localizer.pose().x());
        assertEquals(time.get(), localizer.engine().timestampNanos());
    }

    @Test void externalFactorIsAppliedAtNextAtomicTick() {
        FusedLocalizer localizer = new FusedLocalizer(
                engine(), () -> 0, config(),
                Collections.emptyList());

        localizer.submitFactor(new AbsolutePoseFactor("camera", 0,
                new Pose2(6, 2, .3), MatrixOps.diagonal(.001, .001, .001)));
        assertEquals(0, localizer.pose().x(), 1e-12);
        localizer.update();

        assertTrue(localizer.pose().x() > 5.9);
        assertTrue(localizer.pose().y() > 1.9);
    }

    @Test void updateUsesOneTimestampForPollingAndCommit() {
        AtomicLong time = new AtomicLong(1_000_000_000L);
        FakeSource source = new FakeSource();
        source.onPoll = () -> time.set(2_000_000_000L);
        FusedLocalizer localizer = new FusedLocalizer(
                engine(), time::get, config(),
                sources(source));

        localizer.update();

        assertEquals(1_000_000_000L, localizer.engine().timestampNanos());
    }

    @Test void rejectedTickRollsBackEveryAutomaticSource() {
        FakeSource source = new FakeSource();
        FusedLocalizer localizer = new FusedLocalizer(
                engine(), () -> 1_000_000_000L,
                config(),
                Collections.singletonList(source));

        assertThrows(IllegalStateException.class, localizer::update);
        assertEquals(0, source.commits.get());
        assertEquals(1, source.rollbacks.get());
        assertEquals(0, localizer.engine().timestampNanos());
    }

    @Test void coreLocalizationTypesDoNotImportPedroOrSensorSdks() throws IOException {
        Path root = Paths.get("src/main/java/org/firstinspires/ftc/teamcode/localization");
        if (!Files.exists(root)) root = Paths.get("TeamCode").resolve(root);
        final Path sourceRoot = root;
        try (java.util.stream.Stream<Path> paths = Files.walk(sourceRoot)) {
            paths.filter(path -> path.toString().endsWith(".java"))
                    .filter(path -> !path.toString().contains("/sensor/"))
                    .filter(path -> !path.toString().contains("/pedro/"))
                    .filter(path -> !path.getFileName().toString().equals("FusedLocalizer.java"))
                    .filter(path -> !path.getFileName().toString().equals("FusionMecanumDrivetrain.java"))
                    .forEach(path -> {
                        try {
                            String source = new String(Files.readAllBytes(path), java.nio.charset.StandardCharsets.UTF_8);
                            assertFalse(source.contains("import com.pedropathing"), path.toString());
                            assertFalse(source.contains("import com.qualcomm.hardware"), path.toString());
                            assertFalse(source.contains("import org.firstinspires.ftc.vision"), path.toString());
                        } catch (IOException exception) {
                            throw new RuntimeException(exception);
                        }
                    });
        }
    }

    private static final class FakeSource implements AutomaticFusionSource {
        final AtomicInteger polls = new AtomicInteger();
        final AtomicInteger resets = new AtomicInteger();
        final AtomicInteger commits = new AtomicInteger();
        final AtomicInteger rollbacks = new AtomicInteger();
        PoseDeltaFactor next;
        Runnable onPoll = () -> { };

        @Override public PoseDeltaFactor poll(long fromTimestampNanos,
                                              long tickTimestampNanos) {
            polls.incrementAndGet();
            onPoll.run();
            PoseDeltaFactor result = next;
            next = null;
            return result;
        }

        @Override public void commit() { commits.incrementAndGet(); }
        @Override public void rollback() { rollbacks.incrementAndGet(); }
        @Override public void reset(long timestampNanos) { resets.incrementAndGet(); }
    }

    private static java.util.List<AutomaticFusionSource> sources(FakeSource source) {
        return Arrays.asList(new CommandProcessSource(model()), source);
    }
}
