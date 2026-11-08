package org.firstinspires.ftc.teamcode.localization;

import static org.junit.jupiter.api.Assertions.*;

import com.pedropathing.math.Pose;
import java.io.IOException;
import java.nio.file.Files;
import java.nio.file.Path;
import java.nio.file.Paths;
import java.util.concurrent.atomic.AtomicLong;
import java.util.concurrent.atomic.AtomicInteger;
import org.firstinspires.ftc.teamcode.fusion.core.MatrixOps;
import org.firstinspires.ftc.teamcode.fusion.engine.FusionEngine;
import org.firstinspires.ftc.teamcode.localization.core.ControlInput;
import org.firstinspires.ftc.teamcode.localization.core.Pose2;
import org.firstinspires.ftc.teamcode.localization.core.RobotState;
import org.firstinspires.ftc.teamcode.localization.engine.PoseVelocityProcessModel;
import org.firstinspires.ftc.teamcode.localization.pedro.FusedLocalizer;
import org.junit.jupiter.api.Test;

class FusedLocalizerTest {
    private static FusionEngine<ControlInput> engine() {
        return new FusionEngine<>(
                RobotState.create(Pose2.zero(), 1, 2, 3,
                        MatrixOps.diagonal(1, 1, 1, 1, 1, 1)).gaussian(), 0,
                new PoseVelocityProcessModel(1, 1, 1, 1, new double[] {.1, .1, .1, .1, .1, .1}),
                ControlInput.ZERO,
                10_000_000_000L, 100);
    }

    @Test void adaptsPoseVelocityUpdateResetAndDebugWithoutPollingSensors() {
        AtomicLong time = new AtomicLong(0);
        AtomicInteger sourceResets = new AtomicInteger();
        FusedLocalizer localizer = new FusedLocalizer(engine(), time::get,
                MatrixOps.diagonal(1, 1, 1, 1, 1, 1), sourceResets::incrementAndGet);
        assertEquals(1, localizer.velocity().vx, 1e-12);
        localizer.setPose(new Pose(4, 5, .2));
        assertEquals(4, localizer.pose().x(), 1e-12);
        time.set(1_000_000_000L);
        localizer.update();
        assertEquals(1_000_000_000L, localizer.engine().timestampNanos());
        assertTrue(localizer.debug().containsKey("covarianceDiagonal"));
        localizer.reset();
        assertEquals(0, localizer.pose().x(), 1e-12);
        assertEquals(2, sourceResets.get());
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
                    .filter(path -> !path.getFileName().toString().equals("FusionFollower.java"))
                    .filter(path -> !path.getFileName().toString().equals("FusionMecanumDrivetrain.java"))
                    .filter(path -> !path.getFileName().toString().equals("LocalizationRuntime.java"))
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
}
