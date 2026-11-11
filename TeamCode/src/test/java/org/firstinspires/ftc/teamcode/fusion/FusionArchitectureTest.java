package org.firstinspires.ftc.teamcode.fusion;

import static org.junit.jupiter.api.Assertions.assertFalse;

import java.io.IOException;
import java.nio.charset.StandardCharsets;
import java.nio.file.Files;
import java.nio.file.Path;
import java.nio.file.Paths;
import org.junit.jupiter.api.Test;

class FusionArchitectureTest {
    @Test void fusionHasNoLocalizationOrRoboticsDependencies() throws IOException {
        Path root = Paths.get("src/main/java/org/firstinspires/ftc/teamcode/fusion");
        if (!Files.exists(root)) root = Paths.get("TeamCode").resolve(root);
        try (java.util.stream.Stream<Path> paths = Files.walk(root)) {
            paths.filter(path -> path.toString().endsWith(".java")).forEach(path -> {
                try {
                    String source = new String(Files.readAllBytes(path), StandardCharsets.UTF_8);
                    assertFalse(source.contains("teamcode.localization"), path.toString());
                    assertFalse(source.contains("com.pedropathing"), path.toString());
                    assertFalse(source.contains("com.qualcomm"), path.toString());
                    assertFalse(source.contains("ftc.vision"), path.toString());
                    assertFalse(source.contains("AprilTag"), path.toString());
                    assertFalse(source.contains("OctoQuad"), path.toString());
                } catch (IOException exception) {
                    throw new RuntimeException(exception);
                }
            });
        }
    }
}
