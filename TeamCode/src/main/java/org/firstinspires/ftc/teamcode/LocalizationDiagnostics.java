package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.pedropathing.math.Pose;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.teamcode.constants.Constants;
import org.firstinspires.ftc.teamcode.helper.NanoClock;
import org.firstinspires.ftc.teamcode.localization.LocalizationRuntime;

/** Read-only side-by-side localization diagnostic. A/B/X select process/raw/combined display modes. */
@Config
@TeleOp(name = "Localization Diagnostics")
public final class LocalizationDiagnostics extends OpMode {
    public static double commandVelocityScale = 80;
    public static double commandAngularScale = 4;
    private LocalizationRuntime runtime;
    private Mode mode = Mode.COMBINED;

    @Override public void init() {
        runtime = new LocalizationRuntime(hardwareMap, Constants.localizerConfig,
                Constants::isStationaryLocalizationTag, NanoClock.SYSTEM,
                Constants.fusionLocalizationConfig, commandVelocityScale, commandAngularScale);
    }

    @Override public void loop() {
        if (gamepad1.a) mode = Mode.PROCESS_ONLY;
        if (gamepad1.b) mode = Mode.OCTOQUAD_ONLY;
        if (gamepad1.x) mode = Mode.COMBINED;
        if (mode == Mode.OCTOQUAD_ONLY) runtime.setDriveInput(0, 0, 0);
        else runtime.setDriveInput(-gamepad1.left_stick_y, -gamepad1.left_stick_x, -gamepad1.right_stick_x);
        if (mode == Mode.PROCESS_ONLY) runtime.localizer().update();
        else runtime.updateOctoQuad();
        Pose raw = runtime.rawOctoQuadLocalizer().pose();
        telemetry.addData("Mode", mode);
        telemetry.addData("Raw OctoQuad", raw);
        runtime.addTelemetry(telemetry);
        telemetry.update();
    }

    private enum Mode { PROCESS_ONLY, OCTOQUAD_ONLY, COMBINED }
}
