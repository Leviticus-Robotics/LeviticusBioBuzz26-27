package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.pedropathing.math.Pose;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.teamcode.constants.Constants;
import org.firstinspires.ftc.teamcode.localization.pedro.FusedLocalizer;
import org.firstinspires.ftc.teamcode.localization.sensor.octoquad.OctoQuadFusionSource;

/** Read-only side-by-side localization diagnostic. A/B/X select process/raw/combined display modes. */
@Config
@TeleOp(name = "Localization Diagnostics")
public final class LocalizationDiagnostics extends OpMode {
    public static double commandVelocityScale = 80;
    public static double commandAngularScale = 4;
    private FusedLocalizer localizer;
    private OctoQuadFusionSource octoQuad;
    private Mode mode = Mode.COMBINED;

    @Override public void init() {
        Constants.FusedLocalizationComponents components =
                Constants.createFusedLocalizationComponents(
                        hardwareMap, commandVelocityScale, commandAngularScale);
        localizer = components.localizer;
        octoQuad = components.octoQuad;
    }

    @Override public void loop() {
        if (gamepad1.a) mode = Mode.PROCESS_ONLY;
        if (gamepad1.b) mode = Mode.OCTOQUAD_ONLY;
        if (gamepad1.x) mode = Mode.COMBINED;
        if (mode == Mode.OCTOQUAD_ONLY) localizer.setDriveInput(0, 0, 0);
        else localizer.setDriveInput(
                -gamepad1.left_stick_y, -gamepad1.left_stick_x, -gamepad1.right_stick_x);
        if (mode == Mode.PROCESS_ONLY) localizer.predictToNow();
        else localizer.update();
        Pose raw = octoQuad.rawLocalizer().pose();
        telemetry.addData("Mode", mode);
        telemetry.addData("Raw OctoQuad", raw);
        for (java.util.Map.Entry<String, Object> entry : localizer.debug().entrySet()) {
            telemetry.addData("Fusion " + entry.getKey(), entry.getValue());
        }
        telemetry.update();
    }

    private enum Mode { PROCESS_ONLY, OCTOQUAD_ONLY, COMBINED }
}
