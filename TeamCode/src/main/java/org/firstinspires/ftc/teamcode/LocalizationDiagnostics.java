package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.pedropathing.math.Pose;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.teamcode.constants.Constants;
import org.firstinspires.ftc.teamcode.localization.pedro.FusedLocalizer;
import org.firstinspires.ftc.teamcode.localization.sensor.CommandProcessSource;
import org.firstinspires.ftc.teamcode.localization.sensor.octoquad.OctoQuadFusionSource;

/** Read-only side-by-side diagnostic for raw OctoQuad and the combined fused estimate. */
@Config
@TeleOp(name = "Localization Diagnostics")
public final class LocalizationDiagnostics extends OpMode {
    public static double commandVelocityScale = 80;
    public static double commandAngularScale = 4;
    private FusedLocalizer localizer;
    private CommandProcessSource processSource;
    private OctoQuadFusionSource octoQuad;

    @Override public void init() {
        Constants.FusedLocalizationComponents components =
                Constants.createFusedLocalizationComponents(
                        hardwareMap, commandVelocityScale, commandAngularScale);
        localizer = components.localizer;
        processSource = components.processSource;
        octoQuad = components.octoQuad;
    }

    @Override public void loop() {
        processSource.setInput(
                -gamepad1.left_stick_y, -gamepad1.left_stick_x, -gamepad1.right_stick_x);
        localizer.update();
        Pose raw = octoQuad.rawLocalizer().pose();
        telemetry.addData("Process input", processSource.activeInput());
        telemetry.addData("Raw OctoQuad", raw);
        for (java.util.Map.Entry<String, Object> entry : localizer.debug().entrySet()) {
            telemetry.addData("Fusion " + entry.getKey(), entry.getValue());
        }
        telemetry.update();
    }
}
