package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.pedropathing.math.Pose;
import com.pedropathing.revhub.localizers.OctoQuadLocalizer;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.teamcode.constants.Constants;
import org.firstinspires.ftc.teamcode.localization.engine.PoseVelocityProcessModel;
import org.firstinspires.ftc.teamcode.localization.pedro.FusedLocalizer;
import org.firstinspires.ftc.teamcode.localization.sensor.CommandProcessSource;
import org.firstinspires.ftc.teamcode.localization.sensor.octoquad.OctoQuadFactorFactory;
import org.firstinspires.ftc.teamcode.localization.sensor.octoquad.OctoQuadFusionSource;

import java.util.Arrays;

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
        PoseVelocityProcessModel processModel = new PoseVelocityProcessModel(
                Constants.fusionProcessModelConfig.withMaximumVelocities(
                        commandVelocityScale, commandVelocityScale, commandAngularScale));
        processSource = new CommandProcessSource(processModel);
        octoQuad = new OctoQuadFusionSource(
                new OctoQuadLocalizer(hardwareMap, Constants.localizerConfig),
                new OctoQuadFactorFactory(
                        Constants.fusedLocalizerConfig.octoQuadMotionCovariance()));
        localizer = Constants.createLocalizer(Arrays.asList(processSource, octoQuad));
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
