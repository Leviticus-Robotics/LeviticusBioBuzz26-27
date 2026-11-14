package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.pedropathing.math.Pose;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.constants.Constants;
import org.firstinspires.ftc.teamcode.localization.prediction.ControlInput;
import org.firstinspires.ftc.teamcode.localization.pedro.FusedLocalizer;
import org.firstinspires.ftc.teamcode.localization.pedro.PredictiveLocalizer;
import org.firstinspires.ftc.teamcode.subsystems.Drivetrain;

import java.util.Arrays;
import java.util.Map;

import dev.frozenmilk.dairy.mercurial.ftc.GamepadD;
import dev.frozenmilk.dairy.mercurial.ftc.MercurialFTC;
import dev.frozenmilk.dairy.mercurial.ftc.MercurialFTC.RegisterableProgram;
import dev.frozenmilk.dairy.mercurial.pedro.MercurialLocalizer;
import dev.frozenmilk.dairy.mercurial.processes.EventManager;

/** Driveable Mercurial diagnostic for OctoQuad-only versus OctoQuad + process fusion. */
@Config
@SuppressWarnings("unused")
public final class LocalizationDiagnostics {
    public static double slowMultiplier = 0.4;
    public static double predictionHorizonSeconds = 0.08;

    public static final RegisterableProgram localizationDiagnostics = diagnostic(new Pose(0, 0))
            .withName("Localization Diagnostics")
            .withGroup("Testing");

    public static RegisterableProgram diagnostic(Pose startPose) {
        return MercurialFTC.teleop(ctx -> {
            Telemetry telemetry = new MultipleTelemetry(
                    ctx.telemetry(), FtcDashboard.getInstance().getTelemetry());
            telemetry.setAutoClear(false);
            ctx.telemetry(telemetry);

            Drivetrain drivetrain = new Drivetrain(ctx);
            PredictiveLocalizer<ControlInput> predictive = Constants.createFusedLocalizer(
                    ctx.hardwareMap(), drivetrain.getInnerDrivetrain());
            MercurialLocalizer localizer = new MercurialLocalizer(predictive);
            FusedLocalizer fused = predictive.fusedLocalizer();
            localizer.setPose(startPose);

            localizer.addHandler(EventManager.forwardTo(
                    drivetrain, drivetrain::sendMotionState));

            Telemetry.Item fusedTelemetry = telemetry.addData(
                    "Localization/Fused", "waiting");
            Telemetry.Item predictedTelemetry = telemetry.addData(
                    "Localization/Predicted", "waiting");
            Telemetry.Item velocityTelemetry = telemetry.addData(
                    "Localization/Velocity", "waiting");
            Telemetry.Item covarianceTelemetry = telemetry.addData(
                    "Localization/Covariance diagonal", "waiting");
            Telemetry.Item decisionTelemetry = telemetry.addData(
                    "Localization/Last decision", "waiting");
            telemetry.addData("Localization/Controls",
                    "LB: field/robot centric, RB: slow");

            localizer.addHandler(event -> {
                Map<String, Object> debug = fused.debug();
                fusedTelemetry.setValue(debug.get("pose"));
                predictedTelemetry.setValue(predictive.predict(predictionHorizonSeconds).pose());
                velocityTelemetry.setValue(debug.get("velocity"));
                covarianceTelemetry.setValue(
                        Arrays.toString((double[]) debug.get("covarianceDiagonal")));
                decisionTelemetry.setValue(debug.get("lastDecision"));
                return EventManager.EventHandled.ok();
            });

            class DriveSettings {
                boolean fieldCentric;
            }
            DriveSettings driveSettings = new DriveSettings();

            EventManager.Handler<GamepadD.Delta> gamepadHandler =
                    GamepadD.gamepad1Handler(event -> {
                        if (event.leftBumperWasPressed()) {
                            driveSettings.fieldCentric = !driveSettings.fieldCentric;
                            if (driveSettings.fieldCentric) drivetrain.fieldCentric();
                            else drivetrain.robotCentric();
                        }

                        double multiplier = event.rightBumper() ? slowMultiplier : 1.0;
                        drivetrain.sendDrivePowers(
                                -event.leftStickY() * multiplier,
                                event.leftStickX() * multiplier,
                                event.rightStickX() * multiplier);
                        return EventManager.EventHandled.ok();
                    });

            ctx.waitForStart();
            drivetrain.robotCentric();
            ctx.gamepadD().addHandler(gamepadHandler);
            ctx.dropToScheduler();
            return null;
        });
    }

}
