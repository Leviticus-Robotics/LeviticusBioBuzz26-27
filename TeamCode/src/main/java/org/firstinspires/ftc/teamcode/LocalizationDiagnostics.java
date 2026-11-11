package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.pedropathing.math.Pose;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.constants.Constants;
import org.firstinspires.ftc.teamcode.localization.pedro.FusedLocalizer;
import org.firstinspires.ftc.teamcode.localization.sensor.CommandProcessSource;
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

    public static final RegisterableProgram localizationDiagnostics = diagnostic(new Pose(0, 0))
            .withName("Localization Diagnostics")
            .withGroup("Testing");

    public static RegisterableProgram diagnostic(Pose startPose) {
        return MercurialFTC.teleop(ctx -> {
            Drivetrain drivetrain = new Drivetrain(ctx);
            CommandProcessSource processSource = Constants.createProcessSource();
            MercurialLocalizer localizer = Constants.createFusedLocalizer(
                    ctx.hardwareMap(), drivetrain, processSource);
            FusedLocalizer fused = (FusedLocalizer) localizer.localizer();
            localizer.setPose(startPose);

            localizer.addHandler(EventManager.forwardTo(
                    drivetrain, drivetrain::sendMotionState));

            Telemetry.Item modeTelemetry = ctx.telemetry().addData(
                    "Localization/Mode", modeName(processSource));
            Telemetry.Item fusedTelemetry = ctx.telemetry().addData(
                    "Localization/Fused", "waiting");
            Telemetry.Item velocityTelemetry = ctx.telemetry().addData(
                    "Localization/Velocity", "waiting");
            Telemetry.Item covarianceTelemetry = ctx.telemetry().addData(
                    "Localization/Covariance diagonal", "waiting");
            Telemetry.Item decisionTelemetry = ctx.telemetry().addData(
                    "Localization/Last decision", "waiting");
            ctx.telemetry().addData("Localization/Controls",
                    "A: toggle process, LB: field/robot centric, RB: slow");

            localizer.addHandler(event -> {
                Map<String, Object> debug = fused.debug();
                fusedTelemetry.setValue(debug.get("pose"));
                velocityTelemetry.setValue(Arrays.toString((double[]) debug.get("velocity")));
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
                        if (event.aWasPressed()) {
                            processSource.setEnabled(!processSource.enabled());
                            modeTelemetry.setValue(modeName(processSource));
                        }
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

    private static String modeName(CommandProcessSource processSource) {
        return processSource.enabled() ? "OctoQuad + process" : "OctoQuad only";
    }
}
