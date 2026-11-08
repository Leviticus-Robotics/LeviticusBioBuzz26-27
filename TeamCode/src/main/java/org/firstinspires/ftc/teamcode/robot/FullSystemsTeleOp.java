package org.firstinspires.ftc.teamcode.robot;

import static dev.frozenmilk.dairy.mercurial.continuations.Continuations.expression;
import static dev.frozenmilk.dairy.mercurial.continuations.Continuations.noop;
import static dev.frozenmilk.dairy.mercurial.continuations.Continuations.seconds;
import static dev.frozenmilk.dairy.mercurial.continuations.Continuations.waitFor;
import static dev.frozenmilk.dairy.mercurial.continuations.Continuations.waitUntil;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.pedropathing.math.Pose;

import dev.frozenmilk.dairy.mercurial.ftc.GamepadD;
import dev.frozenmilk.dairy.mercurial.ftc.MercurialFTC;
import dev.frozenmilk.dairy.mercurial.processes.EventManager;
import dev.frozenmilk.dairy.mercurial.processes.Spawnable;


@Config
public class FullSystemsTeleOp {
    final static Pose defaultPose = new Pose(0, 0, Math.toRadians(90));

    public static final MercurialFTC.RegisterableProgram SystemTesting = teleOp(new Pose(0, 0))
            .withName("Full Test")
            .withGroup("Testing");

    public static MercurialFTC.RegisterableProgram teleOp(Pose startPose){
        return MercurialFTC.teleop(ctx -> {
            RobotCombined r = new RobotCombined(ctx);
            r.localizer.setPose(startPose);
            FtcDashboard dashboard = FtcDashboard.getInstance();

            //spawns a spawnable to reset turret encoder position from absolute & set turret to center
            Spawnable<?> program = expression((scope, self) -> {
                scope.run(waitUntil(ctx::inInit));
                scope.run(waitFor.bind(seconds(2)));//waits for 2 seconds
                scope.run(r.turretEncoder.resetEncoderSync());
                scope.run(r.turretEncoder.resetFromAbsoluteSync());
                //resets incremental encoder on turret, reads from absolute
                return noop;
            }).spawnable();

            class DriveSettings {
                boolean fieldCentric = false;
                boolean slowMode = false;
                boolean prevFieldCentric = false;
            }
            DriveSettings driveSettings = new DriveSettings();


            //this handles the drive inputs
            EventManager.Handler<GamepadD.Delta> drivetrainHandler =
                    GamepadD.gamepad2Handler(
                            EventManager.forwardTo(r.drivetrain, event -> {
                                        //if current drive settings is different from previous one
                                        if (driveSettings.prevFieldCentric != driveSettings.fieldCentric) {
                                            if (driveSettings.fieldCentric) {
                                                r.drivetrain.fieldCentric();
                                            } else {
                                                r.drivetrain.robotCentric();
                                            }
                                        }

                                        double multiplier = driveSettings.slowMode ? 0.4 : 1;
                                        r.drivetrain.sendDrivePowers(
                                                event.leftStickX() * multiplier,
                                                event.leftStickY() * multiplier,
                                                event.rightStickY() * multiplier
                                        );
                                    }
                            ));


            ctx.gamepadD().addHandler(drivetrainHandler);
            ctx.dropToScheduler();
            return null;
        });
    };
}
