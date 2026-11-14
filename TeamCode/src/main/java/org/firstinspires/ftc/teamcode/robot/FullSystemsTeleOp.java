package org.firstinspires.ftc.teamcode.robot;

import static dev.frozenmilk.dairy.mercurial.continuations.Continuations.expression;
import static dev.frozenmilk.dairy.mercurial.continuations.Continuations.loop;
import static dev.frozenmilk.dairy.mercurial.continuations.Continuations.noop;
import static dev.frozenmilk.dairy.mercurial.continuations.Continuations.seconds;
import static dev.frozenmilk.dairy.mercurial.continuations.Continuations.waitFor;
import static dev.frozenmilk.dairy.mercurial.continuations.Continuations.waitUntil;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.pedropathing.math.Pose;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.game.ALLIANCE;
import org.firstinspires.ftc.teamcode.game.HiveTargetSide;

import dev.frozenmilk.dairy.mercurial.ftc.GamepadD;
import dev.frozenmilk.dairy.mercurial.ftc.MercurialFTC;
import dev.frozenmilk.dairy.mercurial.processes.EventManager;
import dev.frozenmilk.dairy.mercurial.processes.Fiber;
import dev.frozenmilk.dairy.mercurial.processes.Spawnable;


@Config
public class FullSystemsTeleOp {
    final static Pose defaultPose = new Pose(0, 0, Math.toRadians(90));



    public static final MercurialFTC.RegisterableProgram SystemTesting = teleOp(new Pose(0, 0), ALLIANCE.RED, HiveTargetSide.CLOSE)
            .withName("Full Test")
            .withGroup("Testing");

    public static MercurialFTC.RegisterableProgram teleOp(Pose startPose, ALLIANCE alliance, HiveTargetSide startSide) {
        return MercurialFTC.teleop(ctx -> {
                    Telemetry multiTelemetry = new MultipleTelemetry(
                            ctx.telemetry(),
                            FtcDashboard.getInstance().getTelemetry()
                    );
                    multiTelemetry.setAutoClear(false);
                    ctx.telemetry(multiTelemetry);
                    RobotCombined r = new RobotCombined(ctx, startSide);
                    r.localizer.setPose(startPose);

                    //spawns a spawnable to reset turret encoder position from absolute & set turret to center
                    Spawnable<?> program = expression((scope, self) -> {
                        scope.run(waitUntil(ctx::inInit));
                        scope.run(waitFor.bind(seconds(2)));//waits for 2 seconds
                        scope.run(r.turretEncoder.resetEncoderSync());//do this to avoid hitting maximum values
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
                            GamepadD.gamepad1Handler(
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
                                    )
                            );

                    EventManager.Handler<GamepadD.Delta> shootingIntakeHandler =
                            GamepadD.gamepad1Handler(event -> {
                                        boolean manualPower = false;
                                        if (Math.abs(event.rightTrigger()) > 0.1) {
                                            manualPower = true;
                                        }
                                        double power = event.rightTrigger() * (event.rightBumper() ? -1 : 1);
                                        //if we triggered the gate manual control or assisted shooting
                                        //"b" interrupt automatic mode too
                                        if (event.squareWasPressed()) {
                                            r.requestManaulShoot(alliance, r.sideMachine.getSide(), manualPower, power, power);
                                        } else if (Math.abs(power) > 0.1) {
                                            //if we attempt to power intake, exit out of whatever mode or
                                            r.beginDirectManualFeed(power, power);
                                        } else if (event.rightTriggerPassedUnder(0.1)) {
                                            r.endDirectManualFeed();
                                        }

                                        if (event.circleWasPressed()) {
                                            r.startAutomatic(alliance, r.sideMachine.getSide(), 1000, false);
                                        }

                                        if (event.triangleWasPressed()) {
                                            r.requestEarlyShot();
                                        }
                                        if (event.crossWasPressed() && r.isAutomaticRunning()) {
                                            r.requestBallDetect();
                                        }

                                        if (r.intake.fiber().status().alive()
                                                && r.shooter.fiber().status().alive()
                                                && r.turret.fiber().status().alive()
                                                && r.transferGate.fiber().status().alive()) {
                                            return EventManager.EventHandled.ok();
                                        } else {
                                            return EventManager.EventHandled.remove();
                                        }
                                    }
                            );

                    EventManager.Handler<GamepadD.Delta> shootingSideHandler =
                            GamepadD.gamepad1Handler(event -> {
                                        if (event.leftBumperWasPressed()) {
                                            r.sideMachine.setSide(HiveTargetSide.CLOSE);
                                        } else if (event.leftTriggerOver(0.3)) {
                                            r.sideMachine.setSide(HiveTargetSide.Far);
                                        }
                                        return EventManager.EventHandled.ok();
                                    }
                            );





                    loop(
                            expression((scope, _self) -> {
                                scope.runExec(() -> {
                                    ctx.telemetry().clear();
                                    ctx.telemetry().addData("Field Centric", driveSettings.fieldCentric);
                                    ctx.telemetry().addData("Slow Mode", driveSettings.slowMode);
                                    ctx.telemetry().addData("Shooter Velocity", r.shooter.getVelocity());
                                    ctx.telemetry().addData("Shooter Mode", r.shooter.mode());
                                    ctx.telemetry().update();
                                });
                                scope.run(waitFor.bind(seconds(0.1)));
                                return noop;
                            })
                    ).spawnable().spawnLink();


                    ctx.gamepadD().addHandler(drivetrainHandler);
                    ctx.gamepadD().addHandler(shootingSideHandler);
                    ctx.gamepadD().addHandler(shootingIntakeHandler);


                    ctx.waitForStart();
                    ctx.dropToScheduler();
                    return null;

                }
        );
    }




}
