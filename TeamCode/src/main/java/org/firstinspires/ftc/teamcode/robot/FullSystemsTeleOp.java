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
import dev.frozenmilk.dairy.mercurial.processes.SwitchedStateMachine;


@Config
public class FullSystemsTeleOp {
    final static Pose defaultPose = new Pose(0, 0, Math.toRadians(90));



    public static final MercurialFTC.RegisterableProgram SystemTesting = teleOp(new Pose(0, 0), ALLIANCE.RED, HiveTargetSide.Right)
            .withName("Full TeleOp Mock")
            .withGroup("Competition");

    public static MercurialFTC.RegisterableProgram teleOp(Pose startPose, ALLIANCE alliance, HiveTargetSide startSide) {
        return MercurialFTC.teleop(ctx -> {
            RobotCombined r = new RobotCombined(ctx, startSide);
            r.localizer.setPose(startPose);

                    //spawns a spawnable to reset turret encoder position from absolute & set turret to center
                    expression((scope, self) -> {
                        scope.run(waitUntil(ctx::inInit));
                        scope.run(waitFor.bind(seconds(2)));//waits for 2 seconds
                        scope.run(r.turretEncoder.resetEncoderSync());//do this to avoid hitting maximum values
                        scope.run(r.turretEncoder.resetFromAbsoluteSync());
                        //resets incremental encoder on turret, reads from absolute
                        return noop;
                    }).spawnable().spawnLink();

                    class DriveSettings {
                        boolean fieldCentric = false;
                    }
                    DriveSettings driveSettings = new DriveSettings();

                    EventManager.Handler<GamepadD.Delta> drivetrainHandler =
                        GamepadD.gamepad1Handler(event -> {
                            if (event.leftBumperWasPressed()) {
                                driveSettings.fieldCentric = !driveSettings.fieldCentric;
                                if (driveSettings.fieldCentric) r.drivetrain.fieldCentric();
                                r.drivetrain.robotCentric();
                            }

                            r.drivetrain.sendDrivePowers(
                                    -event.leftStickY(),
                                    -event.leftStickX(),
                                    -event.rightStickX());
                            if(checkFiberStatus(r.drivetrain)) {
                                return EventManager.EventHandled.ok();
                            } else{
                                return EventManager.EventHandled.remove();
                            }
                        }
                    );

                    EventManager.Handler<GamepadD.Delta> shootingIntakeHandler =
                        GamepadD.gamepad1Handler(event -> {
                            boolean intakeActive = Math.abs(event.rightTrigger()) > 0.15;
                            double power = event.rightTrigger() * (event.rightBumper() ? -1 : 1);


                            //run assiststed
                            if (intakeActive) {
                                r.requestManualControl(power);
                            } else if (event.rightTriggerPassedUnder(0.15)) {
                                r.endDirectManualFeed();
                            }

                            if(event.rightBumperWasPressed()){
                                r.handleTransferGateTrigger(alliance, r.sideMachine.getSide(), intakeActive);
                            }else if (event.rightTriggerPassedUnder(0.3)) {
                                r.startAutomatic(alliance, r.sideMachine.getSide(), 1000, false);
                            }

                            if (event.triangleWasPressed()) {
                                r.requestEarlyShot();
                            }
                            if (event.crossWasPressed()) {
                                r.confirmBallsLoaded();
                            }

                            if (checkFiberStatus(r.intake) && checkFiberStatus(r.transfer) && checkFiberStatus(r.turret)
                                    && r.transferGate.fiber().status().alive()) {
                                return EventManager.EventHandled.ok();
                            } else {
                                return EventManager.EventHandled.remove();
                            }
                        }
                    );

                    EventManager.Handler<GamepadD.Delta> flowerHandler =
                        GamepadD.gamepad1Handler(event -> {
                            if (event.dpadDownWasPressed()) {//m2 right backside remapped
                                r.flowerWedge.setDown();
                            } else if (event.dpadUpWasPressed()) {//m1 left backside remapped
                                r.flowerWedge.setUp();
                            }
                            return EventManager.EventHandled.ok();
                        }
                    );

                    EventManager.Handler<GamepadD.Delta> shootingSideHandler =
                        GamepadD.gamepad2Handler(event -> {
                            if (event.leftBumperWasPressed()) {
                                r.sideMachine.setSide(HiveTargetSide.Left);
                            } else if (event.rightBumperWasPressed()) {
                                r.sideMachine.setSide(HiveTargetSide.Right);
                            }
                            return EventManager.EventHandled.ok();
                        }
                    );


                    ctx.waitForStart();
                    ctx.gamepadD().addHandler(drivetrainHandler);
                    ctx.gamepadD().addHandler(shootingSideHandler);
                    ctx.gamepadD().addHandler(shootingIntakeHandler);
                    ctx.gamepadD().addHandler(flowerHandler);
                    ctx.dropToScheduler();
                    return null;

                }
        );
    }
    private static boolean checkFiberStatus(SwitchedStateMachine stateMachine){
        return stateMachine.fiber().status().alive();
    }



}
