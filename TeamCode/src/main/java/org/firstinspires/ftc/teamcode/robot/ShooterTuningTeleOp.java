package org.firstinspires.ftc.teamcode.robot;

import static dev.frozenmilk.dairy.mercurial.continuations.Continuations.expression;
import static dev.frozenmilk.dairy.mercurial.continuations.Continuations.loop;
import static dev.frozenmilk.dairy.mercurial.continuations.Continuations.noop;
import static dev.frozenmilk.dairy.mercurial.continuations.Continuations.seconds;
import static dev.frozenmilk.dairy.mercurial.continuations.Continuations.sequence;
import static dev.frozenmilk.dairy.mercurial.continuations.Continuations.waitFor;
import static dev.frozenmilk.dairy.mercurial.continuations.Continuations.waitUntil;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.pedropathing.math.Pose;

import org.firstinspires.ftc.teamcode.game.Alliance;
import org.firstinspires.ftc.teamcode.game.HiveTargetSide;

import dev.frozenmilk.dairy.mercurial.ftc.GamepadD;
import dev.frozenmilk.dairy.mercurial.ftc.MercurialFTC;
import dev.frozenmilk.dairy.mercurial.processes.EventManager;
import dev.frozenmilk.dairy.mercurial.processes.Fiber;
import dev.frozenmilk.dairy.mercurial.processes.Spawnable;
import dev.frozenmilk.dairy.mercurial.processes.SwitchedStateMachine;


@Config
public class ShooterTuningTeleOp {
    final static Pose defaultPose = new Pose(0, 0, Math.toRadians(90));



    public static final MercurialFTC.RegisterableProgram ShooterTesting = teleOp(new Pose(0, 0), Alliance.RED, HiveTargetSide.Left)
            .withName("Shooter Test")
            .withGroup("Testing");

    public static MercurialFTC.RegisterableProgram teleOp(Pose startPose, Alliance alliance, HiveTargetSide startSide) {
        return MercurialFTC.teleop(ctx -> {
                    RobotShooterTuning r = new RobotShooterTuning(ctx, alliance, startSide);
//                    r.localizer.setPose(startPose);
//
//                    Spawnable<?> resetTurretEncoderProgram = expression((scope, self) -> {
//                        scope.run(waitUntil(ctx::inInit));
//                        scope.run(waitFor.bind(seconds(2)));//waits for 2 seconds
//                        scope.run(r.turretEncoder.resetEncoderSync());//do this to avoid hitting maximum values
//                        scope.run(r.turretEncoder.resetFromAbsoluteSync());
//                        //resets incremental encoder on turret, reads from absolute
//                        return noop;
//                    }).spawnable();
                    r.drivetrain.robotCentric();
//


                    //this handles the drive inputs
                    EventManager.Handler<GamepadD.Delta> drivetrainHandler =
                            GamepadD.gamepad1Handler(
                                    EventManager.forwardTo(r.drivetrain, event -> {
                                                //if current drive settings is different from previous one

                                            r.drivetrain.sendDrivePowers(
                                                -event.leftStickY(),
                                                -event.leftStickX(),
                                                -event.rightStickX());
                                            }
                                    )
                            );


                    EventManager.Handler<GamepadD.Delta> tuningHandler =
                            GamepadD.gamepad1Handler(event -> {
                                        if(event.leftBumperWasPressed()){
                                            r.tuneKsKv();
                                        }
                                        else if(event.leftTriggerOver(0.3)){
                                            r.tunePID();
                                        }


                                        if(event.rightBumperWasPressed()){
                                            r.tuneDistVelLUT();
                                        }
                                        else if(event.shareWasPressed()){
                                            r.testDistLUT();
                                        }
                                        else if(event.optionsWasPressed()){
                                            r.tuneDistHoodLUT();
                                        }
                                        else if(event.rightTriggerOver(0.3)){
                                            r.tuneHood();
                                        }

                                        if(event.dpadLeftWasPressed()){
                                            r.shoot();
                                        }
                                        if(event.dpadRightWasPressed()){
                                            r.resetGate();
                                        }

                                        if (isFiberActive(r.intake) && isFiberActive(r.transfer) && isFiberActive(r.shooter) && isFiberActive(r.transferGate)){
                                            return EventManager.EventHandled.ok();
                                        } else {
                                            return EventManager.EventHandled.remove();
                                        }
                                    }
                            );

//                    EventManager.Handler<GamepadD.Delta> turretAngleHandler =
//                            GamepadD.gamepad1Handler(event -> {
//
//                                        if(event.crossWasPressed()){
//                                            r.testRawTurretAngle();
//                                        }
//                                        if(event.circleWasPressed()){
//                                            r.testOptimizedTurretAngle();
//                                        }
//
//                                        if(event.triangleWasPressed()) {
//                                            r.setTurretSetPosition();
//                                        }
//                                        return isFiberActive(r.turret) ? EventManager.EventHandled.ok() : EventManager.EventHandled.remove();
//                                    }
//                            );

                    EventManager.Handler<GamepadD.Delta> sideMachine =
                            GamepadD.gamepad2Handler(event -> {
                                    if (event.leftBumperWasPressed()) {
                                        r.sideMachine.setSide(HiveTargetSide.Left);
                                    } else if (event.leftTriggerOver(0.3)) {
                                        r.sideMachine.setSide(HiveTargetSide.Right);
                                    }
                                    return EventManager.EventHandled.ok();
                                }
                            );



                    loop(
                            expression((scope, _self) -> {
                                scope.runExec(r::updateTelemetry);
                                scope.run(waitFor.bind(seconds(0.1)));
                                return noop;
                            })
                    ).spawnable().spawnLink();





                    ctx.waitForStart();
                    ctx.gamepadD().addHandler(drivetrainHandler);
                    ctx.gamepadD().addHandler(tuningHandler);
//                    ctx.gamepadD().addHandler(turretAngleHandler);
                    ctx.gamepadD().addHandler(sideMachine);
                    ctx.dropToScheduler();
                    return null;

                }
        );
    }

    private static boolean isFiberActive(SwitchedStateMachine activeFiber){
        return activeFiber != null && activeFiber.fiber().status().alive();
    }





}
