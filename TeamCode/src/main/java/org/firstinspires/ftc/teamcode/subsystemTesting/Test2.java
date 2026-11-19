package org.firstinspires.ftc.teamcode.subsystemTesting;

import com.acmerobotics.dashboard.config.Config;
import com.pedropathing.localization.Localizer;
import com.pedropathing.math.Pose;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.constants.Constants;
import org.firstinspires.ftc.teamcode.helper.BallCounter;
import org.firstinspires.ftc.teamcode.sensor.TransferBeamBreak;
import org.firstinspires.ftc.teamcode.subsystems.Drivetrain;
import org.firstinspires.ftc.teamcode.subsystems.FlowerWedge;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.Shooter;
import org.firstinspires.ftc.teamcode.subsystems.Transfer;
import org.firstinspires.ftc.teamcode.subsystems.TransferGate;
import org.firstinspires.ftc.teamcode.subsystems.Turret;

import dev.frozenmilk.dairy.mercurial.ftc.GamepadD;
import dev.frozenmilk.dairy.mercurial.ftc.MercurialFTC;
import dev.frozenmilk.dairy.mercurial.pedro.MercurialLocalizer;
import dev.frozenmilk.dairy.mercurial.processes.EventManager;
import dev.frozenmilk.dairy.mercurial.processes.Fiber;
import dev.frozenmilk.dairy.mercurial.processes.SwitchedStateMachine;


@Config
public class Test2 {
    public static final MercurialFTC.RegisterableProgram test2 = teleOp(new Pose(0, 0))
            .withName("TEST 2")
            .withGroup("Subsystem Testing");
    public static double fixedIntakePower = 1;
    public static double fixedTransferPower = 1;
    public static double hoodPosition = 0.5;

    public static MercurialFTC.RegisterableProgram teleOp(Pose startPose) {
        return MercurialFTC.teleop(ctx -> {
                    Drivetrain drivetrain = new Drivetrain(ctx);
//                    MercurialLocalizer localizer = new MercurialLocalizer(Constants.createLocalizer(ctx.hardwareMap()));
//                    localizer.setPose(startPose);
                    Turret turret = new Turret(ctx, true);
                    Shooter shooter = new Shooter(ctx);
                    Intake intake = new Intake(ctx);
                    Transfer transfer = new Transfer(ctx);
                    TransferGate transferGate = new TransferGate(ctx);
                    TransferBeamBreak transferSensors = new TransferBeamBreak(ctx);
                    BallCounter ballCounter = new BallCounter(intake, transfer);
                    FlowerWedge flowerWedge = new FlowerWedge(ctx);
                    Telemetry.Item ballCounterTelemetry = ctx.telemetry().addData(
                            "BallCounter/Balls", "waiting");
//                    ctx.telemetry().addLine(
//                            "Dpad/up: auto-intake, down: manual");
//                    ctx.telemetry().addLine(
//                            "Dpad/left: open, right: close");
//                    ctx.telemetry().addLine(
//                            "Left Bumper: toggle field/robot");


//                    localizer.addHandler(EventManager.forwardTo(
//                            drivetrain,
//                            drivetrain::sendMotionState
//                    ));

                    transferSensors.addHandler(event -> {
                        if(event.sensorNum == 1){
                            transfer.updateBreaker1(event.interrupted);
                        } else{
                            transfer.updateBreaker2(event.interrupted);
                        }
                        //update Telemetry here
                        ballCounterTelemetry.setValue(ballCounter.totalBalls());
                        return EventManager.EventHandled.ok();
                    });

                    transferSensors.addHandler(event -> {
                        if(event.sensorNum == 1){
                            intake.updateBreaker1(event.interrupted);
                        } else{
                            intake.updateBreaker2(event.interrupted);
                        }
                        return EventManager.EventHandled.ok();
                    });

                    EventManager.Handler<GamepadD.Delta> drivetrainHandler =
                            GamepadD.gamepad1Handler(event -> {
//                                if (event.leftBumperWasPressed()) {
//                                    driveSettings.fieldCentric = !driveSettings.fieldCentric;
//                                    if (driveSettings.fieldCentric) drivetrain.fieldCentric();
//                                }
                                drivetrain.robotCentric();

                                drivetrain.sendDrivePowers(
                                        -event.leftStickY(),
                                        -event.leftStickX(),
                                        -event.rightStickX());
                                if(checkFiberStatus(drivetrain)) {
                                    return EventManager.EventHandled.ok();
                                } else{
                                    return EventManager.EventHandled.remove();
                                }
                            });

                    EventManager.Handler<GamepadD.Delta> intakeTransferHandler =
                            GamepadD.gamepad1Handler(event -> {
                                        if(event.rightBumperWasPressed()) {
                                            intake.fixedPower(fixedIntakePower);
                                            transfer.runPowerTillDetection(fixedTransferPower);
                                            transferGate.setClosed();
                                        }
                                        else if(event.rightTriggerOver(0.2)){
                                            intake.fixedPower(-1);
                                            transfer.fixedPower(-1);
                                            transferGate.setClosed();
                                        }
                                        else if(event.leftBumperWasPressed()){
                                            transferGate.setOpen();
                                            intake.fixedPower(1);
                                            transfer.fixedPower(1);
                                        }

                                        if(checkFiberStatus(transfer) && checkFiberStatus(intake) && transferSensors.fiber().status().alive()){
                                            return EventManager.EventHandled.ok();
                                        } else{
                                            return EventManager.EventHandled.remove();
                                        }
                                    }

                            );
                    EventManager.Handler<GamepadD.Delta> shooterHandler =
                        GamepadD.gamepad1Handler(event -> {
                            shooter.setUseDefaultVelocity(false);
                            shooter.fixedVelocity(1450, hoodPosition);
                            if(checkFiberStatus(shooter)){
                                return EventManager.EventHandled.ok();
                            } else{
                                return EventManager.EventHandled.remove();
                            }
                        }

                    );

                    EventManager.Handler<GamepadD.Delta> flowerMechHandler =
                        GamepadD.gamepad1Handler(event -> {
                            if(event.leftTriggerOver(0.5)){
                                if (flowerWedge.getPrevPosition() == FlowerWedge.UP_POSITION) flowerWedge.setDown();
                                else flowerWedge.setUp();
                            }
                            if(checkFiberStatus(flowerWedge)){
                                return EventManager.EventHandled.ok();
                            } else{
                                return EventManager.EventHandled.remove();
                            }
                        }
                    );

                    ctx.waitForStart();
                    drivetrain.robotCentric();
                    ctx.gamepadD().addHandler(intakeTransferHandler);
                    ctx.gamepadD().addHandler(drivetrainHandler);
                    ctx.gamepadD().addHandler(shooterHandler);
                    ctx.gamepadD().addHandler(flowerMechHandler);
                    ctx.dropToScheduler();
                    return null;

                }
        );
    }

    private static boolean checkFiberStatus(SwitchedStateMachine stateMachine){
        return stateMachine.fiber().status().alive();
    }




}
