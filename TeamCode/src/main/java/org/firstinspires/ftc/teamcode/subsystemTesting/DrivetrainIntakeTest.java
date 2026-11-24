package org.firstinspires.ftc.teamcode.subsystemTesting;

import static dev.frozenmilk.dairy.mercurial.continuations.Continuations.exec;
import static dev.frozenmilk.dairy.mercurial.continuations.Continuations.loop;

import com.acmerobotics.dashboard.config.Config;
import com.pedropathing.math.Pose;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.constants.Constants;
import org.firstinspires.ftc.teamcode.game.Alliance;
import org.firstinspires.ftc.teamcode.helper.BallCounter;
import org.firstinspires.ftc.teamcode.localization.mercurial.MercurialLocalizer;
import org.firstinspires.ftc.teamcode.sensor.TransferSensors;
import org.firstinspires.ftc.teamcode.subsystems.Drivetrain;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.Shooter;
import org.firstinspires.ftc.teamcode.subsystems.Transfer;
import org.firstinspires.ftc.teamcode.subsystems.TransferGate;

import dev.frozenmilk.dairy.mercurial.ftc.GamepadD;
import dev.frozenmilk.dairy.mercurial.ftc.MercurialFTC;

import dev.frozenmilk.dairy.mercurial.processes.EventManager;


@Config
public class DrivetrainIntakeTest {
    public static boolean setManualIntakeTransfer = true;
    public static final MercurialFTC.RegisterableProgram drivetrainIntakeTesting = teleOp(new Pose(8, 8, Math.toRadians(0)), Alliance.RED)
            .withName("Drivetrain Intake Test")
            .withGroup("Subsystem Testing");

    public static MercurialFTC.RegisterableProgram teleOp(Pose startPose, Alliance alliance) {
        return MercurialFTC.teleop(ctx -> {
                    Drivetrain drivetrain = new Drivetrain(ctx);
                    MercurialLocalizer localizer = new MercurialLocalizer(Constants.createLocalizer(ctx.hardwareMap()));
                    localizer.setPose(startPose);
//                    Turret turret = new Turret(ctx, true);
                    Shooter shooter = new Shooter(ctx);
                    Intake intake = new Intake(ctx);
                    Transfer transfer = new Transfer(ctx);
                    TransferGate transferGate = new TransferGate(ctx);
                    TransferSensors transferSensors = new TransferSensors(ctx);
                    BallCounter ballCounter = new BallCounter(intake, transfer);
                    Telemetry.Item ballCounterTelemetry = ctx.telemetry().addData(
                            "BallCounter/Balls", "waiting");
                    ctx.telemetry().addLine(
                            "Dpad/up: auto-intake, down: manual");
                    ctx.telemetry().addLine(
                            "Dpad/left: open, right: close");
                    ctx.telemetry().addLine(
                            "Left Bumper: toggle field/robot");
                    Telemetry.Item poseTelemetry = ctx.telemetry().addData(
                            "Pose: ", "unknown");


                    localizer.addHandler(EventManager.forwardTo(
                            drivetrain,
                            drivetrain::sendMotionState
                    ));

                    transferSensors.addHandler(event -> {
                        if(event.sensorNum == 1){
                            transfer.updateBreaker1(event.interrupted);
                            intake.updateIntake1(event.interrupted);
                        } else{
                            transfer.updateBreaker2(event.interrupted);
                            intake.updateIntake2(event.interrupted);
                        }
                        //update Telemetry here
                        ballCounterTelemetry.setValue(ballCounter.totalBalls());
                        return EventManager.EventHandled.ok();
                    });

                    class DriveSettings {
                        boolean fieldCentric = false;
                    }

                    DriveSettings driveSettings = new DriveSettings();
                    EventManager.Handler<GamepadD.Delta> drivetrainHandler =
                            GamepadD.gamepad1Handler(event -> {
                                if (event.dpadDownWasPressed()) {
                                    driveSettings.fieldCentric = !driveSettings.fieldCentric;
                                    if (driveSettings.fieldCentric) drivetrain.fieldCentric(alliance);
                                    else drivetrain.robotCentric();
                                }
                                drivetrain.sendDrivePowers(
                                        -event.leftStickY(),
                                        -event.leftStickX(),
                                        -event.rightStickX());
                                return EventManager.EventHandled.ok();
                            });

                    EventManager.Handler<GamepadD.Delta> intakeTransferHandler =
                            GamepadD.gamepad1Handler(event -> {
                                        if(event.rightTriggerOver(0.3)){
                                            setManualIntakeTransfer = true;
                                        }
                                        if(setManualIntakeTransfer){
                                            intake.fixedPower((event.rightBumper() ? -1 : 1) * event.rightTrigger());
                                            if(event.rightBumper()){
                                                transfer.fixedPower(-event.rightTrigger());
                                            } else{
                                                transfer.runPowerTillDetection(event.rightTrigger());
                                            }
                                        }
                                        return EventManager.EventHandled.ok();
                                    }

                    );
                    EventManager.Handler<GamepadD.Delta> shooterHandler =
                            GamepadD.gamepad1Handler(event -> {
                                        if(event.triangleWasPressed()){
                                            shooter.setUseDefaultVelocity(false);
                                            shooter.fixedVelocity(1450, false);
                                        } else if(event.squareWasPressed()){
                                            shooter.off();
                                        }
                                        return EventManager.EventHandled.ok();
                                    }

                            );
                    EventManager.Handler<GamepadD.Delta> transferGateHandler =
                            GamepadD.gamepad1Handler(event -> {
                                if(event.dpadLeftWasPressed()){
                                    setManualIntakeTransfer = false;
                                    transferGate.setOpen();
                                    intake.fixedPower(1.0);
                                    transfer.fixedPower(1.0);
                                } else if(event.dpadRightWasPressed()){
                                    transferGate.setClosed();
                                }
                                return EventManager.EventHandled.ok();
                            });

                    ctx.waitForStart();
                    drivetrain.robotCentric();
                    ctx.gamepadD().addHandler(intakeTransferHandler);
                    ctx.gamepadD().addHandler(drivetrainHandler);
                    ctx.gamepadD().addHandler(transferGateHandler);
                    ctx.gamepadD().addHandler(shooterHandler);

                    loop(
                            exec(() -> {
                                poseTelemetry.setValue(localizer.pose());
                            })

                    ).spawnable().spawnLink();
                    ctx.dropToScheduler();
                    return null;

                }
        );
    }




}
