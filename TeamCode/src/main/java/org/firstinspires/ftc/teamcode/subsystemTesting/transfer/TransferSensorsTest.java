package org.firstinspires.ftc.teamcode.subsystemTesting.transfer;
import com.acmerobotics.dashboard.config.Config;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.helper.BallCounter;
import org.firstinspires.ftc.teamcode.sensor.IntakeBallSensors;
import org.firstinspires.ftc.teamcode.sensor.TransferSensors;
import org.firstinspires.ftc.teamcode.subsystems.BulkCacheDaemon;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.Transfer;
import org.firstinspires.ftc.teamcode.subsystems.TransferGate;

import dev.frozenmilk.dairy.mercurial.ftc.GamepadD;
import dev.frozenmilk.dairy.mercurial.ftc.MercurialFTC;
import dev.frozenmilk.dairy.mercurial.processes.EventManager;
import dev.frozenmilk.dairy.mercurial.processes.SwitchedStateMachine;


@Config
public class TransferSensorsTest {


    public static final MercurialFTC.RegisterableProgram transferSensorsTest = teleOp()
            .withName("Transfer Sensors Test")
            .withGroup("Subsystem Testing");

    public static boolean setManualIntakeTransfer = false;

    public static MercurialFTC.RegisterableProgram teleOp() {
        return MercurialFTC.teleop(ctx -> {
                    Intake intake = new Intake(ctx);
                    Transfer transfer = new Transfer(ctx);
                    TransferSensors transferSensors = new TransferSensors(ctx);
                    IntakeBallSensors intakeSensors = new IntakeBallSensors(ctx);
                    TransferGate transferGate = new TransferGate(ctx);
                    BulkCacheDaemon bulkCacheDaemon = new BulkCacheDaemon(ctx);

                    BallCounter ballCounter = new BallCounter(intake, transfer);
                    Telemetry.Item ballCounterTelemetry = ctx.telemetry().addData(
                            "BallCounter/Balls", "waiting");
                    ctx.telemetry().addLine("-------------------------------");
                    Telemetry.Item transfer1DetectedTelemetry = ctx.telemetry().addData(
                            "Transfer1", "unknown");
                    Telemetry.Item transfer2DetectedTelemetry = ctx.telemetry().addData(
                            "Transfer2", "unknown"
                    );
                    Telemetry.Item intake1DetectedTelemetry = ctx.telemetry().addData(
                            "Intake1", "unknown"
                    );
                    Telemetry.Item intake2DetectedTelemetry  = ctx.telemetry().addData(
                            "Intake2", "unknown"
                    );
                    ctx.telemetry().addLine("-------------------------------");


                     transferSensors.addHandler(event -> {
                        if(event.sensorNum == 1){
                            transfer.updateBreaker1(event.interrupted);
                            intake.updateTransfer1(event.interrupted);
                            transfer1DetectedTelemetry.setValue(event.interrupted);
                        } else{
                            transfer2DetectedTelemetry.setValue(event.interrupted);
                            transfer.updateBreaker2(event.interrupted);
                            intake.updateTransfer2(event.interrupted);
                        }
                        ballCounterTelemetry.setValue(ballCounter.totalBalls());
                        return EventManager.EventHandled.ok();
                    });

                    intakeSensors.addHandler(event -> {
                        if(event.sensorNum == 1){
                            intake.updateIntake1(event.interrupted);
                            intake1DetectedTelemetry.setValue(event.interrupted);
                        } else{
                            intake.updateIntake2(event.interrupted);
                            intake2DetectedTelemetry.setValue(event.interrupted);
                        }
                        ballCounter.updateCount();
                        ballCounterTelemetry.setValue(ballCounter.totalBalls());
                        return EventManager.EventHandled.ok();
                    });

                    EventManager.Handler<GamepadD.Delta> intakeTransferHandler =
                        GamepadD.gamepad1Handler(event -> {
                            if(setManualIntakeTransfer) {
                                intake.fixedPower(event.rightStickY());
                                transfer.fixedPower(event.rightStickY());
                            } else{
//                                intake.runPowerTillDetection(1);
                                intake.fixedPower(1);
                                transfer.runPowerTillDetection(1);
                            }

                            if(event.dpadUpWasPressed()){
                                setManualIntakeTransfer = false;
                            } else if(event.dpadDownWasPressed()){
                                setManualIntakeTransfer = true;
                            }

                            return EventManager.EventHandled.ok();
                        }

                    );
                    EventManager.Handler<GamepadD.Delta> transferGateHandler =
                        GamepadD.gamepad1Handler(event -> {
                            if(event.leftBumperWasPressed()){
                                transferGate.setOpen();
                            } else if(event.rightBumperWasPressed()){
                                transferGate.setClosed();
                            }
                            return EventManager.EventHandled.ok();
                        }
                    );

                    transferGate.setClosed();
                    ctx.waitForStart();
                    ctx.gamepadD().addHandler(intakeTransferHandler);
                    ctx.gamepadD().addHandler(transferGateHandler);
                    ctx.dropToScheduler();
                    return null;

                }
        );
    }


}
