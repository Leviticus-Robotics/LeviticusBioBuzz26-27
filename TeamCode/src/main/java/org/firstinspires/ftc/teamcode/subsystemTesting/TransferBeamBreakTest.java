package org.firstinspires.ftc.teamcode.subsystemTesting;
import com.acmerobotics.dashboard.config.Config;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.teamcode.helper.BallCounter;
import org.firstinspires.ftc.teamcode.sensor.TransferBeamBreak;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.Transfer;
import org.firstinspires.ftc.teamcode.subsystems.TransferGate;

import dev.frozenmilk.dairy.mercurial.ftc.GamepadD;
import dev.frozenmilk.dairy.mercurial.ftc.MercurialFTC;
import dev.frozenmilk.dairy.mercurial.processes.EventManager;
import dev.frozenmilk.dairy.mercurial.processes.Fiber;
import dev.frozenmilk.dairy.mercurial.processes.SwitchedStateMachine;


@Config
public class TransferBeamBreakTest {


    public static final MercurialFTC.RegisterableProgram transferBeamBreakTest = teleOp()
            .withName("Transfer Beam Break Test")
            .withGroup("Subsystem Testing");

    public static boolean setManualIntakeTransfer = true;

    public static MercurialFTC.RegisterableProgram teleOp() {
        return MercurialFTC.teleop(ctx -> {
                    Intake intake = new Intake(ctx);
                    Transfer transfer = new Transfer(ctx);
                    TransferGate transferGate = new TransferGate(ctx);
                    TransferBeamBreak transferSensors = new TransferBeamBreak(ctx);
                    BallCounter ballCounter = new BallCounter(intake, transfer);
                    Telemetry.Item ballCounterTelemetry = ctx.telemetry().addData(
                            "BallCounter/Balls", "waiting");

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


                    EventManager.Handler<GamepadD.Delta> intakeTransferHandler =
                        GamepadD.gamepad1Handler(event -> {
                            if(setManualIntakeTransfer) {
                                intake.fixedPower(event.leftStickY());
                                transfer.fixedPower(event.rightStickY());
                            } else{
                                intake.runPowerTillDetection(1);
                                transfer.runPowerTillDetection(1);
                            }

                            if(event.dpadUpWasPressed()){
                                setManualIntakeTransfer = false;
                            } else if(event.dpadDownWasPressed()){
                                setManualIntakeTransfer = true;
                            }

                            if(checkFiberStatus(intake) && checkFiberStatus(transfer) && transferSensors.fiber().status().alive()){
                                return EventManager.EventHandled.ok();
                            } else{
                                return EventManager.EventHandled.remove();
                            }
                        }

                    );
                    EventManager.Handler<GamepadD.Delta> transferGateHandler =
                        GamepadD.gamepad1Handler(event -> {
                            if(event.leftBumperWasPressed()){
                                transferGate.setOpen();
                            } else if(event.rightBumperWasPressed()){
                                transferGate.setClosed();
                            }
                            if(checkFiberStatus(transferGate)){
                                return EventManager.EventHandled.ok();
                            } else{
                                return EventManager.EventHandled.remove();
                            }
                        }
                    );


                    ctx.waitForStart();
                    ctx.gamepadD().addHandler(intakeTransferHandler);
                    ctx.gamepadD().addHandler(transferGateHandler);
                    ctx.dropToScheduler();
                    return null;

                }
        );
    }

    private static boolean checkFiberStatus(SwitchedStateMachine stateMachine){
        return stateMachine.fiber().status().alive();
    }




}
