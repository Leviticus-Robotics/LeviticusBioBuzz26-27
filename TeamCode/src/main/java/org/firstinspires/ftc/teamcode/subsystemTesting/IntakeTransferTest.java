package org.firstinspires.ftc.teamcode.subsystemTesting;

import com.acmerobotics.dashboard.config.Config;

import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.Transfer;
import org.firstinspires.ftc.teamcode.subsystems.TransferGate;

import dev.frozenmilk.dairy.mercurial.ftc.GamepadD;
import dev.frozenmilk.dairy.mercurial.ftc.MercurialFTC;
import dev.frozenmilk.dairy.mercurial.processes.EventManager;
import dev.frozenmilk.dairy.mercurial.processes.Fiber;
import dev.frozenmilk.dairy.mercurial.processes.StateMachine;
import dev.frozenmilk.dairy.mercurial.processes.SwitchedStateMachine;


@Config
public class IntakeTransferTest {

    public static final MercurialFTC.RegisterableProgram shooterTesting = teleOp()
            .withName("Intake Transfer Test")
            .withGroup("Subsystem Testing");

    public static MercurialFTC.RegisterableProgram teleOp() {
        return MercurialFTC.teleop(ctx -> {
                    Intake intake = new Intake(ctx);
                    Transfer transfer = new Transfer(ctx);
                    TransferGate transferGate = new TransferGate(ctx);

                    EventManager.Handler<GamepadD.Delta> intakeTransferHandler =
                            GamepadD.gamepad1Handler(event -> {
                                    intake.fixedPower(event.leftStickY());
                                    transfer.fixedPower(event.rightStickY());
                                    if (checkFiberStatus(intake) & checkFiberStatus(transfer)){
                                        return EventManager.EventHandled.ok();
                                    } else{
                                        return EventManager.EventHandled.remove();
                                    }
                                }
                            );

                    EventManager.Handler<GamepadD.Delta> transferGateHandler =
                            GamepadD.gamepad1Handler(event -> {
                                    if(event.leftBumperWasPressed()){
                                        transferGate.setClosed();
                                    } else{
                                        transferGate.setOpen();
                                    }
                                    if(checkFiberStatus(transferGate)){
                                        return EventManager.EventHandled.ok();
                                    } else{
                                        return EventManager.EventHandled.remove();
                                    }
                            }
                    );

                    ctx.gamepadD().addHandler(intakeTransferHandler);
                    ctx.gamepadD().addHandler(transferGateHandler);
                    ctx.waitForStart();
                    ctx.dropToScheduler();
                    return null;

                }
        );
    }


    private static boolean checkFiberStatus(SwitchedStateMachine stateMachine){
        return stateMachine.fiber().status().alive();
    }



}
