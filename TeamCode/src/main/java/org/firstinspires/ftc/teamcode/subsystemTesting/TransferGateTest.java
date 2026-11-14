package org.firstinspires.ftc.teamcode.subsystemTesting;


import com.acmerobotics.dashboard.config.Config;

import org.firstinspires.ftc.teamcode.constants.Constants;
import org.firstinspires.ftc.teamcode.subsystems.TransferGate;

import dev.frozenmilk.dairy.mercurial.ftc.GamepadD;
import dev.frozenmilk.dairy.mercurial.ftc.MercurialFTC;
import dev.frozenmilk.dairy.mercurial.pedro.MercurialLocalizer;
import dev.frozenmilk.dairy.mercurial.processes.EventManager;
import dev.frozenmilk.dairy.mercurial.processes.Fiber;
import dev.frozenmilk.dairy.mercurial.processes.SwitchedStateMachine;


@Config
public class TransferGateTest {
    public static final MercurialFTC.RegisterableProgram transferGateTest = teleOp()
            .withName("Transfer Gate Test")
            .withGroup("Subsystem Testing");

    public static MercurialFTC.RegisterableProgram teleOp() {
        return MercurialFTC.teleop(ctx -> {
                    TransferGate transferGate = new TransferGate(ctx);
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
