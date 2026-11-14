package org.firstinspires.ftc.teamcode.subsystemTesting;

import static dev.frozenmilk.dairy.mercurial.continuations.Continuations.expression;
import static dev.frozenmilk.dairy.mercurial.continuations.Continuations.loop;
import static dev.frozenmilk.dairy.mercurial.continuations.Continuations.noop;
import static dev.frozenmilk.dairy.mercurial.continuations.Continuations.seconds;
import static dev.frozenmilk.dairy.mercurial.continuations.Continuations.sequence;
import static dev.frozenmilk.dairy.mercurial.continuations.Continuations.waitFor;
import static dev.frozenmilk.dairy.mercurial.continuations.Continuations.waitUntil;

import com.acmerobotics.dashboard.config.Config;

import org.firstinspires.ftc.teamcode.constants.Constants;
import org.firstinspires.ftc.teamcode.subsystems.TransferGate;

import dev.frozenmilk.dairy.mercurial.ftc.GamepadD;
import dev.frozenmilk.dairy.mercurial.ftc.MercurialFTC;
import dev.frozenmilk.dairy.mercurial.pedro.MercurialLocalizer;
import dev.frozenmilk.dairy.mercurial.processes.EventManager;
import dev.frozenmilk.dairy.mercurial.processes.Fiber;


@Config
public class TransferGateTest {

    public static double transferGatePosition = 0.0;
    public static boolean setPosition = false;
    public static boolean setClosed = false;
    public static boolean setOpen = false;

    public static final MercurialFTC.RegisterableProgram TransferGateTest = teleOp()
            .withName("Transfer Gate Test")
            .withGroup("Testing");

    public static MercurialFTC.RegisterableProgram teleOp() {
        return MercurialFTC.teleop(ctx -> {
                    TransferGate transferGate = new TransferGate(ctx);
                    loop(
                            expression((scope, _self) -> {
                                scope.runExec(() -> {
                                    if(setClosed){
                                        transferGate.setClosed();
                                    } else if(setOpen){
                                        transferGate.setOpen();
                                    } else {
                                        transferGate.setCustomPositionMode(transferGatePosition);
                                    }
                                });
                                return noop;
                            })
                    ).spawnable().spawnLink();
//


                    ctx.waitForStart();
                    ctx.dropToScheduler();
                    return null;

                }
        );
    }




}
