package org.firstinspires.ftc.teamcode.subsystemTesting;

import static dev.frozenmilk.dairy.mercurial.continuations.Continuations.expression;
import static dev.frozenmilk.dairy.mercurial.continuations.Continuations.loop;
import static dev.frozenmilk.dairy.mercurial.continuations.Continuations.noop;
import static dev.frozenmilk.dairy.mercurial.continuations.Continuations.seconds;
import static dev.frozenmilk.dairy.mercurial.continuations.Continuations.sequence;
import static dev.frozenmilk.dairy.mercurial.continuations.Continuations.waitFor;
import static dev.frozenmilk.dairy.mercurial.continuations.Continuations.waitUntil;

import com.acmerobotics.dashboard.config.Config;

import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.Transfer;
import org.firstinspires.ftc.teamcode.subsystems.TransferGate;

import dev.frozenmilk.dairy.mercurial.ftc.GamepadD;
import dev.frozenmilk.dairy.mercurial.ftc.MercurialFTC;
import dev.frozenmilk.dairy.mercurial.processes.EventManager;
import dev.frozenmilk.dairy.mercurial.processes.Fiber;


@Config
public class IntakeTransferTest {

    public static double intakePower = 0.0;
    public static boolean runIntake = false;
    public static boolean runTransfer = false;
    public static double transferPower = 0.0;
    public static boolean gateClose = false;

    public static final MercurialFTC.RegisterableProgram ShooterTesting = teleOp()
            .withName("Intake Transfer Test")
            .withGroup("Testing");

    public static MercurialFTC.RegisterableProgram teleOp() {
        return MercurialFTC.teleop(ctx -> {
                    Intake intake = new Intake(ctx);
                    Transfer transfer = new Transfer(ctx);
                    TransferGate transferGate = new TransferGate(ctx);

                    loop(
                            expression((scope, _self) -> {
                                scope.runExec(() -> {
                                    if(runIntake) intake.fixedPower(intakePower);
                                    if(runTransfer) transfer.fixedPower(transferPower);
                                    if(gateClose) {
                                        transferGate.setClosed();
                                    }
                                    else {
                                        transferGate.setOpen();
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
