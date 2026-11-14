package org.firstinspires.ftc.teamcode.subsystemTesting;

import static dev.frozenmilk.dairy.mercurial.continuations.Continuations.expression;
import static dev.frozenmilk.dairy.mercurial.continuations.Continuations.loop;
import static dev.frozenmilk.dairy.mercurial.continuations.Continuations.noop;
import static dev.frozenmilk.dairy.mercurial.continuations.Continuations.seconds;
import static dev.frozenmilk.dairy.mercurial.continuations.Continuations.sequence;
import static dev.frozenmilk.dairy.mercurial.continuations.Continuations.waitFor;
import static dev.frozenmilk.dairy.mercurial.continuations.Continuations.waitUntil;

import com.acmerobotics.dashboard.config.Config;
import com.pedropathing.localization.Localizer;

import org.firstinspires.ftc.teamcode.constants.Constants;
import org.firstinspires.ftc.teamcode.subsystems.Drivetrain;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.Transfer;
import org.firstinspires.ftc.teamcode.subsystems.TransferGate;

import dev.frozenmilk.dairy.mercurial.ftc.GamepadD;
import dev.frozenmilk.dairy.mercurial.ftc.MercurialFTC;
import dev.frozenmilk.dairy.mercurial.pedro.MercurialLocalizer;
import dev.frozenmilk.dairy.mercurial.processes.EventManager;
import dev.frozenmilk.dairy.mercurial.processes.Fiber;


@Config
public class DrivetrainTest {

    public static double intakePower = 0.0;
    public static boolean runIntake = false;
    public static boolean runTransfer = false;
    public static double transferPower = 0.0;
    public static boolean gateClose = false;
    public static final MercurialFTC.RegisterableProgram DrivetrainTesting = teleOp()
            .withName("Drivetrain Test")
            .withGroup("Testing");

    public static MercurialFTC.RegisterableProgram teleOp() {
        return MercurialFTC.teleop(ctx -> {
                    class DriveSettings {
                        boolean fieldCentric = false;
                        boolean slowMode = false;
                        boolean prevFieldCentric = false;
                    }
                    DriveSettings driveSettings = new DriveSettings();
                    Drivetrain drivetrain = new Drivetrain(ctx);
                    MercurialLocalizer localizer = new MercurialLocalizer(Constants.createLocalizer(ctx.hardwareMap()));
                    localizer.addHandler(EventManager.forwardTo(
                            drivetrain,
                            drivetrain::sendMotionState
                    ));

                    EventManager.Handler<GamepadD.Delta> drivetrainHandler =
                            GamepadD.gamepad1Handler(
                                    EventManager.forwardTo(drivetrain, event -> {
                                                //if current drive settings is different from previous one
                                                if (driveSettings.prevFieldCentric != driveSettings.fieldCentric) {
                                                    if (driveSettings.fieldCentric) {
                                                        drivetrain.fieldCentric();
                                                    } else {
                                                        drivetrain.robotCentric();
                                                    }
                                                }

                                                double multiplier = driveSettings.slowMode ? 0.4 : 1;
                                                drivetrain.sendDrivePowers(
                                                        event.leftStickX() * multiplier,
                                                        event.leftStickY() * multiplier,
                                                        event.rightStickY() * multiplier
                                                );
                                            }
                                    )
                            );


                    Intake intake = new Intake(ctx);
                    Transfer transfer = new Transfer(ctx);
                    TransferGate transferGate = new TransferGate(ctx);

                    loop(
                            expression((scope, _self) -> {
                                scope.runExec(() -> {
                                    if(runIntake) intake.fixedPower(intakePower);
                                    if(runTransfer) transfer.fixedPower(transferPower);
                                    if(gateClose) transferGate.setClosed();
                                    else transferGate.setOpen();
                                });
                                return noop;
                            })
                    ).spawnable().spawnLink();

                    loop(
                            expression((scope, _self) -> {
                                scope.runExec(() -> {
                                    ctx.telemetry().clear();
                                    ctx.telemetry().addData("Field Centric", driveSettings.fieldCentric);
                                    ctx.telemetry().addData("Slow Mode", driveSettings.slowMode);
                                    ctx.telemetry().update();
                                });
                                scope.run(waitFor.bind(seconds(0.1)));
                                return noop;
                            })
                    ).spawnable().spawnLink();


                    ctx.gamepadD().addHandler(drivetrainHandler);
                    ctx.waitForStart();
                    ctx.dropToScheduler();
                    return null;

                }
        );
    }




}
