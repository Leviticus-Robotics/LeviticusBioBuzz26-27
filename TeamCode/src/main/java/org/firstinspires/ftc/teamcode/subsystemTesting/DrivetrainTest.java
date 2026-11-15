package org.firstinspires.ftc.teamcode.subsystemTesting;

import com.acmerobotics.dashboard.config.Config;
import com.pedropathing.localization.Localizer;
import com.pedropathing.math.Pose;

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
import dev.frozenmilk.dairy.mercurial.processes.SwitchedStateMachine;


@Config
public class DrivetrainTest {
    public static double slowMultiplier = 0.4;
    public static final MercurialFTC.RegisterableProgram drivetrainTesting = teleOp(new Pose(0, 0))
            .withName("Drivetrain Test")
            .withGroup("Subsystem Testing");

    public static MercurialFTC.RegisterableProgram teleOp(Pose startPose) {
        return MercurialFTC.teleop(ctx -> {
                    Drivetrain drivetrain = new Drivetrain(ctx);
//                    MercurialLocalizer localizer = new MercurialLocalizer(Constants.createLocalizer(ctx.hardwareMap()));
//                    localizer.setPose(startPose);

//                    localizer.addHandler(EventManager.forwardTo(
//                            drivetrain,
//                            drivetrain::sendMotionState
//                    ));
//                    class DriveSettings {
//                        boolean fieldCentric;
//                    }
//                    DriveSettings driveSettings = new DriveSettings();

                    EventManager.Handler<GamepadD.Delta> gamepadHandler =
                            GamepadD.gamepad1Handler(event -> {
//                                if (event.leftBumperWasPressed()) {
//                                    driveSettings.fieldCentric = !driveSettings.fieldCentric;
//                                    if (driveSettings.fieldCentric) drivetrain.fieldCentric();
                                    drivetrain.robotCentric();
//                                }

                                double multiplier = event.rightBumper() ? slowMultiplier : 1.0;
                                drivetrain.sendDrivePowers(
                                        -event.leftStickY() * multiplier,
                                        -event.leftStickX() * multiplier,
                                        -event.rightStickX() * multiplier);
                                if(checkFiberStatus(drivetrain)) {
                                    return EventManager.EventHandled.ok();
                                } else{
                                    return EventManager.EventHandled.remove();
                                }
                    });


                    ctx.waitForStart();
                    drivetrain.robotCentric();
                    ctx.gamepadD().addHandler(gamepadHandler);
                    ctx.dropToScheduler();
                    return null;

                }
        );
    }

    private static boolean checkFiberStatus(SwitchedStateMachine stateMachine){
        return stateMachine.fiber().status().alive();
    }




}
