package org.firstinspires.ftc.teamcode;

import static dev.frozenmilk.dairy.mercurial.continuations.Continuations.expression;
import static dev.frozenmilk.dairy.mercurial.continuations.Continuations.loop;
import static dev.frozenmilk.dairy.mercurial.continuations.Continuations.noop;
import static dev.frozenmilk.dairy.mercurial.continuations.Continuations.seconds;
import static dev.frozenmilk.dairy.mercurial.continuations.Continuations.waitFor;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.pedropathing.drivetrain.DrivePowers;
import com.pedropathing.follower.ManualDrive;
import com.pedropathing.math.Pose;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.subsystems.Drivetrain;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.Shooter;
import org.firstinspires.ftc.teamcode.constants.Constants;

import dev.frozenmilk.dairy.mercurial.ftc.GamepadD;
import dev.frozenmilk.dairy.mercurial.ftc.MercurialFTC;
import dev.frozenmilk.dairy.mercurial.pedro.MercurialFollower;
import dev.frozenmilk.dairy.mercurial.processes.EventManager;
import dev.frozenmilk.dairy.mercurial.processes.EventManager.EventHandled;
import dev.frozenmilk.dairy.mercurial.processes.Fiber;

@Config
public class ShooterTesting{
    final static Pose defaultPose = new Pose(0, 0, Math.toRadians(90));
    public static double fixedPower = 1;
    public static double velocity = 2250;

    //Create Mercurial FTC TeleOp
    public static final MercurialFTC.RegisterableProgram ShooterTesting = teleOp(new Pose(0, 0))
            .withName("Shooter Testing")
            .withGroup("Testing");

    public static MercurialFTC.RegisterableProgram teleOp(Pose pose){
        return MercurialFTC.teleop(ctx -> {
            EventManager<Double> voltageEventManager = new EventManager<>();
            Shooter shooter = new Shooter(ctx.hardwareMap());

            class DriveSettings{
                boolean fieldCentric = false;
                boolean slowMode = false;
            }
            DriveSettings driveSettings = new DriveSettings();

            EventManager.Handler<GamepadD.Delta> gamepad2Handler =
                    GamepadD.gamepad2Handler( event -> {
                        if(event.leftBumperWasPressed()) shooter.fixed(fixedPower, false);
                        else if(event.rightBumperWasPressed()) shooter.fixed(velocity, true);
                        else if(event.rightTrigger() > 0.5) shooter.idle();
                        if(shooter.fiber().status().alive()){
                            return EventHandled.ok();
                        } else{
                            return EventHandled.remove();
                        }
                    }
            );

            Telemetry telemetry = new MultipleTelemetry(
                    ctx.telemetry(),
                    FtcDashboard.getInstance().getTelemetry()
            );


            Fiber<?> telemetryFiber = loop(
                    expression((scope, _self) -> {
                        scope.runExec(() -> {
                            telemetry.clear();
                            telemetry.addData("Field Centric", driveSettings.fieldCentric);
                            telemetry.addData("Slow Mode", driveSettings.slowMode);
                            telemetry.addData("Shooter Velocity", shooter.getVelocity());
                            telemetry.addData("Shooter Mode", shooter.mode());
                            telemetry.addData("Shooter Corrected Velocity", shooter.shooter.getCorrectedVelocity());
                            telemetry.update();
                        });
                        scope.run(waitFor.bind(seconds(0.1)));
                        return noop;
                    })
            ).spawnable().spawnLink();



            ctx.waitForStart();
            ctx.gamepadD().addHandler(gamepad2Handler);
            ctx.dropToScheduler();
            return null;
        }


        );



    }


}
