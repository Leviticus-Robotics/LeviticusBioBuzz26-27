package org.firstinspires.ftc.teamcode;

import static dev.frozenmilk.dairy.mercurial.continuations.Continuations.expression;
import static dev.frozenmilk.dairy.mercurial.continuations.Continuations.loop;
import static dev.frozenmilk.dairy.mercurial.continuations.Continuations.noop;
import static dev.frozenmilk.dairy.mercurial.continuations.Continuations.seconds;
import static dev.frozenmilk.dairy.mercurial.continuations.Continuations.waitFor;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.canvas.Canvas;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.pedropathing.drivetrain.DrivePowers;
import com.pedropathing.follower.ManualDrive;
import com.pedropathing.math.Pose;
import com.pedropathing.utils.Timer;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.subsystems.Drivetrain;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.Shooter;
import org.firstinspires.ftc.teamcode.constants.Constants;

import java.util.concurrent.TimeUnit;

import dev.frozenmilk.dairy.mercurial.ftc.GamepadD;
import dev.frozenmilk.dairy.mercurial.ftc.MercurialFTC;
import dev.frozenmilk.dairy.mercurial.pedro.MercurialFollower;
import dev.frozenmilk.dairy.mercurial.processes.EventManager;
import dev.frozenmilk.dairy.mercurial.processes.EventManager.EventHandled;
import dev.frozenmilk.dairy.mercurial.processes.Fiber;

@Config
public class FullPrototypeTest{
    final static Pose defaultPose = new Pose(0, 0, Math.toRadians(90));
    public static double fixedPower = 0.55;
    public static double velocity = 950;
    public static boolean useVelocity = true; // false is power mode


    //Create Mercurial FTC TeleOp
    public static final MercurialFTC.RegisterableProgram ShooterTesting = teleOp(new Pose(0, 0))
            .withName("Full Test")
            .withGroup("Testing");


    public static MercurialFTC.RegisterableProgram teleOp(Pose pose){
        return MercurialFTC.teleop(ctx -> {
            MercurialFollower follower = Constants.createMercurial(ctx.hardwareMap());
            Shooter shooter = new Shooter(ctx.hardwareMap(), follower.localizationEventManager(), ctx.hardwareMap().voltageSensor.iterator().next());
            Drivetrain drivetrain = new Drivetrain(follower);
            Intake intake = new Intake(ctx.hardwareMap());
            Timer timer = new Timer();
            follower.localizer().setPose(pose);
            FtcDashboard dashboard = FtcDashboard.getInstance();


            class DriveSettings{
                boolean fieldCentric = false;
                boolean slowMode = false;
                double previousTime = 0;
            }



            DriveSettings driveSettings = new DriveSettings();
            EventManager.Handler<GamepadD.Delta> gamepad1Handler =
                    GamepadD.gamepad1Handler(event -> {

                        //drivetrain
                        if(event.squareWasPressed()){
                            driveSettings.fieldCentric = !driveSettings.fieldCentric;
                        }
                        else if(event.bWasPressed()){
                            follower.localizer().setPose(defaultPose);
                        }

                        double multiplier = driveSettings.slowMode ? 0.4 : 1;
                        if(!driveSettings.fieldCentric){
                            drivetrain.sendDrivePowers(
                                    new DrivePowers(
                                            -event.leftStickY() * multiplier,
                                            -event.leftStickX() * multiplier,
                                            -event.rightStickY() * multiplier
                                    )
                            );
                        } else{
                            drivetrain.sendDrivePowers(
                                    ManualDrive.fieldCentric(
                                            event.leftStickX() * multiplier,
                                            -event.leftStickY() * multiplier,
                                            -event.rightStickY() * multiplier,
                                            follower.localizer().pose().heading()
                                    )
                            );

                        }

                        //intake
                        if(event.dpadLeftWasPressed()) {
                            intake.fixed(1, false);
                        } else if(event.dpadRightWasPressed()){
                            intake.fixed(-1, false);
                        } else if(event.dpadDownWasPressed()){
                            intake.idle();
                        }

                        //shooter
                        if(event.leftBumperWasPressed()) shooter.fixed(useVelocity ? velocity: fixedPower, useVelocity);
                        else if(event.rightBumperWasPressed()) shooter.idle();



                        if(drivetrain.fiber().status().alive() && intake.fiber().status().alive() && shooter.fiber().status().alive()){
                            return EventHandled.ok();
                        } else{
                            return EventHandled.remove();
                        }
                    }
            );

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

//                    Fiber<?> voltageEventFiber = loop(
//                            expression((scope, _self) -> {
////                                scope.runExec(()-> {
//                                voltageEventManager.notify(ctx.hardwareMap().voltageSensor.iterator().next().getVoltage());
////                                });
//                                return noop;
//                            })
//                    ).spawnable().spawnLink();

                    Fiber<?> telemetryFiber = loop(
                            expression((scope, _self) -> {
                                scope.runExec(() -> {
                                    telemetry.clear();

                                    double deltaTime = timer.get(TimeUnit.MILLISECONDS) - driveSettings.previousTime;
                                    telemetry.addData("Loop Time", deltaTime);
                                    driveSettings.previousTime = timer.get(TimeUnit.MILLISECONDS);
                                    telemetry.addData("Field Centric", driveSettings.fieldCentric);
                                    telemetry.addData("Slow Mode", driveSettings.slowMode);
                                    telemetry.addData("Drivetrain Mode", drivetrain.mode());
                                    Pose robotPose = follower.localizer().pose();;
                                    telemetry.addData("Pose x", robotPose.x());
                                    telemetry.addData("Pose y", robotPose.y());
                                    telemetry.addData("Pose Heading(Deg)", convertToRadians(robotPose.heading()));
                                    telemetry.addData("Shooter Velocity", shooter.getVelocity());
                                    telemetry.addData("Shooter Mode", shooter.mode());
                                    telemetry.addData("Previous Voltage", shooter.previousVoltage);
                                    telemetry.addData("Intake Mode", intake.mode());
                                    telemetry.update();

                                    double x = robotPose.x();
                                    double y = robotPose.y();
                                    double heading = robotPose.heading(); // radians

                                    TelemetryPacket packet = new TelemetryPacket();
                                    Canvas canvas = packet.fieldOverlay();


                                    canvas.setStroke("#00FF00")
                                            .setStrokeWidth(2)
                                            .strokeCircle(x, y, 9);

                                    packet.put("X (in)", robotPose.x());
                                    packet.put("Y (in)", robotPose.y());
                                    packet.put("Heading (deg)", Math.toDegrees(heading));

                                    dashboard.sendTelemetryPacket(packet);
                                });
                                return noop;
                            })
                    ).spawnable().spawnLink();



                    ctx.waitForStart(); //set to manual mode instead of idle
                    ctx.gamepadD().addHandler(gamepad1Handler);
                    ctx.gamepadD().addHandler(gamepad2Handler);
                    ctx.dropToScheduler();
                    return null;
                }


        );




    }



    public static double convertToRadians(double heading){
        double headingDeg = heading * 180.00 / Math.PI;
        while(headingDeg < 0){
            headingDeg += 360;
        }
        while(headingDeg > 360){
            headingDeg -= 360;
        }
        return headingDeg;
    }


}
