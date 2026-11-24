package org.firstinspires.ftc.teamcode.subsystemTesting;

import static dev.frozenmilk.dairy.mercurial.continuations.Continuations.exec;
import static dev.frozenmilk.dairy.mercurial.continuations.Continuations.expression;
import static dev.frozenmilk.dairy.mercurial.continuations.Continuations.loop;
import static dev.frozenmilk.dairy.mercurial.continuations.Continuations.noop;
import static dev.frozenmilk.dairy.mercurial.continuations.Continuations.seconds;
import static dev.frozenmilk.dairy.mercurial.continuations.Continuations.sequence;
import static dev.frozenmilk.dairy.mercurial.continuations.Continuations.waitFor;
import static dev.frozenmilk.dairy.mercurial.continuations.Continuations.waitUntil;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.pedropathing.localization.Localizer;
import com.pedropathing.math.Pose;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.constants.Constants;
import org.firstinspires.ftc.teamcode.game.ALLIANCE;
import org.firstinspires.ftc.teamcode.game.HiveTargetSide;
import org.firstinspires.ftc.teamcode.helper.BallCounter;
import org.firstinspires.ftc.teamcode.sensor.TransferBeamBreak;
import org.firstinspires.ftc.teamcode.sensor.TurretEncoder;
import org.firstinspires.ftc.teamcode.sensor.VoltageMonitor;
import org.firstinspires.ftc.teamcode.subsystems.Drivetrain;
import org.firstinspires.ftc.teamcode.subsystems.FlowerWedge;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.Shooter;
import org.firstinspires.ftc.teamcode.subsystems.Transfer;
import org.firstinspires.ftc.teamcode.subsystems.TransferGate;
import org.firstinspires.ftc.teamcode.subsystems.Turret;
import org.firstinspires.ftc.teamcode.util.Angle;

import dev.frozenmilk.dairy.mercurial.ftc.GamepadD;
import dev.frozenmilk.dairy.mercurial.ftc.MercurialFTC;
import dev.frozenmilk.dairy.mercurial.pedro.MercurialLocalizer;
import dev.frozenmilk.dairy.mercurial.processes.EventManager;
import dev.frozenmilk.dairy.mercurial.processes.Fiber;
import dev.frozenmilk.dairy.mercurial.processes.SwitchedStateMachine;


@Config
public class Test2 {
    public static final MercurialFTC.RegisterableProgram test2 = teleOp(new Pose(8, 8, Math.toRadians(0)), ALLIANCE.RED, HiveTargetSide.Right)
            .withName("TEST 2")
            .withGroup("Subsystem Testing");
    public static double fixedIntakePower = 1;
    public static double fixedTransferPower = 1;
    public static double hoodPosition = 0.5;
    public static double shooterVelocity = 850;

    public static MercurialFTC.RegisterableProgram teleOp(Pose startPose, ALLIANCE alliance, HiveTargetSide side) {

        return MercurialFTC.teleop(ctx -> {
                    Drivetrain drivetrain = new Drivetrain(ctx);
//                    VoltageMonitor voltageMonitor = new VoltageMonitor(ctx);
                    MercurialLocalizer localizer = new MercurialLocalizer(Constants.createLocalizer(ctx.hardwareMap()));
                    localizer.setPose(startPose);
                    Turret turret = new Turret(ctx, true);
//                    TurretEncoder turretEncoder = new TurretEncoder(ctx);
                    Shooter shooter = new Shooter(ctx);
                    Intake intake = new Intake(ctx);
                    Transfer transfer = new Transfer(ctx);
                    TransferGate transferGate = new TransferGate(ctx);
                    TransferBeamBreak transferSensors = new TransferBeamBreak(ctx);
                    BallCounter ballCounter = new BallCounter(intake, transfer);
                    FlowerWedge flowerWedge = new FlowerWedge(ctx);

                    Telemetry.Item ballCounterTelemetry = ctx.telemetry().addData(
                            "BallCounter/Balls", "waiting");
                    Telemetry.Item poseTelemetry = ctx.telemetry().addData(
                            "Pose", "waiting"

                    );
                    ctx.telemetry().addLine("-------------------------------");
                    Telemetry.Item distanceFromGoal = ctx.telemetry().addData(
                            "Distance from goal (In): ", "waiting"
                    );
                    Telemetry.Item shooterVelocity = ctx.telemetry().addData(
                            "Shooter Velocity (t/s): ", "waiting"
                    );
                    Telemetry.Item shooterTargetVelocity = ctx.telemetry().addData(
                            "Shooter Target Vel (t/s): ", "waiting"
                    );

                    ctx.telemetry().addLine("-------------------------------");

//                    shooter.setHeadingChannel(turret.getTargetHeadingsChannel());
//                    ctx.telemetry().addLine(
//                            "Left bumper hold: ball shooting");
//                    ctx.telemetry().addLine(
//                            "Right bumper hold: intake");
//                    ctx.telemetry().addLine(
//                            "Right trigger hold: reverse intake");
//                    ctx.telemetry().addLine(
//                            "None: Stop Intake/Transfer/Gate");
//
//                    ctx.telemetry().addLine(
//                            "Left trigger toggle: Flower");



                    localizer.addHandler(event -> {
                        drivetrain.sendMotionState(event);
                        shooter.sendMotionState(event);
                        return EventManager.EventHandled.ok();
                    });


                    //send localizer pose to shooter state machine
//                    voltageMonitor.addHandler(EventManager.forwardTo(
//                            shooter,
//                            shooter::updateVoltage
//                    ));

                    transferSensors.addHandler(event -> {
                        if(event.sensorNum == 1){
                            transfer.updateBreaker1(event.interrupted);
                            intake.updateBreaker1(event.interrupted);
                        } else{
                            transfer.updateBreaker2(event.interrupted);
                            intake.updateBreaker2(event.interrupted);
                        }
                        //update Telemetry here
                        ballCounterTelemetry.setValue(ballCounter.totalBalls());
                        return EventManager.EventHandled.ok();
                    });
                    class DriveSettings {
                        boolean fieldCentric = false;
                    }

                    DriveSettings driveSettings = new DriveSettings();
                    EventManager.Handler<GamepadD.Delta> drivetrainHandler =
                            GamepadD.gamepad1Handler(event -> {
                                if (event.dpadDownWasPressed()) {
                                    driveSettings.fieldCentric = !driveSettings.fieldCentric;
                                    if (driveSettings.fieldCentric) drivetrain.fieldCentric();
                                    else drivetrain.robotCentric();
                                }
                                drivetrain.sendDrivePowers(
                                        -event.leftStickY(),
                                        -event.leftStickX(),
                                        -event.rightStickX());
                                return EventManager.EventHandled.ok();
                            });

                    EventManager.Handler<GamepadD.Delta> intakeTransferHandler =
                            GamepadD.gamepad1Handler(event -> {
                                    if(event.leftBumperWasPressed())
                                    {
                                        transferGate.setOpen();
                                        intake.fixedPower(1);
                                        transfer.fixedPower(1);
                                    }
                                    else if(event.rightBumperWasPressed()) {
                                        intake.fixedPower(fixedIntakePower);
                                        transfer.runPowerTillDetection(fixedTransferPower);
                                        transferGate.setClosed();
                                    } else if(event.rightTriggerPassedOver(0.2)){
                                        intake.fixedPower(-1);
                                        transfer.fixedPower(-1);
                                        transferGate.setClosed();
                                    }
                                    if(event.leftBumperWasReleased() && event.rightBumperWasReleased() && event.rightTriggerPassedUnder(0.2)){
                                        intake.fixedPower(0);
                                        transfer.fixedPower(0);
                                        transferGate.setClosed();
                                    }




                                    return EventManager.EventHandled.ok();
                                }
                            );


                    EventManager.Handler<GamepadD.Delta> flowerMechHandler =
                        GamepadD.gamepad1Handler(event -> {
                            if(event.leftTriggerPassedOver(0.5)){
                                if (flowerWedge.getPrevPosition() == FlowerWedge.UP_POSITION) flowerWedge.setDown();
                                else flowerWedge.setUp();
                            }
                            return EventManager.EventHandled.ok();
                        }
                    );

//
//                    expression((scope, self) -> {
//                                scope.run(waitUntil(ctx::inInit));
//                                scope.run(waitFor.bind(seconds(2)));//waits for 2 seconds
////                                exec(() -> turretEncoder.zeroIncremental());//do this to avoid hitting maximum values
////                                exec(() -> turretEncoder.resetFromAbsolute());
//                                //resets incremental encoder on turret, reads from absolute
//                                return noop;
//                            }).spawnable().spawnLink();a
                    ctx.waitForStart();
                    drivetrain.robotCentric();
                    shooter.setUseDefaultVelocity(false);
                    shooter.lutStatic(alliance, side);
//                    turret.setTrackingMode(false);
                    ctx.gamepadD().addHandler(intakeTransferHandler);
                    ctx.gamepadD().addHandler(drivetrainHandler);
                    ctx.gamepadD().addHandler(flowerMechHandler);



                    loop(
                            sequence(
                                    exec(() -> distanceFromGoal.setValue(shooter.distanceFromGoal)),
                                    exec(()-> shooterVelocity.setValue(shooter.getVelocity())),
                                    exec(()-> shooterTargetVelocity.setValue(shooter.getShootingTargetVelocity())),
                                    exec(() -> poseTelemetry.setValue(localizer.pose())),
                                    waitFor.bind(seconds(0.25))
                            )

                    ).spawnable().spawnLink();


                    ctx.dropToScheduler();
                    return null;

                }
        );
    }

    public static void updateTelemetry(Shooter shooter){
//        dashboardTelem.addData("Velocity", shooter.getVelocity());
//        dashboardTelem.update();

    }




}
