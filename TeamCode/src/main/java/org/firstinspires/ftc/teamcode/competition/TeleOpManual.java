package org.firstinspires.ftc.teamcode.competition;

import static dev.frozenmilk.dairy.mercurial.continuations.Continuations.exec;
import static dev.frozenmilk.dairy.mercurial.continuations.Continuations.expression;
import static dev.frozenmilk.dairy.mercurial.continuations.Continuations.ifThen;
import static dev.frozenmilk.dairy.mercurial.continuations.Continuations.loop;
import static dev.frozenmilk.dairy.mercurial.continuations.Continuations.noop;
import static dev.frozenmilk.dairy.mercurial.continuations.Continuations.seconds;
import static dev.frozenmilk.dairy.mercurial.continuations.Continuations.sequence;
import static dev.frozenmilk.dairy.mercurial.continuations.Continuations.waitFor;
import static dev.frozenmilk.dairy.mercurial.continuations.Continuations.waitUntil;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.pedropathing.math.Pose;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.constants.Constants;
import org.firstinspires.ftc.teamcode.game.Alliance;
import org.firstinspires.ftc.teamcode.game.HiveTargetSide;
import org.firstinspires.ftc.teamcode.helper.BallCounter;
import org.firstinspires.ftc.teamcode.localization.mercurial.MercurialLocalizer;
import org.firstinspires.ftc.teamcode.sensor.HiveSideHelper;
import org.firstinspires.ftc.teamcode.sensor.IntakeBallSensors;
import org.firstinspires.ftc.teamcode.sensor.TransferSensors;
import org.firstinspires.ftc.teamcode.sensor.TurretEncoder;
import org.firstinspires.ftc.teamcode.sensor.VoltageMonitor;
import org.firstinspires.ftc.teamcode.subsystems.BulkCacheDaemon;
import org.firstinspires.ftc.teamcode.subsystems.Drivetrain;
import org.firstinspires.ftc.teamcode.subsystems.FlowerWedge;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.Shooter;
import org.firstinspires.ftc.teamcode.subsystems.Transfer;
import org.firstinspires.ftc.teamcode.subsystems.TransferGate;
import org.firstinspires.ftc.teamcode.subsystems.Turret;

import dev.frozenmilk.dairy.mercurial.ftc.GamepadD;
import dev.frozenmilk.dairy.mercurial.ftc.MercurialFTC;
import dev.frozenmilk.dairy.mercurial.processes.EventManager;


@Config
public class TeleOpManual {
    public static final MercurialFTC.RegisterableProgram test2 = teleOp(new Pose(8, 8, Math.toRadians(0)), Alliance.RED, HiveTargetSide.Right)
            .withName("TeleOp Manual")
            .withGroup("Competition");
    public static double fixedIntakePower = 1;
    public static double fixedTransferPower = 1;

    public static MercurialFTC.RegisterableProgram teleOp(Pose startPose, Alliance alliance, HiveTargetSide startSide) {

        return MercurialFTC.teleop(ctx -> {
                    Telemetry.Line instructions1 = ctx.telemetry().addLine(
                            "Left bumper hold: ball shooting");
                    Telemetry.Line instructions2 = ctx.telemetry().addLine(
                            "Right bumper hold: intake");
                    Telemetry.Line instructions3 = ctx.telemetry().addLine(
                            "Right trigger hold: reverse intake");
                    Telemetry.Line instructions4 = ctx.telemetry().addLine(
                            "None: Stop Intake/Transfer/Gate");

                    Telemetry.Line instructions5 = ctx.telemetry().addLine(
                            "Left trigger toggle: Flower");
                    ctx.telemetry().addLine("-------------------------------");

                    Drivetrain drivetrain = new Drivetrain(ctx);
                    VoltageMonitor voltageMonitor = new VoltageMonitor(ctx);
                    MercurialLocalizer localizer = new MercurialLocalizer(Constants.createLocalizer(ctx.hardwareMap()));
                    localizer.setPose(startPose);
                    Turret turret = new Turret(ctx, true);
                    TurretEncoder turretEncoder = new TurretEncoder(ctx);
                    Shooter shooter = new Shooter(ctx);
                    Intake intake = new Intake(ctx);
                    Transfer transfer = new Transfer(ctx);
                    TransferGate transferGate = new TransferGate(ctx);
                    TransferSensors transferSensors = new TransferSensors(ctx);
                    IntakeBallSensors intakeSensors = new IntakeBallSensors(ctx);
                    BallCounter ballCounter = new BallCounter(intake, transfer);
                    FlowerWedge flowerWedge = new FlowerWedge(ctx);
                    HiveSideHelper hiveSideHelper = new HiveSideHelper(startSide);
                    BulkCacheDaemon bulkCacheDaemon = new BulkCacheDaemon(ctx);

                    ctx.telemetry().addLine("-------------------------------");
                    Telemetry.Item transfer1DetectedTelemetry = ctx.telemetry().addData(
                            "Transfer1", "unknown");
                    Telemetry.Item transfer2DetectedTelemetry = ctx.telemetry().addData(
                            "Transfer2", "unknown"
                    );
                    Telemetry.Item intake1DetectedTelemetry = ctx.telemetry().addData(
                            "Intake1", "unknown"
                    );
                    Telemetry.Item intake2DetectedTelemetry  = ctx.telemetry().addData(
                            "Intake2", "unknown"
                    );
                    ctx.telemetry().addLine("-------------------------------");
                    Telemetry.Item ballCounterTelemetry = ctx.telemetry().addData(
                            "BallCounter/Balls", "waiting");
                    Telemetry.Item poseTelemetry = ctx.telemetry().addData(
                            "Pose", "waiting"
                    );
                    Telemetry.Item bulkCacheTelemetry= ctx.telemetry().addData(
                            "Bulk Cached", "unknown"
                    );
                    Telemetry.Item allianceTelemetry = ctx.telemetry().addData(
                            "Alliance", alliance
                    );
                    Telemetry.Item hiveTargetSideTelemetry = ctx.telemetry().addData(
                            "Hive Target Side", "unknown"
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
                    Telemetry.Item pollDurationTelemetry = ctx.telemetry().addData(
                            "Loop Times (sec)", ctx.scheduler().getPollDurationSeconds()
                    );
                    Telemetry.Item averageFiberTimeTelemetry = ctx.telemetry().addData(
                            "Avg Fiber time (sec)", ctx.scheduler().getAverageFiberDurationSeconds()
                    );

                    ctx.telemetry().addLine("-------------------------------");

                    shooter.setHeadingChannel(turret.getTargetHeadingsChannel());

                    bulkCacheDaemon.addHandler(event -> {
                        bulkCacheTelemetry.setValue(event);
                        return EventManager.EventHandled.ok();
                    });

                    localizer.addHandler(event -> {
                        drivetrain.sendMotionState(event);
                        poseTelemetry.setValue(event.pose());
                        shooter.sendMotionState(event);
                        return EventManager.EventHandled.ok();
                    });


                    //send  pose to shooter state machine
                    voltageMonitor.addHandler(EventManager.forwardTo(
                            shooter,
                            shooter::updateVoltage
                    ));

                    transferSensors.addHandler(event -> {
                        if(event.sensorNum == 1){
                            transfer.updateBreaker1(event.interrupted);
                            intake.updateTransfer1(event.interrupted);
                            transfer1DetectedTelemetry.setValue(event.interrupted);
                        } else{
                            transfer.updateBreaker2(event.interrupted);
                            intake.updateTransfer2(event.interrupted);
                            transfer2DetectedTelemetry.setValue(event.interrupted);
                        }
                        ballCounter.updateCount();
                        ballCounterTelemetry.setValue(ballCounter.totalBalls());
                        return EventManager.EventHandled.ok();
                    });

                    intakeSensors.addHandler(event -> {
                        if(event.sensorNum == 1){
                            intake.updateIntake1(event.interrupted);
                            intake1DetectedTelemetry.setValue(event.interrupted);
                        } else{
                            intake.updateIntake2(event.interrupted);
                            intake2DetectedTelemetry.setValue(event.interrupted);
                        }
                        ballCounter.updateCount();
                        ballCounterTelemetry.setValue(ballCounter.totalBalls());
                        return EventManager.EventHandled.ok();
                    });


                    hiveSideHelper.addHandler(event -> {
                        hiveTargetSideTelemetry.setValue(event);
//                        shooter.lutStatic(alliance, hiveSideHelper.getSide());
                        return EventManager.EventHandled.ok();
                    });

                    class DriveSettings {
                        boolean fieldCentric = true;
                    }

                    DriveSettings driveSettings = new DriveSettings();
                    EventManager.Handler<GamepadD.Delta> drivetrainHandler =
                            GamepadD.gamepad1Handler(event -> {
                                if (event.dpadDownWasPressed()) {
                                    driveSettings.fieldCentric = !driveSettings.fieldCentric;
                                    if (driveSettings.fieldCentric) drivetrain.fieldCentric(alliance);
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
                                    if(event.leftBumperWasReleased() || event.rightBumperWasReleased() || event.rightTriggerPassedUnder(0.2)){
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
                                if (flowerWedge.isDown) flowerWedge.setUp();
                                else flowerWedge.setDown();
                            }
                            return EventManager.EventHandled.ok();
                        }
                    );
                    EventManager.Handler<GamepadD.Delta> shootingSideHandler =
                            GamepadD.gamepad2Handler(event -> {
                                        if(event.leftBumperWasPressed()){
                                            hiveSideHelper.setSide(HiveTargetSide.Left);
                                        } else if(event.rightBumperWasPressed()){
                                            hiveSideHelper.setSide(HiveTargetSide.Right);
                                        }
                                        return EventManager.EventHandled.ok();
                                    }
                            );


            //reset turret start fiber
                    expression((scope, self) -> {
                        scope.run(waitUntil(ctx::inInit));
                        scope.run(waitFor.bind(seconds(2)));//waits for 2 seconds
                        exec(() -> turretEncoder.zeroIncremental());//do this to avoid hitting maximum values
                        exec(() -> turretEncoder.resetFromAbsolute());
                        //resets incremental encoder on turret, reads from absolute
                        return noop;
                    }).spawnable().spawnLink();

                    ctx.waitForStart();
                    //remove telemetry

                    ctx.telemetry().removeLine(instructions1);
                    ctx.telemetry().removeLine(instructions2);
                    ctx.telemetry().removeLine(instructions3);
                    ctx.telemetry().removeLine(instructions4);
                    ctx.telemetry().removeLine(instructions5);


                    drivetrain.fieldCentric(alliance);
                    shooter.setUseDefaultVelocity(false);
                    shooter.lutStatic(alliance, startSide);
                    turret.setTrackingMode(false);


                    ctx.gamepadD().addHandler(intakeTransferHandler);
                    ctx.gamepadD().addHandler(drivetrainHandler);
                    ctx.gamepadD().addHandler(flowerMechHandler);
                    ctx.gamepadD().addHandler(shootingSideHandler);

                    //handles the shooter default velocity stuff
                   loop(
                            ifThen(
                                    () -> ballCounter.totalBalls() > 1,
                                exec(() -> shooter.setUseDefaultVelocity(false)),
                                ifThen(
                                        () -> transferGate.openPosition,
                                        exec(()-> shooter.setUseDefaultVelocity(false)),
                                        exec(()-> shooter.setUseDefaultVelocity(true))
                                )
                            )
                    ).spawnable().spawnLink();

                    //update telemetry
                    loop(
                            sequence(
                                    exec(() -> distanceFromGoal.setValue(shooter.distanceFromGoal)),
                                    exec(()-> shooterVelocity.setValue(shooter.getVelocity())),
                                    exec(()-> shooterTargetVelocity.setValue(shooter.getShootingTargetVelocity())),
                                    exec(()-> pollDurationTelemetry.setValue(ctx.scheduler().getPollDurationSeconds())),
                                    exec(()-> averageFiberTimeTelemetry.setValue(ctx.scheduler().getAverageFiberDurationSeconds())),
                                    exec(() -> FtcDashboard.getInstance().getTelemetry().addData("shooter vel", shooter.getVelocity())),
                                    exec(() -> FtcDashboard.getInstance().getTelemetry().update()),
                                    waitFor.bind(seconds(0.25))
                            )

                    ).spawnable().spawnLink();


                    ctx.dropToScheduler();
                    return null;

                }
        );
    }


}
