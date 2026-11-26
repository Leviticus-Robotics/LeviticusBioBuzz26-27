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
import static org.firstinspires.ftc.teamcode.constants.Constants.*;

import android.hardware.Sensor;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.pedropathing.math.Pose;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.constants.Constants;
import org.firstinspires.ftc.teamcode.game.Alliance;
import org.firstinspires.ftc.teamcode.game.HiveTargetSide;
import org.firstinspires.ftc.teamcode.helper.AngleHelper;
import org.firstinspires.ftc.teamcode.helper.BallCounter;
import org.firstinspires.ftc.teamcode.helper.SensorEvent;
import org.firstinspires.ftc.teamcode.localization.mercurial.MercurialLocalizer;
import org.firstinspires.ftc.teamcode.sensor.HiveSideHelper;
import org.firstinspires.ftc.teamcode.sensor.IntakeBallSensors;
import org.firstinspires.ftc.teamcode.sensor.TransferSensors;
import org.firstinspires.ftc.teamcode.sensor.TurretEncoder;
import org.firstinspires.ftc.teamcode.sensor.VoltageMonitor;
import org.firstinspires.ftc.teamcode.subsystems.BulkCacheDaemon;
import org.firstinspires.ftc.teamcode.subsystems.Drivetrain;
import org.firstinspires.ftc.teamcode.subsystems.FlowerWedge;
import org.firstinspires.ftc.teamcode.subsystems.GobildaLightOutputs;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.Shooter;
import org.firstinspires.ftc.teamcode.subsystems.Transfer;
import org.firstinspires.ftc.teamcode.subsystems.TransferGate;
import org.firstinspires.ftc.teamcode.subsystems.Turret;
import org.firstinspires.ftc.teamcode.util.Angle;

import dev.frozenmilk.dairy.mercurial.ftc.GamepadD;
import dev.frozenmilk.dairy.mercurial.ftc.MercurialFTC;
import dev.frozenmilk.dairy.mercurial.processes.EventManager;


@Config
public class TeleOpManual {
    public static final MercurialFTC.RegisterableProgram red1 = teleOp(new Pose(CENTER_TO_BACK_INCH, CENTER_TO_SIDE_INCH, Math.toRadians(0)), Alliance.RED, HiveTargetSide.Right)
            .withName("TeleOp Red Manual")
            .withGroup("Competition");
    public static final MercurialFTC.RegisterableProgram blue1 = teleOp(new Pose(GAME_FIELD_LENGTH_INCH - CENTER_TO_BACK_INCH, CENTER_TO_SIDE_INCH, Math.PI), Alliance.BLUE, HiveTargetSide.Left)
            .withName("TeleOp Blue Manual")
            .withGroup("Competition");
    public static double fixedIntakePower = 1;
    public static double fixedTransferPower = 0.5;

    public static double transferTargetVelocity = 2000;
    public static double shootingTolerance = 50;//ticks per second
    public static double turretToleranceRad = Math.toRadians(5);
    public static double turningSensitivityMultiplier = 0.6;

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
                    Telemetry.Item readyToStart = ctx.telemetry().addData(
                            "Setup", "not yet"
                    );

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
                    GobildaLightOutputs leds = new GobildaLightOutputs(ctx);

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
                    Telemetry.Item withinShootingToleranceTelemetry = ctx.telemetry().addData(
                            "Shooting/tolerance: ", "unknown"
                    );
                    Telemetry.Item withinTurretToleranceTelemetry = ctx.telemetry().addData(
                            "Turret/tolerance: ", "unknown"
                    );
                    Telemetry.Item validShootPositionTelemetry = ctx.telemetry().addData(
                            "Valid shoot Pos: ", "unknown"
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

                    //send turret angles to turret

                    turretEncoder.addHandler(event -> {
                        if(event.getAngleType() == AngleHelper.AngleType.Absolute){
                            turret.updateAbsoluteAngle(event.getAngle());
                        }
                        else if(event.getAngleType() == AngleHelper.AngleType.Incremental){
                            turret.updateIncrementalAngle(event.getAngle());
                        }
                        return EventManager.EventHandled.ok();
                    });

                    //send  pose to shooter state machine
                    voltageMonitor.addHandler(EventManager.forwardTo(
                            shooter,
                            shooter::updateVoltage
                    ));
                    voltageMonitor.addHandler(EventManager.forwardTo(
                            transfer,
                            transfer::updateVoltage
                    ));
                    EventManager.Handler<SensorEvent> changeLeds= evt -> {
                        ballCounter.updateCount();
                        int numBalls = ballCounter.totalBalls();
                        switch(numBalls){
                            case 4:
                                leds.setSolidColor(GobildaLightOutputs.ColorMode.Green);
                                break;
                            case 3:
                                leds.setSolidColor(GobildaLightOutputs.ColorMode.Orange);
                                break;
                            case 2:
                                leds.setSolidColor(GobildaLightOutputs.ColorMode.Blue);
                                break;
                            case 1:
                                leds.setSolidColor(GobildaLightOutputs.ColorMode.Red);
                                break;
                            default:
                                leds.setSolidColor(GobildaLightOutputs.ColorMode.Black);
                                break;
                        }
                        return EventManager.EventHandled.ok();
                    };
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
                    transferSensors.addHandler(changeLeds);
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
                    intakeSensors.addHandler(changeLeds);

                    hiveSideHelper.addHandler(event -> {
                        hiveTargetSideTelemetry.setValue(event);
                        if(ctx.inLoop()) {
                            shooter.lutStatic(alliance, hiveSideHelper.getSide());
                        }
                        return EventManager.EventHandled.ok();
                    });
                    class AssitedShooting{
                        boolean waiting = false;
                        boolean feeding = false;
                    }
                    class ManualShooting{
                        boolean waiting = false;
                        boolean feeding = false;
                    }

                    class DriveSettings {
                        boolean fieldCentric = true;
                    }
                    AssitedShooting assistedShootingSettings = new AssitedShooting();
                    ManualShooting manualShootingSettings = new ManualShooting();

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
                                        -event.rightStickX()* turningSensitivityMultiplier);
                                return EventManager.EventHandled.ok();
                            });

                    EventManager.Handler<GamepadD.Delta> intakeTransferHandler =
                            GamepadD.gamepad1Handler(event -> {
                                    if(event.leftBumperWasPressed())//shoots immediantly
                                    {   //reset assisted
                                        assistedShootingSettings.waiting = false;
                                        assistedShootingSettings.feeding = false;
                                        manualShootingSettings.waiting = true;
                                        manualShootingSettings.feeding = false;
                                        transferGate.setOpen();
                                        intake.fixedPower(0);
                                        transfer.fixedPower(0);
                                    }
                                    else if(event.rightBumperWasPressed()) {//intake
                                        assistedShootingSettings.waiting = false;
                                        assistedShootingSettings.feeding = false;
                                        manualShootingSettings.waiting = false;
                                        manualShootingSettings.feeding = false;
                                        intake.runPowerTillDetection(fixedIntakePower);
                                        transfer.runVelocityTillDetection(transferTargetVelocity);
                                        transferGate.setClosed();
                                    } else if(event.rightTriggerPassedOver(0.2)){//reverse intake
                                        assistedShootingSettings.waiting = false;
                                        assistedShootingSettings.feeding = false;
                                        manualShootingSettings.waiting = false;
                                        manualShootingSettings.feeding = false;
                                        intake.fixedPower(-1);
                                        transfer.fixedPower(-1);
                                        transferGate.setOpen();
                                    } else if(event.dpadLeftWasPressed()){//assisted shooting: wait for criteria bfore shooting
                                        assistedShootingSettings.waiting = true;
                                        assistedShootingSettings.feeding = false;
                                        manualShootingSettings.waiting = false;
                                        manualShootingSettings.feeding = false;
                                        shooter.setUseDefaultVelocity(false);
                                        transferGate.setClosed();
                                        intake.fixedPower(0);
                                        transfer.fixedPower(0);
                                    }
                                    if(event.leftBumperWasReleased()
                                            || event.rightBumperWasReleased()
                                            || event.rightTriggerPassedUnder(0.2)
                                            || event.dpadLeftWasReleased()
                                    ){
//                                     if(!event.leftBumper() && !event.rightBumper() && !(event.rightTrigger() > 0.2) && !event.dpadLeft()){
                                        assistedShootingSettings.waiting = false;
                                        assistedShootingSettings.feeding = false;
                                        manualShootingSettings.waiting = false;
                                        manualShootingSettings.feeding = false;
                                        intake.fixedPower(0);
                                        transfer.fixedPower(0);
                                        transferGate.setClosed();
                                    }
                                    return EventManager.EventHandled.ok();
                                }
                            );
                    //intake.fixedPower(0);
            //                                        transfer.fixedPower(0);


                    EventManager.Handler<GamepadD.Delta> flowerMechHandler =
                        GamepadD.gamepad1Handler(event -> {
                            if(event.leftTriggerPassedOver(0.5)){
                                if (flowerWedge.isDown) flowerWedge.setUp();
                                else flowerWedge.setDown();
                            }
                            return EventManager.EventHandled.ok();
                        }
                    );

                    EventManager.Handler<GamepadD.Delta> resetPoseHandler =
                            GamepadD.gamepad1Handler(event -> {
                                if(event.squareWasPressed()){
                                    localizer.setPose(alliance == Alliance.BLUE ? BLUE_ALLIANCE_FAR_LEFT : RED_ALLIANCE_FAR_LEFT);
                                } else if(event.triangleWasPressed()){
                                    localizer.setPose(alliance == Alliance.BLUE ? BLUE_ALLIANCE_FAR_RIGHT : RED_ALLIANCE_FAR_RIGHT);
                                } else if(event.crossWasPressed()){
                                    localizer.setPose(alliance == Alliance.BLUE ? BLUE_ALLIANCE_CLOSE_LEFT : RED_ALLIANCE_CLOSE_LEFT);
                                } else if(event.circleWasPressed()){
                                    localizer.setPose(alliance == Alliance.BLUE ? BLUE_ALLIANCE_CLOSE_RIGHT : RED_ALLIANCE_CLOSE_RIGHT);
                                }
                                return EventManager.EventHandled.ok();
                            }
                    );
                    //Gampead 2 bindings below
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

                    EventManager.Handler<GamepadD.Delta> turretTrimhandler =
                            GamepadD.gamepad2Handler(event -> {
                                if(event.dpadRightWasPressed()){
                                    turret.adjustAimTrimDeg(Turret.AIM_TRIM_STEP_DEG);
                                }
                                if(event.dpadLeftWasPressed()){
                                    turret.adjustAimTrimDeg(-Turret.AIM_TRIM_STEP_DEG);
                                }
                                if(event.dpadDownWasPressed()){
                                    turret.resetAimTrim();
                                }
                                return EventManager.EventHandled.ok();
                            }
                    );


                    //reset turret start fiber, init turret mechnaism
                    expression((scope, _self) -> {
                        scope.runExec(() -> turret.setServos(Turret.SERVO_CENTER_POSITION, false));
                        scope.run(waitFor.bind(seconds(2)));
                        scope.runExec(() -> turretEncoder.zeroIncremental());
                        scope.runExec(() -> turretEncoder.resetFromAbsolute());
                        scope.runExec(() -> shooter.setUseDefaultVelocity(false));
                        scope.run(waitFor.bind(seconds(3)));
                        scope.run(turret.setTrackingModeSync(false));
                        scope.runExec(() -> readyToStart.setValue("Ready"));
                        scope.runExec(() -> drivetrain.fieldCentric(alliance));
                        return noop;
                    }).spawnable().spawnLink();

                    ctx.waitForStart();


                    //remove telemetry
                    shooter.lutStatic(alliance, startSide);

                    ctx.telemetry().removeLine(instructions1);
                    ctx.telemetry().removeLine(instructions2);
                    ctx.telemetry().removeLine(instructions3);
                    ctx.telemetry().removeLine(instructions4);
                    ctx.telemetry().removeLine(instructions5);
                    ctx.telemetry().removeItem(readyToStart);






                    ctx.gamepadD().addHandler(intakeTransferHandler);
                    ctx.gamepadD().addHandler(drivetrainHandler);
                    ctx.gamepadD().addHandler(flowerMechHandler);
                    ctx.gamepadD().addHandler(resetPoseHandler);
                    ctx.gamepadD().addHandler(shootingSideHandler);
                    ctx.gamepadD().addHandler(turretTrimhandler);

                    loop(
                            sequence(
                                ifThen(
                                        ()-> localizer.pose().y() >= 141.5 / 2.0 && alliance == Alliance.BLUE
                                                || localizer.pose().y() < 141.5/2.0 && alliance == Alliance.RED,
                                        exec(() -> hiveSideHelper.setSide(HiveTargetSide.Right)),
                                        exec(()-> hiveSideHelper.setSide(HiveTargetSide.Left))
                                ),
                                waitFor.bind(seconds(0.10))
                            )
                    ).spawnable().spawnLink();



                   loop(
                           //handles the shooter default velocity to turn off default if more than 1 ball, doing assisted shooting, or open gate
                           sequence(
                                   ifThen(
                                           () -> assistedShootingSettings.waiting
                                                   || ballCounter.totalBalls() > 1
                                                   || transferGate.openPosition,
                                           exec(()-> shooter.setUseDefaultVelocity(false)),
                                           exec(()-> shooter.setUseDefaultVelocity(true))
                                   ),
                                   ifThen(
                                            ()-> assistedShootingSettings.waiting
                                                && !assistedShootingSettings.feeding
                                                && shooter.isAtShootingTargetVelocity(shootingTolerance)
                                                && turret.isAtAngle(turretToleranceRad)
                                                && shooter.isValidShootPosition(hiveSideHelper.getSide()),
                                            exec(()->{
                                                assistedShootingSettings.feeding = true;
                                               transferGate.setOpen();
                                               intake.fixedPower(1);
                                               transfer.fixedVelocity(transferTargetVelocity);
                                            }
                                   ),
                                   waitFor.bind(seconds(0.10))
                           )
                   )).spawnable().spawnLink();

                   //handles the anual shooting
                    loop(
                            //handles the shooter default velocity to turn off default if more than 1 ball, doing assisted shooting, or open gate
                            ifThen(
                                    ()-> manualShootingSettings.waiting && !manualShootingSettings.feeding,
                                    sequence(
                                            waitFor.bind(seconds(0.2)),
                                            ifThen(
                                                    () -> manualShootingSettings.waiting,
                                                    exec(()-> {
                                                        manualShootingSettings.feeding = true;
                                                        transfer.fixedVelocity(transferTargetVelocity);
                                                        intake.fixedPower(1);
                                                    })
                                            )
                                    )
                            )
                    ).spawnable().spawnLink();


                    //update telemetry
                    loop(
                            sequence(
                                    exec(()-> withinShootingToleranceTelemetry.setValue(shooter.isAtShootingTargetVelocity(shootingTolerance))),
                                    exec(()-> withinTurretToleranceTelemetry.setValue(turret.isAtAngle(turretToleranceRad))),
                                    exec(()-> validShootPositionTelemetry.setValue(shooter.isValidShootPosition(hiveSideHelper.getSide()))),
                                    exec(() -> distanceFromGoal.setValue(shooter.distanceFromGoal)),
                                    exec(()-> shooterVelocity.setValue(shooter.getVelocity())),
                                    exec(()-> shooterTargetVelocity.setValue(shooter.getShootingTargetVelocity())),
                                    exec(()-> pollDurationTelemetry.setValue(ctx.scheduler().getPollDurationSeconds())),
                                    exec(()-> averageFiberTimeTelemetry.setValue(ctx.scheduler().getAverageFiberDurationSeconds())),
                                    exec(() -> FtcDashboard.getInstance().getTelemetry().addData("shooter vel", shooter.getVelocity())),
                                    exec(() -> FtcDashboard.getInstance().getTelemetry().addData("transfer vel", transfer.getVelocity())),
//                                    exec(() -> FtcDashboard.getInstance().getTelemetry().addData("Atan aim angle(deg)", Shooter.angle1Rad)),
//                                    exec(() -> FtcDashboard.getInstance().getTelemetry().addData("DeltaY", Shooter.deltaY)),
//                                    exec(() -> FtcDashboard.getInstance().getTelemetry().addData("DeltaX", Shooter.deltaX)),
//                                    exec(() -> FtcDashboard.getInstance().getTelemetry().addData("turret rel(deg)", turret.targetTurretAngle.toDegrees())),
                                    exec(() -> FtcDashboard.getInstance().getTelemetry().update()),
                                    waitFor.bind(seconds(0.20))
                            )

                    ).spawnable().spawnLink();



                    ctx.dropToScheduler();
                    return null;

                }
        );
    }


}
