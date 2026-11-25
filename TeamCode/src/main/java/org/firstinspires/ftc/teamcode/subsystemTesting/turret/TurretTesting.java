package org.firstinspires.ftc.teamcode.subsystemTesting.turret;

import static dev.frozenmilk.dairy.mercurial.continuations.Continuations.exec;
import static dev.frozenmilk.dairy.mercurial.continuations.Continuations.loop;
import static dev.frozenmilk.dairy.mercurial.continuations.Continuations.seconds;
import static dev.frozenmilk.dairy.mercurial.continuations.Continuations.waitUntil;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.hardware.lynx.LynxModule;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.helper.AngleHelper;
import org.firstinspires.ftc.teamcode.sensor.TurretEncoder;
import org.firstinspires.ftc.teamcode.subsystems.BulkCacheDaemon;
import org.firstinspires.ftc.teamcode.subsystems.Turret;
import org.firstinspires.ftc.teamcode.util.Angle;

import java.util.List;

import dev.frozenmilk.dairy.mercurial.ftc.GamepadD;
import dev.frozenmilk.dairy.mercurial.ftc.MercurialFTC;
import dev.frozenmilk.dairy.mercurial.processes.EventManager;


@Config
public class TurretTesting {

    public static final MercurialFTC.RegisterableProgram TurretTesting = teleOp()
            .withName("Turret Testing")
            .withGroup("Subsystem Testing");

    public static Angle testAngle1 = Angle.fromDegrees(-135);
    public static Angle testAngle2 = Angle.fromDegrees(-90);
    public static Angle testAngle3 = Angle.fromDegrees(135);
    public static Angle testAngle4 = Angle.fromDegrees(90);
    public static Angle testAngle5 = Angle.fromDegrees(180);
    public static double testPosition = 0;

    //plug bounds in flower wedge state machine
    public static MercurialFTC.RegisterableProgram teleOp() {
        return MercurialFTC.teleop(ctx -> {
            Turret turret = new Turret(ctx, true);
            TurretEncoder turretEncoder = new TurretEncoder(ctx);
            BulkCacheDaemon bulkCacheDaemon = new BulkCacheDaemon(ctx);
            Telemetry.Item absoluteRawEncoderPos = ctx.telemetry().addData("Absolute Raw: ", "unknown");
            Telemetry.Item incrementalRawEncoderPos = ctx.telemetry().addData("Incremental Raw: ", "unknown");
            Telemetry.Item absoluteCorrectEncoder = ctx.telemetry().addData("Absolute Correct: ", "unknown");
            Telemetry.Item incrementalCorrectEncoder = ctx.telemetry().addData("Incremental Correct: ", "unknown");
            Telemetry.Item servoSetPosition = ctx.telemetry().addData("Servo Positions: ", "unknown");


            turretEncoder.addHandler(event -> {
                if(event.getAngleType() == AngleHelper.AngleType.Absolute){
                    turret.updateAbsoluteAngle(event.getAngle());
                    absoluteRawEncoderPos.setValue(event.getAngle().toDegrees());
                }
                else{
                    turret.updateIncrementalAngle(event.getAngle());
                    incrementalRawEncoderPos.setValue(event.getAngle().toDegrees());
                }

                return EventManager.EventHandled.ok();
            });



            EventManager.Handler<GamepadD.Delta> turretHandler =
                    GamepadD.gamepad1Handler(event -> {
                        if(event.leftBumperWasPressed()){
                            turret.setRawPosition(0, true);
                        }
                        if(event.rightBumperWasPressed()){
                            turret.setRawPosition(1, true);
                        }
                        if(event.rightTriggerOver(0.3)){
                            turret.setRawPosition(testPosition, true);
                        }

                        if(event.dpadUpWasPressed()){
                            turret.setRawAngle(Angle.fromRadians(testAngle1.toRadians()), false);
                        }
                        else if(event.dpadDownWasPressed()){
                            turret.setRawAngle(Angle.fromRadians(testAngle2.toRadians()), false);
                        }
                        else if(event.dpadLeftWasPressed()){
                            turret.setRawAngle(Angle.fromRadians(testAngle3.toRadians()), false);
                        }
                        else if(event.dpadRightWasPressed()){
                            turret.setRawAngle(Angle.fromRadians(testAngle4.toRadians()), false);
                        }
                        else if(event.triangleWasPressed()){
                            turret.setRawAngle(Angle.fromRadians(testAngle5.toRadians()), false);
                        }
                        else if(event.crossWasPressed()){
                            turret.setRawPosition(Turret.SERVO_CENTER_POSITION, false);
                        }
//                        else if(event.dpadDownWasPressed()){
//                            testAngle.toRadians();
//                            turret.setOptimizedAngle(testAngle, true);
//                        }

                        if(event.squareWasPressed()){
                            turretEncoder.zeroIncremental();
                        }
                        if(event.circleWasPressed()){
                            turretEncoder.resetFromAbsolute();
                        }

                        return EventManager.EventHandled.ok();
                    }
            );

            ctx.waitForStart();

            loop(
                    exec(() -> {
                        absoluteCorrectEncoder.setValue(turret.currentAbsolutePosition().toDegrees());
                        incrementalCorrectEncoder.setValue(turret.currentIncrementalPosition().toDegrees());
                        servoSetPosition.setValue(turret.servoLeft.get());

                    })
            ).spawnable().spawnLink();

            ctx.gamepadD().addHandler(turretHandler);
            ctx.dropToScheduler();
            return null;

        });
    }




}
