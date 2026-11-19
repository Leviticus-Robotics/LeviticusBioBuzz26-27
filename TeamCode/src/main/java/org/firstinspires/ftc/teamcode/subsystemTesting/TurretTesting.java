package org.firstinspires.ftc.teamcode.subsystemTesting;

import static dev.frozenmilk.dairy.mercurial.continuations.Continuations.expression;
import static dev.frozenmilk.dairy.mercurial.continuations.Continuations.loop;
import static dev.frozenmilk.dairy.mercurial.continuations.Continuations.noop;
import static dev.frozenmilk.dairy.mercurial.continuations.Continuations.seconds;
import static dev.frozenmilk.dairy.mercurial.continuations.Continuations.waitFor;
import static dev.frozenmilk.dairy.mercurial.continuations.Continuations.waitUntil;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.pedropathing.math.Pose;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.game.ALLIANCE;
import org.firstinspires.ftc.teamcode.game.HiveTargetSide;
import org.firstinspires.ftc.teamcode.helper.AngleHelper;
import org.firstinspires.ftc.teamcode.sensor.TurretEncoder;
import org.firstinspires.ftc.teamcode.subsystems.FlowerWedge;
import org.firstinspires.ftc.teamcode.subsystems.Turret;
import org.firstinspires.ftc.teamcode.util.Angle;

import dev.frozenmilk.dairy.mercurial.ftc.GamepadD;
import dev.frozenmilk.dairy.mercurial.ftc.MercurialFTC;
import dev.frozenmilk.dairy.mercurial.processes.EventManager;
import dev.frozenmilk.dairy.mercurial.processes.Fiber;
import dev.frozenmilk.dairy.mercurial.processes.Spawnable;


@Config
public class TurretTesting {

    public static final MercurialFTC.RegisterableProgram TurretTesting = teleOp()
            .withName("Turret Testing")
            .withGroup("Subsystem Testing");

    public static Angle testAngle = Angle.fromDegrees(180);
    public static double testPosition = 0.5;
    //plug bounds in flower wedge state machine
    public static MercurialFTC.RegisterableProgram teleOp() {
        return MercurialFTC.teleop(ctx -> {
            Turret turret = new Turret(ctx, true);
            TurretEncoder turretEncoder = new TurretEncoder(ctx);
            Telemetry.Item absoluteRawEncoderPos = ctx.telemetry().addData("Absolute Raw: ", "unknown");
            Telemetry.Item incrmentalRawEncoderPos = ctx.telemetry().addData("Incremental Raw: ", "unknown");

//            expression((scope, self) -> {
//                scope.run(waitUntil(ctx::inInit));
//                scope.run(waitFor.bind(seconds(2)));//waits for 2 seconds
//                scope.run(turretEncoder.resetEncoderSync());//do this to avoid hitting maximum values
//                scope.run(turretEncoder.resetFromAbsoluteSync());
//                //resets incremental encoder on turret, reads from absolute
//                return noop;
//            }).spawnable().spawnLink();

            turretEncoder.addHandler(event -> {
                if(event.getAngleType() == AngleHelper.AngleType.Absolute){
                    turret.updateAbsoluteAngle(event.getAngle());
                    absoluteRawEncoderPos.setValue(event.getAngle().toDegrees());
                }
                if(event.getAngleType() == AngleHelper.AngleType.Incremental){
                    turret.updateIncrementalAngle(event.getAngle());
                    incrmentalRawEncoderPos.setValue(event.getAngle().toDegrees());
                }

                return EventManager.EventHandled.ok();
            });
//            ctx.telemetry().addLine("Left Bumper to set Intake Test Angle");


            EventManager.Handler<GamepadD.Delta> turretHandler =
                    GamepadD.gamepad1Handler(event -> {
                        if(event.leftBumperWasPressed()){
                            turret.setRawPosition(0);
                        } else if(event.rightBumperWasPressed()){
                            turret.setRawPosition(1);
                        } else if(event.rightTriggerOver(0.3)){
                            turret.setRawPosition(testPosition);
                        }

                        if(event.dpadUpWasPressed()){
                            testAngle.toRadians();
                            turret.setRawAngle(testAngle);
                        } else if(event.dpadDownWasPressed()){
                            testAngle.toRadians();
                            turret.setOptimizedAngle(testAngle);
                        }

                        return EventManager.EventHandled.ok();
                    }
            );

            ctx.waitForStart();
            ctx.gamepadD().addHandler(turretHandler);
            ctx.dropToScheduler();
            return null;

        });
    }




}
