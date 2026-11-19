package org.firstinspires.ftc.teamcode.subsystemTesting;

import static dev.frozenmilk.dairy.mercurial.continuations.Continuations.exec;
import static dev.frozenmilk.dairy.mercurial.continuations.Continuations.expression;
import static dev.frozenmilk.dairy.mercurial.continuations.Continuations.loop;
import static dev.frozenmilk.dairy.mercurial.continuations.Continuations.noop;
import static dev.frozenmilk.dairy.mercurial.continuations.Continuations.seconds;
import static dev.frozenmilk.dairy.mercurial.continuations.Continuations.waitFor;
import static dev.frozenmilk.dairy.mercurial.continuations.Continuations.waitUntil;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.pedropathing.math.Pose;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.game.ALLIANCE;
import org.firstinspires.ftc.teamcode.game.HiveTargetSide;
import org.firstinspires.ftc.teamcode.helper.AngleHelper;
import org.firstinspires.ftc.teamcode.sensor.TurretEncoder;
import org.firstinspires.ftc.teamcode.subsystems.FlowerWedge;
import org.firstinspires.ftc.teamcode.subsystems.Shooter;
import org.firstinspires.ftc.teamcode.subsystems.Turret;
import org.firstinspires.ftc.teamcode.util.Angle;

import dev.frozenmilk.dairy.mercurial.ftc.GamepadD;
import dev.frozenmilk.dairy.mercurial.ftc.MercurialFTC;
import dev.frozenmilk.dairy.mercurial.processes.Channel;
import dev.frozenmilk.dairy.mercurial.processes.EventManager;
import dev.frozenmilk.dairy.mercurial.processes.Fiber;
import dev.frozenmilk.dairy.mercurial.processes.Spawnable;


@Config
//seems to be bugging -> noisy readings seems that Mercurial not updating constantly
public class TurretEncoderTest {

    public static final MercurialFTC.RegisterableProgram TUTest = teleOp()
            .withName("Turret Encoder Testing")
            .withGroup("Subsystem Testing");

    public static MercurialFTC.RegisterableProgram teleOp() {
        return MercurialFTC.teleop(ctx -> {
            TurretEncoder turretEncoder = new TurretEncoder(ctx);
            Telemetry.Item absoluteRawEncoderPos = ctx.telemetry().addData("Absolute Raw: ", "unknown");
            Telemetry.Item incrmentalRawEncoderPos = ctx.telemetry().addData("Incremental Raw: ", "unknown");
            turretEncoder.addHandler(event -> {
                if(event.getAngleType() == AngleHelper.AngleType.Absolute){
                    absoluteRawEncoderPos.setValue(event.getAngle().toDegrees());
                }
                else if(event.getAngleType() == AngleHelper.AngleType.Incremental){
                    incrmentalRawEncoderPos.setValue(event.getAngle().toDegrees());
                }

                return EventManager.EventHandled.ok();
            });



            ctx.waitForStart();
            ctx.dropToScheduler();
            return null;

        });
    }




}
