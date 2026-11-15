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
import org.firstinspires.ftc.teamcode.sensor.TurretEncoder;
import org.firstinspires.ftc.teamcode.subsystems.FlowerWedge;
import org.firstinspires.ftc.teamcode.subsystems.Turret;

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

    //plug bounds in flower wedge state machine
    public static MercurialFTC.RegisterableProgram teleOp() {
        return MercurialFTC.teleop(ctx -> {
            Turret turret = new Turret(ctx, true);
            TurretEncoder turretEncoder = new TurretEncoder(ctx);
//            turretEncoder.addHandler(EventManager.forwardTo(
//                    turret,
//                    turret::updateCurrentTurretAngle
//            ));

//            EventManager.Handler<GamepadD.Delta> intakeTransferHandler =
//                    GamepadD.gamepad1Handler(event -> {
//                        if(event.dpadLeftWasPressed()){
//
//                        }
//
//                    });

            ctx.waitForStart();
            ctx.dropToScheduler();
            return null;

        });
    }




}
