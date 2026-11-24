package org.firstinspires.ftc.teamcode.subsystemTesting.flower;

import static dev.frozenmilk.dairy.mercurial.continuations.Continuations.expression;
import static dev.frozenmilk.dairy.mercurial.continuations.Continuations.loop;
import static dev.frozenmilk.dairy.mercurial.continuations.Continuations.noop;
import static dev.frozenmilk.dairy.mercurial.continuations.Continuations.seconds;
import static dev.frozenmilk.dairy.mercurial.continuations.Continuations.waitUntil;

import com.acmerobotics.dashboard.config.Config;

import org.firstinspires.ftc.teamcode.subsystems.FlowerWedge;

import dev.frozenmilk.dairy.mercurial.ftc.GamepadD;
import dev.frozenmilk.dairy.mercurial.ftc.MercurialFTC;
import dev.frozenmilk.dairy.mercurial.processes.EventManager;


@Config
public class FlowerWedgeTuning {

    public static final MercurialFTC.RegisterableProgram FlowerTesting = teleOp()
            .withName("Flower Positioning")
            .withGroup("Subsystem Testing");
    public static double pos = 0.1;

    //plug bounds in flower wedge state machine
    public static MercurialFTC.RegisterableProgram teleOp() {
        return MercurialFTC.teleop(ctx -> {
           FlowerWedge flowerWedge = new FlowerWedge(ctx);

           loop(
                expression((scope, _self) -> {
                    scope.runExec(() -> {
                        flowerWedge.setCustomPos(pos);
                    });
                    return noop;
                })
            ).spawnable().spawnLink();
            ctx.gamepadD().addHandler(GamepadD.gamepad1Handler(gamepad -> {
                if(gamepad.dpadUpWasPressed()) {
                    pos += 0.1;
                } else if (gamepad.dpadDownWasPressed()) {
                    pos -= 0.1;
                } else if (gamepad.dpadLeftWasPressed()) {
                    pos = FlowerWedge.UP_POSITION;
                } else if (gamepad.dpadRightWasPressed()) {
                    pos = FlowerWedge.DOWN_POSITION;
                }
                return EventManager.EventHandled.ok();
            }));
            ctx.waitForStart();
            ctx.dropToScheduler();
            return null;

        });
    }




}
