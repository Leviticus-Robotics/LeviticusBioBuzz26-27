package org.firstinspires.ftc.teamcode.subsystemTesting.shooter;

import static dev.frozenmilk.dairy.mercurial.continuations.Continuations.loop;
import static dev.frozenmilk.dairy.mercurial.continuations.Continuations.seconds;
import static dev.frozenmilk.dairy.mercurial.continuations.Continuations.waitUntil;

import com.acmerobotics.dashboard.config.Config;

import org.firstinspires.ftc.teamcode.subsystems.Shooter;

import dev.frozenmilk.dairy.mercurial.ftc.GamepadD;
import dev.frozenmilk.dairy.mercurial.ftc.MercurialFTC;
import dev.frozenmilk.dairy.mercurial.processes.EventManager;


@Config
public class HoodSetting {

    public static final MercurialFTC.RegisterableProgram HoodSetting = teleOp()
            .withName("Hood Testing")
            .withGroup("Subsystem Testing");

    public static double targetHoodPosition = 0.4;
    public static MercurialFTC.RegisterableProgram teleOp() {
        return MercurialFTC.teleop(ctx -> {
            Shooter shooter = new Shooter(ctx);

            EventManager.Handler<GamepadD.Delta> hoodHandler =
                    GamepadD.gamepad1Handler(event -> {
                                if(event.leftBumperWasPressed()) {
                                    shooter.setHood(targetHoodPosition);
                                }
                                return EventManager.EventHandled.ok();
                            }
                    );

            ctx.waitForStart();
            ctx.gamepadD().addHandler(hoodHandler);
            ctx.dropToScheduler();
            return null;

        });
    }




}
