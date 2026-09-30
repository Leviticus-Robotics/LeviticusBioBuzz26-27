package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.pedropathing.math.Pose;

import org.firstinspires.ftc.teamcode.subsystems.Shooter;

import dev.frozenmilk.dairy.mercurial.ftc.GamepadD;
import dev.frozenmilk.dairy.mercurial.ftc.MercurialFTC;
import dev.frozenmilk.dairy.mercurial.pedro.MercurialFollower;
import dev.frozenmilk.dairy.mercurial.processes.EventManager;
import dev.frozenmilk.dairy.mercurial.processes.EventManager.EventHandled;

@Config
public class ShooterTesting{
    final static Pose pose = new Pose(0, 0);
    public static double fixedPower = 0.5;
    public static double velocity = 1500;
    public static MercurialFTC.RegisterableProgram teleOp(Pose pose){
        return MercurialFTC.teleop(ctx -> {
            MercurialFollower follower = Constants.createMercurial(ctx.hardwareMap());
            Shooter shooter = new Shooter(ctx.hardwareMap(), follower.localizationEventManager());


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

            ctx.waitForStart();

            ctx.gamepadD().addHandler(gamepad2Handler);
            ctx.dropToScheduler();
            return null;
        }


        );

    }

    public static final MercurialFTC.RegisterableProgram program = teleOp(new Pose(0, 0));
}
