package org.firstinspires.ftc.teamcode.subsystemTesting.shooter;
import com.acmerobotics.dashboard.config.Config;
import org.firstinspires.ftc.teamcode.subsystems.Shooter;
import dev.frozenmilk.dairy.mercurial.ftc.GamepadD;
import dev.frozenmilk.dairy.mercurial.ftc.MercurialFTC;
import dev.frozenmilk.dairy.mercurial.processes.EventManager;

@Config
public class ShooterTesting {

    public static final MercurialFTC.RegisterableProgram ShooterTesting = teleOp()
            .withName("Shooter Testing")
            .withGroup("Subsystem Testing");

    public static double shootingVelocity = 1200;
    public static double fixedPower = 0.5;


    //plug bounds in flower wedge state machine
    public static MercurialFTC.RegisterableProgram teleOp() {
        return MercurialFTC.teleop(ctx -> {
            Shooter shooter = new Shooter(ctx);


            EventManager.Handler<GamepadD.Delta> shooterHandler =
                    GamepadD.gamepad1Handler(event -> {
                            if(event.leftBumperWasPressed()){
                                shooter.fixedVelocity(shootingVelocity, false);
                            } else if(event.rightBumperWasPressed()){
                                shooter.fixedPower(fixedPower);
                            }

                            return EventManager.EventHandled.ok();
                        }
                    );

            ctx.waitForStart();
            ctx.gamepadD().addHandler(shooterHandler);
            ctx.dropToScheduler();
            return null;

        });
    }




}
