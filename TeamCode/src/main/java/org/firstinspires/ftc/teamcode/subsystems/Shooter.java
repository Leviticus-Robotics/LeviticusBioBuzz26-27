package org.firstinspires.ftc.teamcode.subsystems;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.config.Config;
import com.pedropathing.localization.MotionState;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;


import org.firstinspires.ftc.teamcode.hardware.Motor;
import org.firstinspires.ftc.teamcode.hardware.MotorEx;
import org.firstinspires.ftc.teamcode.util.ConfigNames;
import org.jetbrains.annotations.NotNull;

import dev.frozenmilk.dairy.mercurial.processes.Channel;
import dev.frozenmilk.dairy.mercurial.processes.EventManager;
import dev.frozenmilk.dairy.mercurial.processes.StateMachine;

@Config
public class Shooter extends StateMachine {
    MotorEx shooter;
    Channel<MotionState> motionStates = new Channel.Single<>();
    Channel<Mode> switchModes = new Channel.Single<>();
    public static double[] pidCoefficients = new double[] {1.0, 0.0, 0.0};
    public static double[] feedforwardCoefficients = new double[]{0, 0, 0};
    public static boolean usePIDCoefficients = true;
    public static boolean useFeedforwardCoefficients = true;

    public Shooter(HardwareMap hardwareMap, EventManager<MotionState> localizationEventManager){
        shooter = new MotorEx(hardwareMap, ConfigNames.shooter);
        shooter.setRunMode(Motor.RunMode.RawPower);
        if(usePIDCoefficients){
            shooter.setVeloCoefficients(pidCoefficients[0], pidCoefficients[1], pidCoefficients[2]);
        }
        if(useFeedforwardCoefficients){
            shooter.setFeedforwardCoefficients(feedforwardCoefficients[0], feedforwardCoefficients[1], feedforwardCoefficients[2]);
        }
        localizationEventManager.addHandler(motionStateForwarder);
    }

    public class Fixed implements Mode{
        double value;
        boolean useVelocity;
        //value either 0 - 1 for motor power or in velocity
        public Fixed(double value, boolean useVelocity){
            this.value = value;
            this.useVelocity = useVelocity;
        }

        @Override
        public void enter(@NotNull Mode previousMode){
             if(useVelocity) {
                 shooter.setRunMode(Motor.RunMode.VelocityControl);
                 shooter.set(value);
             }
             else{
                 shooter.setRunMode(Motor.RunMode.RawPower);
                 shooter.set(value);
             }

        }

        @NonNull
        @Override
        public Mode eval() {
            Mode nextMode = switchModes.poll();
            if(nextMode == null){
                return this;
            } else{
                return nextMode;
            }
        }
    }

    Mode idle = new Mode() {
        @Override
        public void enter(@NotNull Mode previousMode){
            shooter.setRunMode(Motor.RunMode.RawPower);
            shooter.set(0);
        }
        @NonNull
        @Override
        public Mode eval() {
            Mode nextMode = switchModes.poll();
            if(nextMode == null){
                return this;
            } else{
                return nextMode;
            }
        }
    };
    @NonNull
    @Override
    protected Mode init() {
        return idle;
    }

    EventManager.Handler<MotionState> motionStateForwarder = motionState -> {
        motionStates.send(motionState);
        if(fiber().status().alive()) {
            return EventManager.EventHandled.ok();
        } else{
            return EventManager.EventHandled.remove();
        }
    };


    public void idle(){
        switchModes.send(idle);
    }
    public void fixed(double value, boolean useVelocity){
        switchModes.send(new Fixed(value, useVelocity));
    }
}
