package org.firstinspires.ftc.teamcode.subsystems;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.seattlesolvers.solverslib.hardware.motors.Motor;
import com.seattlesolvers.solverslib.hardware.motors.MotorEx;

import org.firstinspires.ftc.teamcode.constants.ConfigNames;
import org.jetbrains.annotations.NotNull;

import dev.frozenmilk.dairy.mercurial.processes.Channel;
import dev.frozenmilk.dairy.mercurial.processes.StateMachine;

@Config
public class Intake extends StateMachine {
    MotorEx intake;
    Channel<Mode> switchModes = new Channel.Single();
    public static double[] pidCoefficients = new double[] {1.0, 0.0, 0.0};
    public static double[] feedforwardCoefficients = new double[]{0, 0, 0};
    public static boolean usePIDCoefficients = true;
    public static boolean useFeedforwardCoefficients = true;

    public Intake(HardwareMap hardwareMap){
        intake = new MotorEx(hardwareMap, ConfigNames.intake);
        intake.setRunMode(Motor.RunMode.RawPower);
        intake.setInverted(true);
        if(usePIDCoefficients){
            intake.setVeloCoefficients(pidCoefficients[0], pidCoefficients[1], pidCoefficients[2]);
        }
        if(useFeedforwardCoefficients){
            intake.setFeedforwardCoefficients(feedforwardCoefficients[0], feedforwardCoefficients[1], feedforwardCoefficients[2]);
        }
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
                intake.setRunMode(Motor.RunMode.VelocityControl);
                intake.set(value);
            }
            else{
                intake.setRunMode(Motor.RunMode.RawPower);
                intake.set(value);
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

        @Override
        public String toString(){
            return "Fixed{value =" + value +
                    ",useVelocity=" + useVelocity;
        }
    }
    Mode idle = new Mode() {
        @Override
        public void enter(@NotNull Mode previousMode){
            intake.setRunMode(Motor.RunMode.RawPower);
            intake.set(0);
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

        @Override
        public String toString(){
            return "Idle";
        }
    };

    @NonNull
    @Override
    protected Mode init() {
        return idle;
    }


    public void fixed(double value, boolean useVelocity){
        switchModes.send(new Fixed(value, useVelocity));
    }

    public void idle(){
        switchModes.send(idle);
    }
    public double getVelocity(){
        return intake.getVelocity();
    }
}
