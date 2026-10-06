package org.firstinspires.ftc.teamcode.subsystems;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.config.Config;
import com.pedropathing.localization.MotionState;
import com.pedropathing.math.Pose;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.seattlesolvers.solverslib.hardware.motors.Motor;
import com.seattlesolvers.solverslib.util.InterpLUT;


import org.firstinspires.ftc.teamcode.game.SIDE;
import org.firstinspires.ftc.teamcode.hardware.MotorEx;
import org.firstinspires.ftc.teamcode.constants.ConfigNames;
import org.jetbrains.annotations.NotNull;

import dev.frozenmilk.dairy.mercurial.processes.Channel;
import dev.frozenmilk.dairy.mercurial.processes.EventManager;
import dev.frozenmilk.dairy.mercurial.processes.StateMachine;
import org.firstinspires.ftc.teamcode.game.ALLIANCE;

@Config
public class Shooter extends StateMachine {
    public MotorEx shooter;
    Channel<MotionState> motionStates = new Channel.Single<>();
    Channel<Mode> switchModes = new Channel.Single<>();
    Channel<Double> voltageChannel = new Channel.Single<>();
    public static double[] pidCoefficients = new double[] {0.004, 0.0, 0.0};
    public static double[] feedforwardCoefficients = new double[]{0.0291871, 0.000355203, 0};
    public static boolean usePIDCoefficients = true;
    public static boolean useFeedforwardCoefficients = true;
    public static double[] distances =  new double[] {20,   30,   40,   50,   60,   70,  80};
    public static double[] velocities = new double[] {700, 900, 1000, 1050, 1100, 1200, 1250};

    public static boolean DIRECTION_REVERSE = false;

    public static Pose AIM_BLUE_CLOSE = new Pose(61, 55);
    public static Pose AIM_BLUE_FAR = new Pose(61, 67);
    public static Pose AIM_RED_CLOSE = new Pose(141.5 - 61, 55);
    public static Pose AIM_RED_FAR = new Pose(141.5 - 61, 67);
    InterpLUT interpLUT;//distance to velocity interpLUT
    public Shooter(HardwareMap hardwareMap, EventManager<MotionState> localizationEventManager, EventManager<Double> voltageEventManager){
        shooter = new MotorEx(hardwareMap, ConfigNames.shooter);
        shooter.setRunMode(Motor.RunMode.RawPower);
        shooter.setInverted(DIRECTION_REVERSE);
        if(usePIDCoefficients){
            shooter.setVeloCoefficients(pidCoefficients[0], pidCoefficients[1], pidCoefficients[2]);
        }
        if(useFeedforwardCoefficients){
            shooter.setFeedforwardCoefficients(feedforwardCoefficients[0], feedforwardCoefficients[1], feedforwardCoefficients[2]);
        }
        localizationEventManager.addHandler(motionStateForwarder);
        voltageEventManager.addHandler(voltageForwarder);

        interpLUT = new InterpLUT();
        for(int i = 0; i < distances.length; i++){
            interpLUT.add(distances[i], velocities[i]);
        }
        interpLUT.createLUT();
    }

    public Shooter(HardwareMap hardwareMap){
        shooter = new MotorEx(hardwareMap, ConfigNames.shooter);
        shooter.setRunMode(Motor.RunMode.VelocityControl);
        shooter.setInverted(DIRECTION_REVERSE);
        if(usePIDCoefficients){
            shooter.setVeloCoefficients(pidCoefficients[0], pidCoefficients[1], pidCoefficients[2]);
        }
        if(useFeedforwardCoefficients){
            shooter.setFeedforwardCoefficients(feedforwardCoefficients[0], feedforwardCoefficients[1], feedforwardCoefficients[2]);
        }
//        localizationEventManager.addHandler(motionStateForwarder);
//        voltageEventManager.addHandler(voltageForwarder);

        interpLUT = new InterpLUT();
        for(int i = 0; i < distances.length; i++){
            interpLUT.add(distances[i], velocities[i]);
        }
        interpLUT.createLUT();
    }

    public class LUTStatic implements Mode{
        ALLIANCE alliance;
        SIDE side;
        public LUTStatic(ALLIANCE alliance, SIDE side){
            this.alliance = alliance;
            this.side = side;
        }

        @Override
        public void enter(@NotNull Mode previousMode){
            shooter.setRunMode(Motor.RunMode.VelocityControl);
        }
        @NonNull
        @Override
        public Mode eval() {
            MotionState motionState = motionStates.poll();
            Pose robotPose = motionState.pose();
            Pose targetShootPose = getPose(alliance, side);

            double distance = robotPose.distance(targetShootPose);
            double targetVelocity = interpLUT.get(distance);

            double currentVoltage = voltageChannel.poll();
            if(currentVoltage == 0) {
                shooter.set(targetVelocity, currentVoltage);
            } else{
                shooter.set(targetVelocity);
            }

            Mode mode = switchModes.poll();
            if(mode == null){
                return this;
            } else{
                return mode;
            }
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
            if(useVelocity) {
                shooter.setRunMode(Motor.RunMode.VelocityControl);
                shooter.set(value);
            }
            else{
                shooter.setRunMode(Motor.RunMode.RawPower);
                shooter.set(value);
            }

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

    EventManager.Handler<MotionState> motionStateForwarder = motionState -> {
        motionStates.send(motionState);
        if(fiber().status().alive()) {
            return EventManager.EventHandled.ok();
        } else{
            return EventManager.EventHandled.remove();
        }
    };

    EventManager.Handler<Double> voltageForwarder = voltage -> {
        voltageChannel.send(voltage);
        if(fiber().status().alive()){
            return EventManager.EventHandled.ok();
        } else{
            return EventManager.EventHandled.remove();
        }
    };


    public Pose getPose(ALLIANCE alliance, SIDE side){
        if(alliance == ALLIANCE.BLUE){
            return side == SIDE.CLOSE ? AIM_BLUE_CLOSE : AIM_BLUE_FAR;
        } else{
            return side == SIDE.CLOSE ? AIM_RED_CLOSE : AIM_RED_FAR;
        }
    }

    public void idle(){
        switchModes.send(idle);
    }
    public void fixed(double value, boolean useVelocity){
        switchModes.send(new Fixed(value, useVelocity));
    }



    public double getVelocity(){
        return shooter.getVelocity();
    }



}
