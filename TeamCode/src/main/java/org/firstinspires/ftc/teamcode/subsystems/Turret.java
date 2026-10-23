package org.firstinspires.ftc.teamcode.subsystems;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.config.Config;
import com.pedropathing.revhub.localizers.Encoder;
import com.pedropathing.utils.Angle;
import com.qualcomm.hardware.digitalchickenlabs.OctoQuad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.seattlesolvers.solverslib.hardware.motors.Motor;
import com.seattlesolvers.solverslib.hardware.servos.ServoEx;
import com.seattlesolvers.solverslib.hardware.servos.ServoExGroup;

import org.firstinspires.ftc.teamcode.constants.ConfigNames;
import org.jetbrains.annotations.NotNull;

import java.util.ArrayList;
import java.util.List;

import dev.frozenmilk.dairy.mercurial.processes.Channel;
import dev.frozenmilk.dairy.mercurial.processes.StateMachine;

@Config
public class Turret extends StateMachine {
    //for now modeling as positional servos with same value, can change one or two to continuous later
    Channel<Mode> switchModes = new Channel.Single<>();
    ServoEx servo1;
    ServoEx servo2;
    ServoEx servo3;
    //Encoder encoder;
    ServoExGroup servoGroup;
    OctoQuad octoquad;
    private int encoderPosition = 0;
    private int encoderPort = 2;
    final int TICKS_ENCODER = 8182;//throughbore encoder ticks per revolution
    List<Double> prevServoSetPosition;
    private double startingPosition = 0.5;

    public Turret(HardwareMap hardwareMap){
        servo1 = new ServoEx(hardwareMap, ConfigNames.turretServo1);
        servo2 = new ServoEx(hardwareMap, ConfigNames.turretServo2);
        servo3 = new ServoEx(hardwareMap, ConfigNames.turretServo3);

        servoGroup = new ServoExGroup(servo1, servo2, servo3);
        octoquad = hardwareMap.get(OctoQuad.class, ConfigNames.OCTOQUAD);
        octoquad.setChannelBankConfig(OctoQuad.ChannelBankConfig.ALL_PULSE_WIDTH);
        updateEncoder();
    }
    public Turret(HardwareMap hardwareMap, double startingPosition){
        this(hardwareMap);
        this.startingPosition = startingPosition;
    }

    public int updateEncoder(){
        encoderPosition = octoquad.readSinglePosition(encoderPort);
        encoderPosition /= TICKS_ENCODER;
        return encoderPosition;
    }

    public List<Double> getPreviousSetPosition(){
        prevServoSetPosition = servoGroup.getPositions();
        return prevServoSetPosition;
    }


    public class RawServoSet implements Mode{
        double value;
        //value from 0 - 1 for servo
        public RawServoSet(double value){
            this.value = value;
        }

        @Override
        public void enter(@NotNull Mode previousMode){
            servoGroup.set(value);
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
            return "Servos Raw Set" + value;
        }
    }

//    public class RawAngleSet implements Mode{
//        Angle angle;
//        public RawAngleSet(Angle angle){
//            this.angle = angle;
//        }
//        @Override
//        public void enter(@NotNull Mode previousMode){
//            servoGroup.set(angle)
//        }
//        @NonNull
//        @Override
//        public Mode eval() {
//            return null;
//        }
//    }
    Mode disabled = new Mode() {
        @Override
        public void enter(@NotNull Mode previousMode){
            servoGroup.disable();
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
        return new RawServoSet(startingPosition);
    }



}
