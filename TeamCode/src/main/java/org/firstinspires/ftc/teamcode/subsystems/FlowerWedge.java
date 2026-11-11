package org.firstinspires.ftc.teamcode.subsystems;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.PwmControl;
import com.seattlesolvers.solverslib.hardware.servos.ServoEx;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.constants.ConfigNames;

import dev.frozenmilk.dairy.mercurial.ftc.Context;
import dev.frozenmilk.dairy.mercurial.processes.SwitchedStateMachine;

/** Flower Wedge
 *  Uses: 1 Servo
 *  Positional Control Servo w/ boundaries set
 */

@Config
public class FlowerWedge extends SwitchedStateMachine {
    ServoEx servo;
    Telemetry.Item stateTelemetry;
    Mode down;
    Mode up;
    public static double DOWN_POSITION = 0.5;
    public static double UP_POSITION = 0.8;
    public static boolean INVERTED = false;

    public FlowerWedge(Context ctx){
        servo = new ServoEx(ctx.hardwareMap(), ConfigNames.flowerMechanism);
        servo.setInverted(INVERTED);
        servo.setPwm(PwmControl.PwmRange.defaultRange);//can alter pwm range like so
        servo.setCachingTolerance(0.0001); //can alter caching tolerance like so

        stateTelemetry = ctx.telemetry().addData("FlowerWedge/State", "unknown");
        //start from this mode
        up = entranceMode(from -> {
            stateTelemetry.setValue("up");
            servo.set(UP_POSITION);
        });

        down = mode(self -> {
            stateTelemetry.setValue("down");
            servo.set(DOWN_POSITION);
            return self;
        });
    }
    @NonNull
    @Override
    protected Mode init(){
        return up;
    }

    public void setDown(){
        switchTo(down);
    }

    public void setUp(){
        switchTo(up);
    }

    //Returns previous set position (not current servo position) b/c assumes no encoder attached
    public double getPrevPosition(){
        return servo.getRawPosition();
    }
}

