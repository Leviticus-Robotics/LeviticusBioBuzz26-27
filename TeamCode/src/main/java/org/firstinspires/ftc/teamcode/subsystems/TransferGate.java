package org.firstinspires.ftc.teamcode.subsystems;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.PwmControl;
import com.seattlesolvers.solverslib.hardware.servos.ServoEx;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.constants.ConfigNames;

import dev.frozenmilk.dairy.mercurial.ftc.Context;
import dev.frozenmilk.dairy.mercurial.processes.SwitchedStateMachine;

/** Transfer Gate
 * Uses 1 servo
 * Active(meaning it prevents balls from going up)
 * Inactive(balls can freely go up)
 */

@Config
public class TransferGate extends SwitchedStateMachine {
    ServoEx servo;
    Telemetry.Item stateTelemetry;
    Mode closed;
    Mode open;
    public static double OPEN_POSITION = 0.5;
    public static double CLOSED_POSITION = 0.8;
    public static boolean INVERTED = false;

    public TransferGate(Context ctx){
        servo = new ServoEx(ctx.hardwareMap(), ConfigNames.transferGate);
        servo.setInverted(INVERTED);
        servo.setPwm(PwmControl.PwmRange.defaultRange);//can alter pwm range like so
        servo.setCachingTolerance(0.0001); //can alter caching tolerance like so

        stateTelemetry = ctx.telemetry().addData("TransferGate/State", "unknown");
        //start from this mode
        open = entranceMode(from -> {
            stateTelemetry.setValue("inactive");
            servo.set(OPEN_POSITION);
        });

        closed = mode(self -> {
            stateTelemetry.setValue("active");
            servo.set(CLOSED_POSITION);
            return self;
        });
    }
    @NonNull
    @Override
    protected Mode init(){
        return closed;
    }

    public void setOpen(){
        switchTo(open);
    }

    public void setClosed(){
        switchTo(closed);
    }

    //Returns previous set position (not current servo position) b/c assumes no encoder attached
    public double getPrevPosition(){
        return servo.getRawPosition();
    }
}

