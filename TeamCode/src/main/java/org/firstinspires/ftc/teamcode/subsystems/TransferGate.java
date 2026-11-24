package org.firstinspires.ftc.teamcode.subsystems;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.PwmControl;
import com.qualcomm.robotcore.hardware.Servo;
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
    Servo servo;
    private Telemetry.Item stateTelemetry;
    private Telemetry.Line endBreakLine;
    Mode closed;
    Mode open;
    Mode wait;
    public static double OPEN_POSITION = 0.65;
    public static double CLOSED_POSITION = 0.45;
    public static boolean INVERTED = false;
    public static double cachingTolerance = 0.01;
    public boolean openPosition = false;
    public TransferGate(Context ctx){
//        servo = new ServoEx(ctx.hardwareMap(), ConfigNames.transferGate);
//        servo.setInverted(INVERTED);
          servo = ctx.hardwareMap().get(Servo.class, ConfigNames.transferGate);
//        servo.setCachingTolerance(cachingTolerance);
//        servo.setPwm(PwmControl.PwmRange.defaultRange);//can alter pwm range like so
//        servo.setCachingTolerance(0.0001); //can alter caching tolerance like so

        stateTelemetry = ctx.telemetry().addData("TransferGate/State", "Unknown");
        endBreakLine = ctx.telemetry().addLine("-----------------------");
        //start from this mode
        open = entranceMode(from -> {
            stateTelemetry.setValue("Open");
            servo.setPosition(OPEN_POSITION);
            openPosition = true;
        });

        closed = entranceMode(self -> {
            stateTelemetry.setValue("Closed");
            servo.setPosition(CLOSED_POSITION);
            openPosition = false;
        });
//        wait = entranceMode(self -> {
//            //does nothing
//        });
    }

    public Mode customPositionMode(double servoPos){
        return entranceMode(self -> {
            servo.setPosition(servoPos);
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

    public Mode getClosed(){
        return mode(self -> {
            servo.setPosition(CLOSED_POSITION);
            openPosition = false;
            stateTelemetry.setValue("Closed");
            return self;
        });
    }
    public void setClosed(){
        switchTo(getClosed());
    }

//    private void setWait(){switchTo(wait);}
    public void setCustomPosition(double servoPos){
        switchTo(customPositionMode(servoPos));
    }
    //Returns previous set position (not current servo position) b/c assumes no encoder attached
    public double getPrevPosition(){
        return servo.getPosition();
    }
}

