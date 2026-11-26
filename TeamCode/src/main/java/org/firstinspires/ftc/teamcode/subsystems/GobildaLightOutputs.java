package org.firstinspires.ftc.teamcode.subsystems;



import androidx.annotation.NonNull;

import com.qualcomm.robotcore.hardware.LED;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.constants.ConfigNames;

import dev.frozenmilk.dairy.mercurial.ftc.Context;
import dev.frozenmilk.dairy.mercurial.processes.Channel;
import dev.frozenmilk.dairy.mercurial.processes.SwitchedStateMachine;

/**
 * 2 lights:
 * #1) for reporting
 * #2) for reporting
 *
 */
public class GobildaLightOutputs extends SwitchedStateMachine{
    Servo led1;
    Servo led2;
    Mode startMode;

    @NonNull
    @Override
    protected Mode init() {
        return startMode;
    }



    public enum ColorMode{
        Black(0.0),
        Red(.277),
        Orange(.333),
        Yellow(.388),
        Sage(.444),
        Green(.500),
        Azure(.555),
        Blue(.611),
        Indigo(.666),
        Violet(.722),
        White(1.0);
        public final double value;
        ColorMode(double value) {
            this.value = value;
        }
        public double getValue(){
            return this.value;
        }
    }
    public GobildaLightOutputs(Context ctx) {
        led1 = ctx.hardwareMap().get(Servo.class, ConfigNames.lightLeft);
        led2 = ctx.hardwareMap().get(Servo.class, ConfigNames.lightRight);
        startMode = entranceMode(self -> {
            led1.setPosition(ColorMode.Black.getValue());
            led2.setPosition(ColorMode.Black.getValue());
        });
    }
    private Mode solidColor(ColorMode color) {
        return entranceMode(self -> {
            this.led1.setPosition(color.getValue());
            this.led2.setPosition(color.getValue());
        });
    }

    public void setSolidColor(ColorMode color) {
        switchTo(solidColor(color));
    }
}
