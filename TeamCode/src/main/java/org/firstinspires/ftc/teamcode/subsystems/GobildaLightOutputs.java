package org.firstinspires.ftc.teamcode.subsystems;



import androidx.annotation.NonNull;

import com.qualcomm.robotcore.hardware.LED;
import com.qualcomm.robotcore.hardware.Servo;

import dev.frozenmilk.dairy.mercurial.ftc.Context;
import dev.frozenmilk.dairy.mercurial.processes.Channel;
import dev.frozenmilk.dairy.mercurial.processes.SwitchedStateMachine;

/**
 * 2 lights:
 * #1) for reporting turret within range & shooter velocity good & at right position to shoot
 * #2) for reporting
 *
 */
public class GobildaLightOutputs extends SwitchedStateMachine {
    Servo led;

    @NonNull
    @Override
    protected Mode init() {
        return solidColor(ColorMode.Black);
    }

    enum ColorMode{
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
        led = ctx.hardwareMap().get(Servo.class, "Gobilda LED");
        led.setPosition(1);
    }
    private Mode solidColor(ColorMode color) {
        return mode(self -> {
            this.led.setPosition(color.getValue());
            return self;
        });
    }
    public void setSolidColor(ColorMode color) {
        this.switchTo(solidColor(color));
    }
}
