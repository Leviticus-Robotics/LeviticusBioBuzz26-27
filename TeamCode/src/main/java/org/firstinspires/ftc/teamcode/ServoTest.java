package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.constants.ConfigNames;

@TeleOp
public class ServoTest extends OpMode {
    Servo servo;
    @Override
    public void init() {
        servo = hardwareMap.get(Servo.class,  ConfigNames.transferGate);
    }

    @Override
    public void loop() {
        servo.setPosition(0.50);
    }
}
