package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;
import com.seattlesolvers.solverslib.hardware.motors.CRServo;

@Config
@TeleOp
public class ServoTest extends OpMode {
    CRServo servo;
    public static double position = 0;
    public static Servo.Direction direction = Servo.Direction.FORWARD;

    @Override
    public void init() {
        servo = new CRServo(hardwareMap, "servo");
        servo.setInverted(direction == Servo.Direction.FORWARD);
    }

    @Override
    public void loop() {
        servo.set(position);

    }
}
