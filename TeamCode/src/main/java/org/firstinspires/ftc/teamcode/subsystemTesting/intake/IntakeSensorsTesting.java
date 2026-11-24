package org.firstinspires.ftc.teamcode.subsystemTesting.intake;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.DigitalChannel;

import org.firstinspires.ftc.teamcode.constants.ConfigNames;

@Config
@TeleOp
public class IntakeSensorsTesting extends OpMode {
    DigitalChannel gobildaDistance1;//closest to shooter
    DigitalChannel breaker2;
    DigitalChannel breaker3;
    DigitalChannel gobildaDistance4;

    @Override
    public void init() {
        gobildaDistance1 = hardwareMap.get(DigitalChannel.class, ConfigNames.ballSensor1);
        breaker2 = hardwareMap.get(DigitalChannel.class, ConfigNames.ballSensor2);
        breaker3 = hardwareMap.get(DigitalChannel.class, ConfigNames.ballSensor3);
        gobildaDistance4 = hardwareMap.get(DigitalChannel.class, ConfigNames.ballSensor4);

    }

    @Override
    public void loop() {
        telemetry.addData("Sensor1", gobildaDistance1.getState());
        telemetry.addData("Sensor2", !breaker2.getState());
        telemetry.addData("Sensor3", !breaker3.getState());
        telemetry.addData("Sensor4", gobildaDistance4.getState());

        telemetry.update();
    }

}
