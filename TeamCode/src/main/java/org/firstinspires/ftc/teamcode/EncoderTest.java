package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.AnalogInput;

import org.firstinspires.ftc.teamcode.constants.ConfigNames;
import org.firstinspires.ftc.teamcode.util.Angle;

@TeleOp
public class EncoderTest extends OpMode{

    AnalogInput analogInput;
    @Override
    public void init() {
        analogInput = hardwareMap.get(AnalogInput.class, ConfigNames.turretAnalogEncoder);
    }

    @Override
    public void loop() {
        telemetry.addData("Analog Voltage", analogInput.getVoltage());//scale voltage to radians
        telemetry.update();
    }
}
