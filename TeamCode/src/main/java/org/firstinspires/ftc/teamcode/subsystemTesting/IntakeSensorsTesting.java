package org.firstinspires.ftc.teamcode.subsystemTesting;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.DigitalChannel;

import org.firstinspires.ftc.teamcode.constants.ConfigNames;

@Config
@TeleOp
public class IntakeSensorsTesting extends OpMode {
    DigitalChannel left1;

    public static final double MAX_VOLTS = 3.3;
    public static final double MAX_DISTANCE_MM = 1000.0;
    double distance;
    boolean left1Detected;
    public static double LEFT1_THRESHOLD = 2.1;
    double volts;

    @Override
    public void init() {
        left1 = hardwareMap.get(DigitalChannel.class, ConfigNames.gobildaLeft1);
    }

    @Override
    public void loop() {
//        left1Detected = isDetected(left1, LEFT1_THRESHOLD, 0);

        telemetry.addData("Breaker1/State", "unknown");
        telemetry.addData("Left1 Detected", left1.getState());
//        telemetry.addData("Distance", distance);
//        telemetry.addData("Volts", volts);
    }

    private double getDistance(AnalogInput sensor){
        volts = sensor.getVoltage();
        double distanceInch = (volts / MAX_VOLTS) * MAX_DISTANCE_MM / 25.4;
        return distanceInch;
    }

    private boolean isDetected(AnalogInput sensor, double thresholdDetection, int index){
        distance = getDistance(sensor);//update sensor distances
        if(thresholdDetection < 0) return false;
        return distance < thresholdDetection;
    }

}
