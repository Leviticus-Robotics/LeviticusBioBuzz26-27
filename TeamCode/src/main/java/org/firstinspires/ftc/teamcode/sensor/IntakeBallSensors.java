package org.firstinspires.ftc.teamcode.sensor;


import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.AnalogInput;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.BallSlot;
import org.firstinspires.ftc.teamcode.constants.ConfigNames;

import dev.frozenmilk.dairy.mercurial.ftc.Context;
import dev.frozenmilk.dairy.mercurial.processes.EventManager;

/**Event Manager for 4 x Gobilda Distance Sensor
 * 2 sensors per ball; total of 2 balls being detected, w/ intake bias to get balls close to the same spot
 * Need to tune thresholds
 * Supports Analog/Digital Mode
 */
@Config
public class IntakeBallSensors extends EventManager<BallSlot> {
    //left when looking into the direction of the intake behind the robot, 1 is leftmost, 2 is rightmost for that slot
    AnalogInput left1;
    AnalogInput left2;
    AnalogInput right1;
    AnalogInput right2;

    public static double LEFT1_THRESHOLD = 2.1;
    public static double LEFT2_THRESHOLD = 2.1;
    public static double RIGHT1_THRESHOLD = 3.2;
    public static double RIGHT2_THRESHOLD = 3.2;
    //threshold at which anything below detects (leave some buffer)
    public static final double MAX_VOLTS = 3.3;
    public static final double MAX_DISTANCE_MM = 1000.0;

    private double sensorDistances[] = new double[4];//for telemetry/checks

    private boolean left1Detected;
    private boolean left2Detected;
    private boolean right1Detected;
    private boolean right2Detected;

    private boolean prevSlot1Detection;
    private boolean prevSlot2Detection;
    private Telemetry.Item breaker1Telemetry;
    private Telemetry.Item breaker2Telemetry;



    public IntakeBallSensors(Context ctx){
        this.left1 = ctx.hardwareMap().get(AnalogInput.class, ConfigNames.gobildaLeft1);
        this.left2 = ctx.hardwareMap().get(AnalogInput.class, ConfigNames.gobildaLeft2);
        this.right1 = ctx.hardwareMap().get(AnalogInput.class, ConfigNames.gobildaRight1);
        this.right2 = ctx.hardwareMap().get(AnalogInput.class, ConfigNames.gobildaRight2);
        breaker1Telemetry = ctx.telemetry().addData("Breaker1/State", "unknown");
        breaker2Telemetry = ctx.telemetry().addData("Breaker2/State", "unknown");
    }

    @Override
    protected void eventSource(){
        //FOR SLOT 1
        left1Detected = isDetected(left1, LEFT1_THRESHOLD, 0);
        left2Detected = isDetected(left2, LEFT2_THRESHOLD, 1);
        boolean slot1Detected = left1Detected || left2Detected;

        //as long as either of them detecting
        //solves the ball in the hole issue: when half the sensor beam sees the ball, other half sees the ball
        //2 sensors ensures that it has a higher chance to see solid part of ball
        if (prevSlot1Detection != slot1Detected){
            notify(new BallSlot(BallSlot.Slot.LEFT, slot1Detected));
        }
        prevSlot1Detection = slot1Detected;
        breaker1Telemetry.setValue(slot1Detected ? "Interrupted" : "Clear");


        //FOR SLOT 2
        right1Detected = isDetected(right1, RIGHT1_THRESHOLD, 2);
        right2Detected = isDetected(right2, RIGHT2_THRESHOLD, 3);
        boolean slot2Detected = right1Detected || right2Detected;

        if (prevSlot2Detection != slot2Detected) {
            notify(new BallSlot(BallSlot.Slot.RIGHT, slot2Detected));
        }
        prevSlot2Detection = slot2Detected;
        breaker2Telemetry.setValue(slot2Detected ? "Interrupted" : "Clear");
    }

    //need to tune MAX_VOLTS * MAX_DISTANCE_MM once
    private double getDistance(AnalogInput sensor){
        double volts = sensor.getVoltage();
        double distanceInch = (volts / MAX_VOLTS) * MAX_DISTANCE_MM / 25.4;
        return distanceInch;
    }

    private boolean isDetected(AnalogInput sensor, double thresholdDetection, int index){
        double distance = getDistance(sensor);
        sensorDistances[index] = distance;//update sensor distances
        if(thresholdDetection < 0) return false;
        return distance < thresholdDetection;
    }


    public boolean getSlot1Detection(){
        return prevSlot1Detection;
    }

    public boolean getSlot2Detection(){
        return prevSlot2Detection;
    }

    public boolean[] getSensorDetections(){
        return new boolean[]{left1Detected, left2Detected, right1Detected, right2Detected};
    }
    public double[] getSensorDistances(){
        return sensorDistances.clone();
    }


}