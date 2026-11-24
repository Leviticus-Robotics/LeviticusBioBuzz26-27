package org.firstinspires.ftc.teamcode.sensor;


import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DigitalChannel;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.constants.ConfigNames;
import org.firstinspires.ftc.teamcode.helper.SensorEvent;

import dev.frozenmilk.dairy.mercurial.ftc.Context;
import dev.frozenmilk.dairy.mercurial.processes.EventManager;

/**Event Manager for 4 x Gobilda Distance Sensor
 * 2 sensors per ball; total of 2 balls being detected, w/ intake bias to get balls close to the same spot
 * Need to tune thresholds
 * Supports Analog/Digital Mode
 */
@Config
public class IntakeBallSensors extends EventManager<SensorEvent> {
    //left when looking into the direction of the intake behind the robot, 1 is leftmost, 2 is rightmost for that slot
    DigitalChannel breaker;
    DigitalChannel gobildaDistance;

    //threshold at which anything below detects (leave some buffer)
    boolean prevBreaker1Interrupted;
    boolean prevGobilda2Interrupted;
    private Telemetry.Item sensor1Telemetry;
    private Telemetry.Item sensor2Telemetry;
    boolean start1;
    boolean start2;



    public IntakeBallSensors(Context ctx){
        breaker = ctx.hardwareMap().get(DigitalChannel.class, ConfigNames.ballSensor3);
        gobildaDistance = ctx.hardwareMap().get(DigitalChannel.class, ConfigNames.ballSensor4);
        breaker.setMode(DigitalChannel.Mode.INPUT);
        gobildaDistance.setMode(DigitalChannel.Mode.INPUT);
        sensor1Telemetry = ctx.telemetry().addData("Breaker3/State", "unknown");
        sensor2Telemetry = ctx.telemetry().addData("Gobilda4/State", "unknown");
    }


    @Override
    protected void eventSource(){
        //FOR Breaker 3
        boolean breaker1Interrupted = isInterrupted(breaker);
        if(prevBreaker1Interrupted != breaker1Interrupted || !start1){
            start1 = true;
            notify(new SensorEvent(1, breaker1Interrupted));
        }
        prevBreaker1Interrupted = breaker1Interrupted;
        sensor1Telemetry.setValue(breaker1Interrupted ? "Interrupted" : "Clear");

        //FOR Gobilda 4
        boolean gobilda2Interrupted = gobildaDistance.getState();
        if(prevGobilda2Interrupted != gobilda2Interrupted || !start2){
            start2 = true;
            notify(new SensorEvent(2, gobilda2Interrupted));
        }
        prevGobilda2Interrupted = gobilda2Interrupted;
        sensor2Telemetry.setValue(gobilda2Interrupted ? "Interrupted" : "Clear");

    }

    public boolean isInterrupted(DigitalChannel sensor){
        return !sensor.getState();
    }

    public boolean getBreaker1Interrupted(){
        return prevBreaker1Interrupted;
    }

    public boolean getBreaker2Interrupted(){
        return prevGobilda2Interrupted;
    }

}