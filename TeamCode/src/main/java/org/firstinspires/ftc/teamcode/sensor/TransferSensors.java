package org.firstinspires.ftc.teamcode.sensor;

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

public class TransferSensors extends EventManager<SensorEvent> {
    //if the beam between receiver/trnsmitter is broken (meaning object detected) or unbroken (meaning no object detected)
    DigitalChannel gobildaDistance;//closest to shooter
    DigitalChannel breaker;//farther from shooter
    boolean prevGobilda1Interrupted;
    boolean prevBreaker2Interrupted;
    private Telemetry.Item sensor1Telemetry;
    private Telemetry.Item sensor2Telemetry;
    boolean start1;
    boolean start2;


    public TransferSensors(Context ctx){
        gobildaDistance = ctx.hardwareMap().get(DigitalChannel.class, ConfigNames.ballSensor1);
        breaker = ctx.hardwareMap().get(DigitalChannel.class, ConfigNames.ballSensor2);
        gobildaDistance.setMode(DigitalChannel.Mode.INPUT);
        breaker.setMode(DigitalChannel.Mode.INPUT);
        sensor1Telemetry = ctx.telemetry().addData("Gobilda1/State", "unknown");
        sensor2Telemetry = ctx.telemetry().addData("Breaker2/State", "unknown");
    }


    @Override
    protected void eventSource(){
        //FOR Gobilda 1
        boolean gobilda1Interrupted = gobildaDistance.getState();
        if(prevGobilda1Interrupted != gobilda1Interrupted || !start1){
            start1 = true;
            notify(new SensorEvent(1, gobilda1Interrupted));
        }
        prevGobilda1Interrupted = gobilda1Interrupted;
        sensor1Telemetry.setValue(gobilda1Interrupted ? "Interrupted" : "Clear");

        //FOR BREAKER 2
        boolean breaker2Interrupted = isInterrupted(breaker);
        if(prevBreaker2Interrupted != breaker2Interrupted || !start2){
            start2 = true;
            notify(new SensorEvent(2, breaker2Interrupted));
        }
        prevBreaker2Interrupted = breaker2Interrupted;
        sensor2Telemetry.setValue(breaker2Interrupted ? "Interrupted" : "Clear");

    }

    public boolean isInterrupted(DigitalChannel sensor){
        return !sensor.getState();
    }

    public boolean getBreaker1Interrupted(){
        return prevGobilda1Interrupted;
    }

    public boolean getBreaker2Interrupted(){
        return prevBreaker2Interrupted;
    }
}
