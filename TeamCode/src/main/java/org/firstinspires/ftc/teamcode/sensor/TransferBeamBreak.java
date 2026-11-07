package org.firstinspires.ftc.teamcode.sensor;

import com.qualcomm.robotcore.hardware.DigitalChannel;

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

public class TransferBeamBreak extends EventManager<TransferBeamBreak.BeamBreakEvent> {
    //if the beam between receiver/trnsmitter is broken (meaning object detected) or unbroken (meaning no object detected)
    DigitalChannel breaker1;//most inward spot
    DigitalChannel breaker2;
    boolean prevBreaker1Interrupted;
    boolean prevBreaker2Interrupted;
    private Telemetry.Item breaker1Telemetry;
    private Telemetry.Item breaker2Telemetry;

    public static class BeamBreakEvent{
        public int sensorNum;
        public boolean interrupted;
        public BeamBreakEvent(int sensorInt, boolean interrupted){
            this.sensorNum = sensorInt;
            this.interrupted = interrupted;
        }
    }

    public TransferBeamBreak(Context ctx){
        breaker1 = ctx.hardwareMap().get(DigitalChannel.class, ConfigNames.breaker1);
        breaker2 = ctx.hardwareMap().get(DigitalChannel.class, ConfigNames.breaker2);
        breaker1.setMode(DigitalChannel.Mode.INPUT);
        breaker2.setMode(DigitalChannel.Mode.INPUT);
        breaker1Telemetry = ctx.telemetry().addData("Breaker1/State", "unknown");
        breaker2Telemetry = ctx.telemetry().addData("Breaker2/State", "unknown");
    }


    @Override
    protected void eventSource(){
        //FOR BREAKER 1
        boolean breaker1Interrupted = isInterrupted(breaker1);
        if(prevBreaker1Interrupted != breaker1Interrupted){
            notify(new BeamBreakEvent(1, breaker1Interrupted));
        }
        prevBreaker1Interrupted = breaker1Interrupted;
        breaker1Telemetry.setValue(breaker1Interrupted ? "Interrupted" : "Clear");

        //FOR BREAKER 2
        boolean breaker2Interrupted = isInterrupted(breaker2);
        if(prevBreaker2Interrupted != breaker2Interrupted){
            notify(new BeamBreakEvent(2, breaker2Interrupted));
        }
        prevBreaker2Interrupted = breaker2Interrupted;
        breaker2Telemetry.setValue(breaker2Interrupted ? "Interrupted" : "Clear");

    }

    public boolean isInterrupted(DigitalChannel sensor){
        return sensor.getState();
    }

    public boolean getBreaker1Interrupted(){
        return prevBreaker1Interrupted;
    }

    public boolean getBreaker2Interrupted(){
        return prevBreaker2Interrupted;
    }
}
