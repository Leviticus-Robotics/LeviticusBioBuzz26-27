package org.firstinspires.ftc.teamcode.helper;

public class SensorEvent{
    public int sensorNum;
    public boolean interrupted;
    public SensorEvent(int sensorInt, boolean interrupted){
        this.sensorNum = sensorInt;
        this.interrupted = interrupted;
    }
}