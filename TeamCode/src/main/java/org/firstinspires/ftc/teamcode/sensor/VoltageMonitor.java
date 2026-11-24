package org.firstinspires.ftc.teamcode.sensor;

import com.qualcomm.robotcore.hardware.VoltageSensor;

import dev.frozenmilk.dairy.mercurial.ftc.Context;
import dev.frozenmilk.dairy.mercurial.processes.EventManager;

/** Simple Voltage Monitoring Event Manger
 *  Receives voltage and sends it to the channel every loop
 */
public class VoltageMonitor extends EventManager<Double> {
    private VoltageSensor sensor;
    boolean hasReadSensors;
    double lastSensorReadTime;
    public double SENSOR_UPDATE_INTERVAL_MS = 20;

    public VoltageMonitor(Context ctx){
        sensor = ctx.hardwareMap().voltageSensor.iterator().next();
    }

    @Override
    protected void eventSource(){
        long currTime = System.currentTimeMillis();
        if(!hasReadSensors || currTime - lastSensorReadTime >= SENSOR_UPDATE_INTERVAL_MS) {
            lastSensorReadTime = currTime;
            hasReadSensors = true;
            double currentVoltage = sensor.getVoltage();
            if (currentVoltage > 0) {
                notify(currentVoltage);
            }
        }

    }

}
