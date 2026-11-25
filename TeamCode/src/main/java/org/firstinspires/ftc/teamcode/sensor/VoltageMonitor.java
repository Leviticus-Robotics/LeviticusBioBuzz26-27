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
    private static final long CACHE_DURATION_NANOS = 20_000_000L;
    private long nextNanos = System.nanoTime();
    public VoltageMonitor(Context ctx){
        sensor = ctx.hardwareMap().voltageSensor.iterator().next();
    }

    @Override
    protected void eventSource(){
        long currTime = System.nanoTime();
        if (currTime >= nextNanos) {
            lastSensorReadTime = currTime;
            hasReadSensors = true;
            double currentVoltage = sensor.getVoltage();
            if (currentVoltage > 0) {
                notify(currentVoltage);
            }
            nextNanos = System.nanoTime() + CACHE_DURATION_NANOS;
        }

    }

}
