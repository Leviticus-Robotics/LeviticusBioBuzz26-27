package org.firstinspires.ftc.teamcode.sensor;

import com.qualcomm.robotcore.hardware.VoltageSensor;

import dev.frozenmilk.dairy.mercurial.ftc.Context;
import dev.frozenmilk.dairy.mercurial.processes.EventManager;

/** Simple Voltage Monitoring Event Manger
 *  Receives voltage and sends it to the channel every loop
 */
public class VoltageMonitor extends EventManager<Double> {
    private VoltageSensor sensor;
    public VoltageMonitor(Context ctx){
        sensor = ctx.hardwareMap().voltageSensor.iterator().next();
    }

    @Override
    protected void eventSource(){
        if(sensor != null){
            double currentVoltage = sensor.getVoltage();
            if(currentVoltage > 0){
                notify(currentVoltage);
            }
        }
    }

}
