package org.firstinspires.ftc.teamcode.sensor;


import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.BallSlot;
import org.firstinspires.ftc.teamcode.constants.ConfigNames;
import org.firstinspires.ftc.teamcode.hardware.IncrementalEncoder;

import dev.frozenmilk.dairy.mercurial.ftc.Context;
import dev.frozenmilk.dairy.mercurial.processes.EventManager;

/**
 * Melonbotics through bore encoder
 * Supports both 4 pin quadrature (incremental) and 3 pin analog (absolute)
 * Use absolute to get the starting position, set incremental to that position
 */
public class TurretEncoder extends EventManager<Double> {
    AnalogInput absoluteEncoder;
    IncrementalEncoder incrementalEncoder;
    Telemetry.Item telemetry;
    private double startPosition;//radians
    private double recentPosition;//acoutning for wrap around

    private final double TICKS_PER_REV = 4096;

    //need to
    public TurretEncoder(Context ctx){
        //when the turret encoder is started read the absolute position
        absoluteEncoder = ctx.hardwareMap().get(AnalogInput.class, ConfigNames.turretAnalogEncoder);
        incrementalEncoder = new IncrementalEncoder(ctx.hardwareMap().get(DcMotorEx.class, ConfigNames.turretQuadratureEncoder), TICKS_PER_REV, AngleUnit.RADIANS);
        telemetry = ctx.telemetry().addData("TurretEncoder", "unknown");
        //might not always want to reset encoder at the start in case wraparound exists
    }

    public void resetEncoderToAbsolute(){//gets the absolute encoder val and set the incremental encoder to that position
        double newAbsPosition = getRadiansFromVoltage(absoluteEncoder);
        incrementalEncoder.setAngle(newAbsPosition);//reset incremental encoder to new absolute position
    }

    public void resetOffset(){//this is not ideal b/c can't reset to absoute angle anymore
        incrementalEncoder.resetOffset();
    }

    public void resetEncoderToZero(){
        incrementalEncoder.setAngle(0);
    }

    @Override
    protected void eventSource() {
    }


    private double getRadiansFromVoltage(AnalogInput input){
        return input.getVoltage() / input.getMaxVoltage() * 360.0 * Math.PI / 180;//scale voltage to degrees
    }




}
