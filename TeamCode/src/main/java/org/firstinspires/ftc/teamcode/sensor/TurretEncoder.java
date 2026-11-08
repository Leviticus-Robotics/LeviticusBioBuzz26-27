package org.firstinspires.ftc.teamcode.sensor;


import static dev.frozenmilk.dairy.mercurial.continuations.Continuations.exec;
import static dev.frozenmilk.dairy.mercurial.continuations.Continuations.expression;

import com.qualcomm.robotcore.hardware.AnalogInput;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.constants.ConfigNames;
import org.firstinspires.ftc.teamcode.hardware.IncrementalEncoder;
import org.firstinspires.ftc.teamcode.util.Angle;

import dev.frozenmilk.dairy.mercurial.continuations.Continuation;
import dev.frozenmilk.dairy.mercurial.continuations.Continuations;
import dev.frozenmilk.dairy.mercurial.ftc.Context;
import dev.frozenmilk.dairy.mercurial.processes.EventManager;
import dev.frozenmilk.util.collections.Cons;

/**
 * Melonbotics through bore encoder
 * Supports both 4 pin quadrature (incremental) and 3 pin analog (absolute)
 * Use absolute to get the starting position, set incremental to that position
 */
public class TurretEncoder extends EventManager<Angle> {
    AnalogInput absoluteEncoder;
    IncrementalEncoder incrementalEncoder;
    Telemetry.Item telemetry;
    private double startPosition;//radians
    private double recentPosition;//acoutning for wrap around

    private final double TICKS_PER_REV = 4096;

    //reset encoder resets the incrementla encoder
    public TurretEncoder(Context ctx){
        //when the turret encoder is started read the absolute position
        absoluteEncoder = ctx.hardwareMap().get(AnalogInput.class, ConfigNames.turretAnalogEncoder);
        incrementalEncoder = new IncrementalEncoder(ctx.hardwareMap(), ConfigNames.turretQuadratureEncoder, TICKS_PER_REV, AngleUnit.RADIANS);
        telemetry = ctx.telemetry().addData("Turret Encoder Position: ", "unknown");
        //might not always want to reset encoder at the start in case wraparound exists
        startPosition = getRadiansFromVoltage(absoluteEncoder);
    }

    //only need to call this in autonomous, not teleop b/c incremental value saves over


    public void resetEncoderAsync(){
        incrementalEncoder.resetEncoder();
    }
    public Continuation.Builder<?> resetEncoderSync() {
        return exec(this::resetEncoderAsync);
    }


    //gets the absolute encoder val and set the incremental encoder to that position
    public void resetFromAbsoluteAsync(){
        double newAbsPosition = getRadiansFromVoltage(absoluteEncoder);
        setEncoderAngle(newAbsPosition);
        //reset incremental encoder offset to new absolute position
    }
    public Continuation.Builder<?> resetFromAbsoluteSync(){
        return exec(this::resetFromAbsoluteAsync);
    }

    //reset encoder angle to zero(changes offset)
    public void setEncoderAngle(double angleRad){
        incrementalEncoder.setAngle(angleRad);
    }

    public void resetEncoderAngle(){
        setEncoderAngle(0);
    }

    @Override
    protected void eventSource() {
        recentPosition = update(incrementalEncoder);
        Angle angle = Angle.fromRadians(recentPosition);
        notify(angle);
        telemetry.setValue(Math.toDegrees(recentPosition) + " deg");
    }

    private double update(IncrementalEncoder encoder){
        double radians = encoder.getAngleUnnormalized(); //update incremental sensor value
        return radians;
    }

    private double getRadiansFromVoltage(AnalogInput input){
        return input.getVoltage() / input.getMaxVoltage() * 2 * Math.PI;//scale voltage to radians
    }


    public double getStartPosition(){
        return startPosition;
    }

    public double getRecentPosition(){
        return recentPosition;
    }



}
