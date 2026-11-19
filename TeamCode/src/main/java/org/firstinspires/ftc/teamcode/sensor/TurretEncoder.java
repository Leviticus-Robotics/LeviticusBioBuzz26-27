package org.firstinspires.ftc.teamcode.sensor;


import static dev.frozenmilk.dairy.mercurial.continuations.Continuations.exec;
import static dev.frozenmilk.dairy.mercurial.continuations.Continuations.expression;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.hardware.AnalogInput;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.constants.ConfigNames;
import org.firstinspires.ftc.teamcode.hardware.IncrementalEncoder;
import org.firstinspires.ftc.teamcode.helper.AngleHelper;
import org.firstinspires.ftc.teamcode.util.Angle;

import dev.frozenmilk.dairy.mercurial.continuations.Continuation;
import dev.frozenmilk.dairy.mercurial.continuations.Continuations;
import dev.frozenmilk.dairy.mercurial.ftc.Context;
import dev.frozenmilk.dairy.mercurial.processes.Channel;
import dev.frozenmilk.dairy.mercurial.processes.EventManager;
import dev.frozenmilk.util.collections.Cons;

/**
 * Melonbotics through bore encoder
 * Supports both 4 pin quadrature (incremental) and 3 pin analog (absolute)
 * Use absolute to get the starting position, set incremental to that position
 */
public class TurretEncoder extends EventManager<AngleHelper> {
    AnalogInput absoluteEncoder;
    IncrementalEncoder incrementalEncoder;
    Telemetry.Item telemetry;
    private Angle startPosition;//radians
    public Angle recentIncrementalPosition;//use this, accounting for wrap around
    public Angle recentAbsolutePosition;

    private final double TICKS_PER_REV = 4096;
    private final double GR_ENCODER_TO_SERVO = (40.0 / 115.0);

    //reset encoder resets the incrementla encoder
    public TurretEncoder(Context ctx){
        //when the turret encoder is started read the absolute position
        absoluteEncoder = ctx.hardwareMap().get(AnalogInput.class, ConfigNames.turretAnalogEncoder);
        incrementalEncoder = new IncrementalEncoder(ctx.hardwareMap(), ConfigNames.turretQuadratureEncoder, TICKS_PER_REV, AngleUnit.RADIANS);
        telemetry = ctx.telemetry().addData("Turret Encoder Position: ", "unknown");
        //might not always want to reset encoder at the start in case wraparound exists
        startPosition = updateAbsolute(absoluteEncoder);
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
        Angle newAbsPosition = updateAbsolute(absoluteEncoder);
        setEncoderAngle(newAbsPosition.getValue());
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

    boolean hasReadSensors;
    double lastSensorReadTime;//ms
    final long SENSOR_UPDATE_INTERVAL_MS = 20;
    @Override
    protected void eventSource() {
        long currTime = System.currentTimeMillis();
        //only update every X seconds for semi-expensive io reads
        if(!hasReadSensors || currTime - lastSensorReadTime >= SENSOR_UPDATE_INTERVAL_MS){
            lastSensorReadTime = currTime;
            hasReadSensors = true;
            recentIncrementalPosition = updateIncremental(incrementalEncoder);
            recentAbsolutePosition = updateAbsolute(absoluteEncoder);

        }
        notify(new AngleHelper(recentIncrementalPosition, AngleHelper.AngleType.Incremental));
        notify(new AngleHelper(recentAbsolutePosition, AngleHelper.AngleType.Absolute));
    }

    private Angle updateIncremental(IncrementalEncoder encoder){
        double angle = encoder.getAngleUnnormalized();//update incremental sensor value
        return Angle.fromRadians(angle * GR_ENCODER_TO_SERVO);
    }

    private Angle updateAbsolute(AnalogInput input){
        return Angle.fromRadians(input.getVoltage() / input.getMaxVoltage() * 2 * Math.PI * GR_ENCODER_TO_SERVO);//scale voltage to radians
    }


    public Angle getStartPosition(){
        return startPosition;
    }

    public Angle getRecentIncrementalPosition(){
        return recentIncrementalPosition;
    }

    public Angle getRecentAbsolutePosition(){
        return recentAbsolutePosition;
    }




}
