package org.firstinspires.ftc.teamcode.sensor;


import static dev.frozenmilk.dairy.mercurial.continuations.Continuations.exec;
import static dev.frozenmilk.dairy.mercurial.continuations.Continuations.expression;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.hardware.AnalogInput;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.constants.ConfigNames;
import org.firstinspires.ftc.teamcode.hardware.AbsoluteAnalogEncoder;
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
@Config
public class TurretEncoder extends EventManager<AngleHelper> {
    AbsoluteAnalogEncoder absoluteEncoder;
    IncrementalEncoder incrementalEncoder;
    private Angle startPosition;//radians
    public Angle recentIncrementalPosition;//use this, accounting for wrap around
    public Angle recentAbsolutePosition;

    private final double TICKS_PER_REV = 4096;
    private final double GR_ENCODER_TO_TURRET = (40.0 / 115.0);
    public static double range = 3.23;
    public static double analogOffset = 0;

    //reset encoder resets the incrementla encoder
    Telemetry.Item voltageOffsetAnalog;
    Telemetry.Item voltageRawAnalog;
    public TurretEncoder(Context ctx){
        //both using radians
        absoluteEncoder = new AbsoluteAnalogEncoder(ctx.hardwareMap(), ConfigNames.turretAnalogEncoder, range);
        incrementalEncoder = new IncrementalEncoder(ctx.hardwareMap(), ConfigNames.turretQuadratureEncoder, TICKS_PER_REV, AngleUnit.RADIANS);
        voltageOffsetAnalog = ctx.telemetry().addData("Voltage analog", "unknown");
        voltageRawAnalog = ctx.telemetry().addData("Voltage Raw analog", "unknown");
        incrementalEncoder.setReversed(true);
    }
    //only need to call this in autonomous, not teleop b/c incremental value saves over


    public void zeroIncremental(){
        incrementalEncoder.resetEncoder();
    }

    //gets the absolute encoder val and set the incremental encoder to that position
    public void resetFromAbsolute(){
        Angle newAbsPosition = absoluteEncoder.getCurrentPosition();
        setEncoderAngle(newAbsPosition.getValue());
        //reset incremental encoder offset to new absolute position
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
        //not notifying every loop time now
        if(!hasReadSensors || currTime - lastSensorReadTime >= SENSOR_UPDATE_INTERVAL_MS){
            lastSensorReadTime = currTime;
            hasReadSensors = true;
            recentIncrementalPosition = updateIncremental(incrementalEncoder);
            recentAbsolutePosition = updateAbsolute(absoluteEncoder);
            notify(new AngleHelper(recentIncrementalPosition, AngleHelper.AngleType.Incremental));
            notify(new AngleHelper(recentAbsolutePosition, AngleHelper.AngleType.Absolute));
        }
    }

    //scales incremental so that the angle returned is from turret perspective, not encoder
    private Angle updateIncremental(IncrementalEncoder encoder){
        double angle = encoder.getAngleUnnormalized();//update incremental sensor value
        return Angle.fromRadians(angle * GR_ENCODER_TO_TURRET);
    }

    //scales absoltue so that angle returned is from turret perspective, not encoder
    private Angle updateAbsolute(AbsoluteAnalogEncoder absoluteEncoder){
        voltageOffsetAnalog.setValue(absoluteEncoder.getOffsetedVoltage());
        voltageRawAnalog.setValue(absoluteEncoder.getRawVoltage());
        return Angle.fromRadians(absoluteEncoder.getCurrentPosition().getValue() * GR_ENCODER_TO_TURRET);//scale voltage to radians
    }


    public Angle getRecentIncrementalPosition(){
        return recentIncrementalPosition;
    }

    public Angle getRecentAbsolutePosition(){
        return recentAbsolutePosition;
    }




}
