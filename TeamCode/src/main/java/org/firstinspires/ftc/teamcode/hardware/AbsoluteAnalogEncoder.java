package org.firstinspires.ftc.teamcode.hardware;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.seattlesolvers.solverslib.util.MathUtils;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.util.Angle;

@Config
public class AbsoluteAnalogEncoder {
    public static double DEFAULT_RANGE = 3.2;
    private AnalogInput encoder;
    private double offset = -0.95 - 2.0 + 0.8;//volts
    private final double range;
    private boolean reversed;
    double lastPosition;
    //Angle Unit Radians
    public AbsoluteAnalogEncoder(HardwareMap map, String name, double range) {
        this.encoder = map.get(AnalogInput.class, name);
        this.range = range;
        this.reversed = true;
    }

    public AbsoluteAnalogEncoder(HardwareMap hwMap, String id) {
        this(hwMap, id, DEFAULT_RANGE);
    }

    public AbsoluteAnalogEncoder zero(double offset) {
        this.offset = offset;
        return this;
    }

    public AbsoluteAnalogEncoder setReversed(boolean reversed) {
        this.reversed = reversed;
        return this;
    }

    public boolean getReversed() {
        return reversed;
    }

    public Angle getCurrentPosition() {
        double newPos = ((reversed ? (-encoder.getVoltage() - offset + 1.6)/ (range) : (encoder.getVoltage() - offset) / range) * 2.0 * Math.PI);
        lastPosition = newPos;
        Angle absAngle = Angle.fromRadians(lastPosition).wrap();//wrap -180 to 180
        return absAngle;
    }


    public AnalogInput getEncoder() {
        return encoder;
    }

    public double getOffsetedVoltage(){
        return -encoder.getVoltage() - offset;
    }

    public double getRawVoltage(){
        return encoder.getVoltage();
    }
}