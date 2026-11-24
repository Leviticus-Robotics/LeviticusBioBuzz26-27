package org.firstinspires.ftc.teamcode.hardware;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.seattlesolvers.solverslib.util.MathUtils;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

/** DcMotor Throughbore Encoder
 * Does not support absolute, so incremental works
 *
 */
public class IncrementalEncoder {
    public final DcMotorEx encoder;
    private final double cpr;
    private double offset = 0.0;//this in ticks
    private RotationDirection direction = RotationDirection.FORWARD;
    private AngleUnit angleUnit;

    /**
     * The constructor for incremental encoders
     *
     * @param hwMap               the hardwareMap
     * @param id                  the ID of the encoder as configured
     * @param countsPerRevolution the number of encoder ticks per full revolution, aka cycles per revolution
     */
    public IncrementalEncoder(HardwareMap hwMap, String id, double countsPerRevolution, AngleUnit angleUnit) {
        this(hwMap.get(DcMotorEx.class, id), countsPerRevolution, angleUnit);
    }


    /**
     * The constructor for incremental encoders
     *
     * @param encoder             the DcMotor which encoder is bound to
     * @param countsPerRevolution the number of encoder ticks per full revolution, aka cycles per revolution
     */
    public IncrementalEncoder(DcMotorEx encoder, double countsPerRevolution, AngleUnit angleUnit) {
        this.encoder = encoder;
        this.cpr = countsPerRevolution;
        this.angleUnit = angleUnit;
    }

    /**
     * @return the current position of the encoder
     */
    public int getPosition() {
        int currentPosition = encoder.getCurrentPosition();
        return getDirectionMultiplier() * currentPosition - (int) offset;
    }

    public double getRevolutions() {
        return getPosition() / cpr;
    }


    public int angleToTicks(double angle) {
        return (int) (angle * cpr / MathUtils.returnMaxForAngleUnit(angleUnit));
    }

    //no normalizing happening here
    public double getAngleUnnormalized() {
        return getRevolutions() * MathUtils.returnMaxForAngleUnit(angleUnit);
    }

    public IncrementalEncoder setAngle(double angle) {
        int rawPosition = encoder.getCurrentPosition();
        offset = getDirectionMultiplier() * rawPosition - angleToTicks(angle);
        return this;
    }


    public String getDeviceType() {
        return "Incremental Encoder; " + encoder.getDeviceName();
    }

    public int getDirectionMultiplier() {
        return direction.getMultiplier();
    }

    public IncrementalEncoder setReversed(boolean reversed) {
        direction = reversed ? RotationDirection.REVERSE : RotationDirection.FORWARD;
        return this;
    }


    //sets encoder value to 0
    public IncrementalEncoder zero() {
        this.setAngle(0);
        return this;
    }

    //gets direction
    public RotationDirection getDirection() {
        return direction;
    }

    //gets whether its reversed
    public boolean getReversed() {
        return direction == RotationDirection.REVERSE;
    }

    //gets the angle
    public AngleUnit getAngleUnit() {
        return angleUnit;
    }

    //different from resetting offset, directly resets motor count to zero
    public void resetEncoder(){
        encoder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        encoder.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        offset = 0;
    }
    public void resetOffset() {
        this.offset = 0;
    }

    private static double secondsNow() {
        return System.nanoTime() / 1E9;
    }
}
