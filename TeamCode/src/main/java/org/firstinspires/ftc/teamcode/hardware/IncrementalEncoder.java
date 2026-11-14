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
    private int lastPosition;
    private double positionTimestamp;
    private double velocityTimestamp;
    private double veloEstimate, dpp, accel, lastVelo;

    private double offset = 0.0;
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
        resetEncoder();
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
        dpp = 1;
        lastPosition = encoder.getCurrentPosition();
        veloEstimate = 0;
        lastVelo = encoder.getVelocity();
        positionTimestamp = secondsNow();
        velocityTimestamp = positionTimestamp;
    }

    /**
     * @return the current position of the encoder
     */
    public int getPosition() {
        int currentPosition = encoder.getCurrentPosition();
        double currentTime = (double) System.nanoTime() / 1E9;
        double dt = currentTime - positionTimestamp;
        if (dt > 0) {
            veloEstimate = (currentPosition - lastPosition) / dt;
        }
        lastPosition = currentPosition;
        positionTimestamp = currentTime;
        return getDirectionMultiplier() * currentPosition - (int) offset;
    }

    /**
     * @return the distance per pulse
     */
    public double getDpp() {
        return dpp;
    }

    /**
     * @return the distance traveled by the encoder
     */
    public double getDistance() {
        return dpp * getPosition();
    }

    /**
     * @return the velocity of the encoder adjusted to account for the distance per pulse
     */
    public double getRate() {
        return dpp * ((DcMotorEx) encoder).getVelocity();
    }

    /**
     * Sets the distance per pulse of the encoder.
     *
     * @param distancePerPulse the desired distance per pulse (in units per tick)
     */
    public IncrementalEncoder setDistancePerPulse(double distancePerPulse) {
        dpp = distancePerPulse;
        return this;
    }

    /**
     * @return the number of revolutions turned by the encoder
     */
    public double getRevolutions() {
        return getPosition() / cpr;
    }

    /**
     * @return the raw velocity of the motor reported by the encoder
     */
    public double getRawVelocity() {
        double velo = ((DcMotorEx) encoder).getVelocity();
        double currentTime = secondsNow();
        double dt = currentTime - velocityTimestamp;
        if (dt > 0) {
            accel = (velo - lastVelo) / dt;
        }
        lastVelo = velo;
        velocityTimestamp = currentTime;
        return velo;
    }

    /**
     * @return the estimated acceleration of the motor in ticks per second squared
     */
    public double getAcceleration() {
        return accel;
    }

    private final static int CPS_STEP = 0x10000;

    /**
     * Corrects for velocity overflow
     *
     * @return the corrected velocity
     */
    public double getCorrectedVelocity() {
        double real = getRawVelocity();
        while (Math.abs(veloEstimate - real) > CPS_STEP / 2.0) {
            real += Math.signum(veloEstimate - real) * CPS_STEP;
        }
        return real;
    }

    public int angleToTicks(double angle) {
        return (int) (angle * cpr / MathUtils.returnMaxForAngleUnit(angleUnit));
    }

    public double getAngleUnnormalized() {
        return getRevolutions() * MathUtils.returnMaxForAngleUnit(angleUnit);
    }

    public IncrementalEncoder setAngle(double angle) {
        offset = getPosition() - angleToTicks(angle);
        return this;
    }

    public double getAngle() {
        return MathUtils.normalizeAngle(
                getAngleUnnormalized(),
                true,
                angleUnit
        );
    }

    public void disable() {
        // "take no action" (encoder.close() call in SDK)
    }

    public String getDeviceType() {
        return "Incremental Encoder; " + encoder.getDeviceName();
    }

    public int getDirectionMultiplier() {
        return direction.getMultiplier();
    }

    public IncrementalEncoder setDirection(RotationDirection direction) {
        this.direction = direction;
        return this;
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
        lastPosition = 0;
        veloEstimate = 0;
        lastVelo = 0;
        accel = 0;
        positionTimestamp = secondsNow();
        velocityTimestamp = positionTimestamp;
        offset = 0;
    }
    public void resetOffset() {
        this.offset = 0;
    }

    private static double secondsNow() {
        return System.nanoTime() / 1E9;
    }
}
