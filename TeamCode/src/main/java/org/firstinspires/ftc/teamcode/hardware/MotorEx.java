package org.firstinspires.ftc.teamcode.hardware;

import androidx.annotation.NonNull;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.seattlesolvers.solverslib.hardware.motors.Motor;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;

/**
 * Modified version of SolversLib MotorEx class
 * Only change is allowing for Voltage Compensation
 *
 *
 */
public class MotorEx extends Motor {
    public static double REFERENCE_VOLTAGE = 12.5;
    public DcMotorEx motorEx;
    private double cachingTolerance = 0.0001;

    /**
     * Constructs the instance motor for the wrapper
     *
     * @param hMap the hardware map from the OpMode
     * @param id   the device id from the RC config
     */
    public MotorEx(@NonNull HardwareMap hMap, String id) {
        this(hMap, id, GoBILDA.NONE);
        ACHIEVABLE_MAX_TICKS_PER_SECOND = motorEx.getMotorType().getAchieveableMaxTicksPerSecond();
    }

    /**
     * Constructs the instance motor for the wrapper
     *
     * @param hMap        the hardware map from the OpMode
     * @param id          the device id from the RC config
     * @param gobildaType the type of gobilda 5202 series motor being used
     */
    public MotorEx(@NonNull HardwareMap hMap, String id, @NonNull GoBILDA gobildaType) {
        super(hMap, id, gobildaType);
        motorEx = (DcMotorEx) super.motor;
    }

    /**
     * Constructs an instance motor for the wrapper
     *
     * @param hMap the hardware map from the OpMode
     * @param id   the device id from the RC config
     * @param cpr  the counts per revolution of the motor
     * @param rpm  the revolutions per minute of the motor
     */
    public MotorEx(@NonNull HardwareMap hMap, String id, double cpr, double rpm) {
        super(hMap, id, cpr, rpm);
        motorEx = (DcMotorEx) super.motor;
    }

    //MODIFIED set function correctly
    public void set(double output, double currVoltage) {
        if (runmode == RunMode.VelocityControl) {
            setPower((veloController.calculate(getCorrectedVelocity(), output) + feedforward.calculate(output, getAcceleration())) * (REFERENCE_VOLTAGE) / (currVoltage));
        } else if (runmode == RunMode.PositionControl) {
            double error = positionController.calculate(encoder.getPosition());
            setPower(output * error);
        } else {
            setPower(output);
        }
    }

    @Override
    // default 12.5V
    public void set(double output) {
        set(output, REFERENCE_VOLTAGE);
    }


    /**
     * @return the velocity of the motor in ticks per second
     */
    @Override
    public double getVelocity() {
        return motorEx.getVelocity();
    }

    /**
     * @return the acceleration of the motor in ticks per second squared
     */
    public double getAcceleration() {
        return encoder.getAcceleration();
    }

    @Override
    public String getDeviceType() {
        return "Extended " + super.getDeviceType();
    }

    /**
     * @return the caching tolerance of the motor before it writes a new power to the motor
     */
    public double getCachingTolerance() {
        return cachingTolerance;
    }

    /**
     * @param cachingTolerance the new caching tolerance between motor writes
     * @return this object for chaining purposes
     */
    public MotorEx setCachingTolerance(double cachingTolerance) {
        this.cachingTolerance = cachingTolerance;
        return this;
    }

    /**
     * @param power power to be assigned to the motor if difference is greater than caching tolerance or if power is exactly 0
     */
    public void setPower(double power) {
        if ((Math.abs(power - motorEx.getPower()) > cachingTolerance) || (power == 0 && motorEx.getPower() != 0)) {
            lastPower = power;
            motorEx.setPower(power);
        }
    }

    /**
     * Gets the current in the specified unit
     * @param currentUnit the unit to get the current in
     * @return the current in the specified unit
     */
    public double getCurrent(CurrentUnit currentUnit) {
        return motorEx.getCurrent(currentUnit);
    }

    /**
     * Gets the current alert in the specified unit
     * @param currentUnit the unit to get the current alert in
     * @return the current alert in the specified unit
     */
    public double getCurrentAlert(CurrentUnit currentUnit) {
        return motorEx.getCurrentAlert(currentUnit);
    }

    /**
     * Sets the current alert in the specified unit
     * @param current the current alert to set
     * @param unit the unit to set the current alert in
     */
    public MotorEx setCurrentAlert(double current, CurrentUnit unit) {
        motorEx.setCurrentAlert(current, unit);
        return this;
    }

    /**
     * Checks if the motor is over the current limit
     * @return true if the motor is over the current limit, false otherwise
     */
    public boolean isOverCurrent() {
        return motorEx.isOverCurrent();
    }
}