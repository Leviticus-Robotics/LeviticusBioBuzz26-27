package org.firstinspires.ftc.teamcode.subsystems;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.config.Config;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.hardware.MotorEx;

import dev.frozenmilk.dairy.mercurial.processes.Channel;
import dev.frozenmilk.dairy.mercurial.processes.SwitchedStateMachine;

//Simple Motor State machine including velocity/power components

/** Motor Subsystem
 * Velocity, Raw Power Modes
 * Uses: 1 Motor (encoder needed if using velocity control)
 */
public abstract class MotorSubsystem extends SwitchedStateMachine{
    protected MotorEx motor;

    protected Telemetry.Item currentTelemetry;
    protected Telemetry.Item modeTelemtry;
    protected Mode offMode;
    protected double currentVoltage;
    public enum VelocityStatus{
        AtTarget
    }

    protected Mode fixedVelocityMode(double targetVelocity){
        return mode(self -> {
                    modeTelemtry.setValue("Fixed Velocity (" + targetVelocity + ")");
                    motor.setRunMode(MotorEx.RunMode.VelocityControl);
                    motor.set(targetVelocity, currentVoltage > 0 && useVoltageCompensation() ? currentVoltage : MotorEx.REFERENCE_VOLTAGE);
                    return self;
                });
    }

    public boolean useVoltageCompensation(){
        return true;
    }

    public void fixedVelocity(double targetVelocity){
        switchTo(fixedVelocityMode(targetVelocity));
    }

    protected Mode fixedPowerMode(double power){//parametrized so it keeps holding the velocity
        return mode(
                self -> {
                    modeTelemtry.setValue("Fixed Power (" + power + ")");
                    motor.setRunMode(MotorEx.RunMode.RawPower);
                    motor.set(power);
                    return self;
                }
        );
    }


    public void fixedPower(double power){
        switchTo(fixedPowerMode(power));
    }


    public void off(){
        switchTo(offMode);
    }

    @NonNull
    @Override
    protected Mode init() {
        return offMode;
    }

    public double getVelocity(){//returns velocity in ticks/sec
        return motor.getVelocity();
    }

    public boolean isAtVelocity(double targetVelocity, double tolerance){
        return Math.abs(getVelocity() - targetVelocity) <= Math.abs(tolerance);
    }

    public double getError(double targetVelocity){//keep sign here
        return getVelocity() - targetVelocity;
    }

    public void updateVoltage(double voltage){
        this.currentVoltage = voltage;
    }

}
