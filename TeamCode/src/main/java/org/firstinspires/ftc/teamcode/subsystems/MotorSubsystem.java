package org.firstinspires.ftc.teamcode.subsystems;

import androidx.annotation.NonNull;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.hardware.MotorEx;

import dev.frozenmilk.dairy.mercurial.processes.Channel;
import dev.frozenmilk.dairy.mercurial.processes.SwitchedStateMachine;

//Simple Motor State machine including velocity/power components
public abstract class MotorSubsystem extends SwitchedStateMachine{
    protected MotorEx motor;

    protected Telemetry.Item stateTelemetry;
    protected Mode offMode;
    public enum VelocityStatus{
        AtTarget
    }

    protected Mode fixedVelocityMode(double targetVelocity, double tolerance, Channel<VelocityStatus> resultChannel){
        boolean[] notified = new boolean[1];
        return entranceMode(from -> {
                    stateTelemetry.setValue("Fixed Velocity (" + targetVelocity + ")");
                    motor.setRunMode(MotorEx.RunMode.VelocityControl);
                },
                self -> {
                    motor.set(targetVelocity);
                    if (resultChannel != null && notified[0] == false) {
                        if (isAtVelocity(targetVelocity, tolerance)) {
                            notified[0] = true;
                            resultChannel.sendIfEmpty(VelocityStatus.AtTarget);
                        }
                    }
                    return self;
                });
    }


    public void fixedVelocity(double targetVelocity){
        switchTo(fixedVelocityMode(targetVelocity, 0, null));
    }

    public void fixedVelocity(double targetVelocity, double tolerance) {
        switchTo(fixedVelocityMode(targetVelocity, tolerance, null));
    }

    public void fixedVelocity(double targetVelocity, double tolerance, Channel<VelocityStatus> resultChannel) {
        if (resultChannel != null) {
            switchTo(fixedVelocityMode(targetVelocity, tolerance, resultChannel));
        } else{
            switchTo(fixedVelocityMode(targetVelocity, tolerance, null));
        }
    }

    protected Mode fixedPowerMode(double power){//parametrized so it keeps holding the velocity
        return entranceMode(
                from -> {
                    stateTelemetry.setValue("Fixed Power (" + power + ")");
                    motor.setRunMode(MotorEx.RunMode.RawPower);
                    motor.set(power);
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


}
