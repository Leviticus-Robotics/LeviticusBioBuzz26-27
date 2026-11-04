package org.firstinspires.ftc.teamcode.subsystems;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.constants.ConfigNames;
import org.firstinspires.ftc.teamcode.hardware.MotorEx;

import java.util.concurrent.atomic.AtomicBoolean;

import dev.frozenmilk.dairy.mercurial.ftc.Context;
import dev.frozenmilk.dairy.mercurial.processes.Channel;
import dev.frozenmilk.dairy.mercurial.processes.SwitchedStateMachine;

@Config
public class Intake extends SwitchedStateMachine {
    private MotorEx intake;
    public static double[] pidCoefficients = new double[] {1.0, 0.0, 0.0};
    public static double[] feedforwardCoefficients = new double[]{0, 0, 0};
    public static boolean usePIDCoefficients = false;
    public static boolean useFeedforwardCoefficients = false;
    private Telemetry.Item stateTelemetry;
    private Mode off;

    public Intake(Context ctx){
        stateTelemetry = ctx.telemetry().addData("Intake/State", "unknown");
        intake = new MotorEx(ctx.hardwareMap(), ConfigNames.intake);
        intake.motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        intake.setInverted(true);
        if(usePIDCoefficients){
            intake.setVeloCoefficients(pidCoefficients[0], pidCoefficients[1], pidCoefficients[2]);
        }
        if(useFeedforwardCoefficients){
            intake.setFeedforwardCoefficients(feedforwardCoefficients[0], feedforwardCoefficients[1], feedforwardCoefficients[2]);
        }
        off = entranceMode(from ->{
                intake.setRunMode(MotorEx.RunMode.RawPower);
                intake.set(0);
                stateTelemetry.setValue("Off");
        });


    }

    private Mode fixedVelocityMode(double targetVelocity, double tolerance, Channel<VelocityStatus> resultChannel){
        boolean[] notified = new boolean[1];
        return entranceMode(from -> {
            stateTelemetry.setValue("Fixed Velocity (" + targetVelocity + ")");
            intake.setRunMode(MotorEx.RunMode.VelocityControl);
        },
        self -> {
            intake.set(targetVelocity);
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
            fixedVelocity(targetVelocity, tolerance, null);
        }
    }

    private Mode fixedPowerMode(double power){//parametrized so it keeps holding the velocity
        return entranceMode(
                from -> {
                    stateTelemetry.setValue("Fixed Power (" + power + ")");
                    intake.setRunMode(MotorEx.RunMode.RawPower);
                    intake.set(power);
                }
        );
    }


    public void fixedPower(double power){
        switchTo(fixedPowerMode(power));
    }


    public void off(){
        switchTo(off);
    }

    @NonNull
    @Override
    protected Mode init() {
        return off;
    }

    public double getVelocity(){//returns velocity in ticks/sec
        return intake.getVelocity();
    }

    public boolean isAtVelocity(double targetVelocity, double tolerance){
        return Math.abs(getVelocity() - targetVelocity) <= Math.abs(tolerance);
    }

    public double getError(double targetVelocity){//keep sign here
        return getVelocity() - targetVelocity;
    }

    public enum VelocityStatus{
        AtTarget
    }
}
