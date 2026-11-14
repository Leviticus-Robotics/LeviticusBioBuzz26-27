package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.constants.ConfigNames;
import org.firstinspires.ftc.teamcode.hardware.MotorEx;

import dev.frozenmilk.dairy.mercurial.ftc.Context;
import dev.frozenmilk.dairy.mercurial.processes.Channel;

/** Transfer
 * Extends from motor subsystem
 * Uses 1 motor (transfer)
 */
@Config
public class Transfer extends MotorSubsystem {
    public static double[] pidCoefficients = new double[] {1.0, 0.0, 0.0};
    public static double[] feedforwardCoefficients = new double[]{0, 0, 0};
    public static boolean usePIDCoefficients = false;
    public static boolean useFeedforwardCoefficients = false;
    public static boolean voltageCompesnation = false;

    boolean breaker1Detection = false;
    boolean breaker2Detection = false;
    boolean prevTransferDetected = false;


    public Transfer(Context ctx){
        stateTelemetry = ctx.telemetry().addData("Transfer/State", "unknown");
        motor = new MotorEx(ctx.hardwareMap(), ConfigNames.transfer);
        motor.motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motor.setInverted(false);
        if(usePIDCoefficients){
            motor.setVeloCoefficients(pidCoefficients[0], pidCoefficients[1], pidCoefficients[2]);
        }
        if(useFeedforwardCoefficients){
            motor.setFeedforwardCoefficients(feedforwardCoefficients[0], feedforwardCoefficients[1], feedforwardCoefficients[2]);
        }
        offMode = entranceMode(from ->{
            motor.setRunMode(MotorEx.RunMode.RawPower);
            motor.set(0);
            stateTelemetry.setValue("Off");
        });

    }
    public void updateBreaker1(boolean detected){
        breaker1Detection = detected;
    }

    public void updateBreaker2(boolean detected){
        breaker2Detection = detected;
    }

    public int numBallsDetected(){
        int count = 0;
        if(breaker1Detection) count++;
        if(breaker2Detection) count++;
        return count;
    }

    private boolean detectTwoBalls(){
        return breaker1Detection && breaker2Detection;
    }


    //run target velocity on till detect two balls, then turn off transfer
    public Mode runVelTillDetectionMode(double targetVelocity){
        return mode(self -> {
            stateTelemetry.setValue("Fixed Velocity till Detect (" + targetVelocity + ")");
            motor.setRunMode(MotorEx.RunMode.VelocityControl);
            motor.set(targetVelocity, currentVoltage > 0 && useVoltageCompensation() ? currentVoltage : MotorEx.REFERENCE_VOLTAGE);
            //keep it simple for now assuming breakers working
            boolean currDetected = detectTwoBalls();

            if(currDetected){
                motor.set(0);
                stateTelemetry.setValue("Transfer Balls Detected");
            } else{
                motor.set(targetVelocity);
                stateTelemetry.setValue("Transfer on");
            }
            prevTransferDetected = currDetected;

            return self;
        });
    }

    public Mode runPowerTillDetectionMode(double power){
        return mode(self -> {
            stateTelemetry.setValue("Fixed Power till Detect (" + power + ")");
            motor.setRunMode(MotorEx.RunMode.RawPower);
            motor.set(power);
            //keep it simple for now assuming breakers working
            boolean currDetected = detectTwoBalls();

            if(currDetected){
                motor.set(0);
                stateTelemetry.setValue("Transfer Balls Detected");
            } else{
                motor.set(power);
                stateTelemetry.setValue("Transfer on");
            }

            prevTransferDetected = currDetected;

            return self;
        });
    }
    public void runPowerTillDetection(double power){
        switchTo(runPowerTillDetectionMode(power));
    }
    public void runVelocityTillDetection(double velocity){
        switchTo(runVelTillDetectionMode(velocity));
    }

    @Override
    public boolean useVoltageCompensation(){
        return voltageCompesnation;
    }





}
