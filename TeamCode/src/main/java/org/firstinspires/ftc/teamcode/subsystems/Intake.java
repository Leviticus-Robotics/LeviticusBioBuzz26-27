package org.firstinspires.ftc.teamcode.subsystems;


import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.seattlesolvers.solverslib.hardware.motors.Motor;

import org.firstinspires.ftc.teamcode.constants.ConfigNames;
import org.firstinspires.ftc.teamcode.hardware.MotorEx;

import dev.frozenmilk.dairy.mercurial.ftc.Context;
import dev.frozenmilk.dairy.mercurial.processes.Channel;

/** Intake
 * Extends from motor subsystem
 * Uses: 1 motor (intake)
 */
@Config
public class Intake extends MotorSubsystem {
    public static double[] pidCoefficients = new double[] {1.0, 0.0, 0.0};
    public static double[] feedforwardCoefficients = new double[]{0, 0, 0};
    public static boolean usePIDCoefficients = false;
    public static boolean useFeedforwardCoefficients = false;
    public static boolean voltageCompesnation = false;
    boolean breaker1Detection;
    boolean breaker2Detection;
    boolean intakeSpot1Detected;
    boolean intakeSpot2Detected;
    public Intake(Context ctx){
        stateTelemetry = ctx.telemetry().addData("Intake/State", "unknown");
        motor = new MotorEx(ctx.hardwareMap(), ConfigNames.intake);
        motor.motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motor.setInverted(true);
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

    public void updateIntakeSpot1(boolean detected){
        intakeSpot1Detected = detected;
    }

    public void updateIntakeSpot2(boolean detected){
        intakeSpot2Detected = detected;
    }
    public boolean transferTwoBallsDetected(){
        return breaker1Detection && breaker2Detection;
    }

    public boolean intakeTwoBallsDetected(){
        return intakeSpot1Detected && intakeSpot2Detected;
    }

    public int numBallsDetected(){
        int numBalls = 0;
        if(intakeSpot1Detected) numBalls++;
        if(intakeSpot2Detected) numBalls++;
        return numBalls;
    }

    @Override
    public boolean useVoltageCompensation(){
        return voltageCompesnation;
    }



    //this runs intake until both intake detects two balls and transfer detects two balls
    public Mode runVelTillDetectionMode(double targetVelocity){
        return mode(self -> {
            motor.setRunMode(MotorEx.RunMode.VelocityControl);
            motor.set(targetVelocity, currentVoltage > 0 && useVoltageCompensation() ? currentVoltage : MotorEx.REFERENCE_VOLTAGE);
            //turn intake off if transfer & intake has 4 balls
            if(transferTwoBallsDetected() && intakeTwoBallsDetected()){
                switchTo(offMode);
            }
            return self;
        });
    }

    public Mode runPowerTillDetectionMode(double power){
        return mode(self -> {
            motor.setRunMode(MotorEx.RunMode.RawPower);
            motor.set(power);
            //turn intake off if transfer & intake has 4 balls
            if(transferTwoBallsDetected() && intakeTwoBallsDetected()){
                switchTo(offMode);
            }
            return self;
        });
    }

    public void runVelTillDetection(double targetVelocity){
        switchTo(runVelTillDetectionMode(targetVelocity));
    }

    public void runPowerTillDetection(double power){
        switchTo(runPowerTillDetectionMode(power));
    }
}
