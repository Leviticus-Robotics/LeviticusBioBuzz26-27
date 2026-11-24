package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.seattlesolvers.solverslib.hardware.motors.Motor;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.teamcode.constants.ConfigNames;
import org.firstinspires.ftc.teamcode.hardware.MotorEx;

import dev.frozenmilk.dairy.mercurial.ftc.Context;

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

    public static double POST_DETECTION_WAIT_MS = 0;//wait X ms in runTillTransferDetect() modes
    private final ElapsedTime detectionTimer = new ElapsedTime();
    boolean detectionDelayStated = false;
    private Telemetry.Line startLine;
    private Telemetry.Item extraStateTelemetry;
    private Telemetry.Item transferPowerTelemetry;
    private Telemetry.Line endBreakLine;

    public Transfer(Context ctx){
        startLine = ctx.telemetry().addLine("Transfer");
        modeTelemtry = ctx.telemetry().addData("State", "unknown");
        extraStateTelemetry = ctx.telemetry().addData("Extra info", "unknown");
        transferPowerTelemetry = ctx.telemetry().addData("Transfer Power", "unknown");
        currentTelemetry =  ctx.telemetry().addData("Current(Amps)", "unknown");
        endBreakLine = ctx.telemetry().addLine("-----------------------");
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
            modeTelemtry.setValue("Off");
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

    private boolean detectFirstBall(){
        return breaker1Detection;
    }


    //run target velocity on till detect two balls, then turn off transfer
    public Mode runTillDetectionMode(double target, boolean velocityRun){
        return entranceMode(self -> {
                detectionDelayStated = false;
                if (velocityRun) {
                    motor.setRunMode(Motor.RunMode.VelocityControl);
                } else {
                    motor.setRunMode(MotorEx.RunMode.RawPower);
                }
                updateCurrent();
            },
            self -> {
                updateCurrent();
                //keep it simple for now assuming breakers working
                if(shouldTransferContinue()){
                    if(velocityRun) {
                        double voltage = currentVoltage > 0 && useVoltageCompensation() ? currentVoltage : MotorEx.REFERENCE_VOLTAGE;
                        motor.set(target, voltage);
                    } else{
                        motor.set(target);
                    }
                    extraStateTelemetry.setValue(detectionDelayStated ? "Detected. Finishing Transfer" : "Transfer on");
                }
                else {
                    motor.set(0);
                    extraStateTelemetry.setValue("Transfer Wait");
                }
                transferPowerTelemetry.setValue(motor.get());
                return self;
            }
        );
    }


    public void runPowerTillDetection(double power){
        switchTo(runTillDetectionMode(power, false));
    }
    public void runVelocityTillDetection(double velocity){
        switchTo(runTillDetectionMode(velocity, true));
    }

    @Override
    public boolean useVoltageCompensation(){
        return voltageCompesnation;
    }

    public MotorEx getMotor(){
        return motor;
    }

    public void updateCurrent(){
        currentTelemetry.setValue(motor.getCurrent(CurrentUnit.AMPS));
    }


    private boolean shouldTransferContinue(){
        if(!detectionDelayStated && detectTwoBalls()){
            detectionDelayStated = true;
            detectionTimer.reset();
        }//wait X seconds after first ball detection before turning off transfer motors
        return !detectionDelayStated || detectionTimer.milliseconds() < POST_DETECTION_WAIT_MS;
    }


}
