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
    boolean transfer1;
    boolean transfer2;
    boolean intake1;
    boolean intake2;
    private Telemetry.Line startLine;
    private Telemetry.Item extraStateTelemetry;
    private Telemetry.Item intakePowerTelemery;
    private Telemetry.Line endBreakLine;
    public static double POST_DETECTION_WAIT_MS = 0;//wait X ms in runTillTransferDetect() modes
    private final ElapsedTime detectionTimer = new ElapsedTime();
    boolean detectionDelayStated = false;
    public Intake(Context ctx){
        startLine = ctx.telemetry().addLine("Intake");
        modeTelemtry = ctx.telemetry().addData("Mode", "unknown");
        extraStateTelemetry = ctx.telemetry().addData("Extra info", "Unknown");
        intakePowerTelemery = ctx.telemetry().addData("Transfer Power", "unknown");
        currentTelemetry =  ctx.telemetry().addData("Current(Amps)", "unknown");
        endBreakLine = ctx.telemetry().addLine("-----------------------");

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
                modeTelemtry.setValue("Off");
        });

    }

    public void updateTransfer1(boolean detected){
        transfer1 = detected;
    }

    public void updateTransfer2(boolean detected){
        transfer2 = detected;
    }

    public void updateIntake1(boolean detected){
        intake1 = detected;

    }

    public void updateIntake2(boolean detected){
        intake2 = detected;
    }
    public boolean transferTwoBallsDetected(){
        return transfer1 && transfer2;
    }

    public boolean intakeTwoBallsDetected(){
        return intake1 && intake2;
    }

    //only from the intake
    public int numBallsDetected(){
        int numBalls = 0;
        if(intake1) numBalls++;
        if(intake2) numBalls++;
        return numBalls;
    }

    @Override
    protected Mode fixedPowerMode(double power){//parametrized so it keeps holding the velocity
        return mode(self -> {
            modeTelemtry.setValue("Fixed Power (" + power + ")");
            motor.setRunMode(MotorEx.RunMode.RawPower);
            // goes at half speed after two balls detect
//        }, self -> {
//                if(!transferTwoBallsDetected()){
            motor.set(power);
            extraStateTelemetry.setValue("Full Power");
//                }
//                else{
//                    motor.set(power / 2); // goes at half speed after two balls detect
//                    extraStateTelemetry.setValue("Half Power (transfer balls detected)");
//                }
                return self;
        });
    }

    @Override
    public void fixedPower(double power){
        switchTo(fixedPowerMode(power));
    }

    @Override
    public boolean useVoltageCompensation(){
        return voltageCompesnation;
    }



    //this runs intake until both intake detects two balls and transfer detects two balls
    //continaully checking for ball detections, in case where no more balls left, then it stops temporarilyd

    public void runPowerTillDetection(double power){
        switchTo(runTillDetectionMode(power, false));
    }
    public void runVelocityTillDetection(double velocity){
        switchTo(runTillDetectionMode(velocity, true));
    }


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
                    intakePowerTelemery.setValue(motor.get());
                    return self;
                }
        );
    }

    private boolean shouldTransferContinue(){
//        if(!detectionDelayStated && detect4Balls()){
//            detectionDelayStated = true;
//            detectionTimer.reset();
//        }//wait X seconds after first ball detection before turning off transfer motors
//        return !detectionDelayStated || detectionTimer.milliseconds() < POST_DETECTION_WAIT_MS;
        return !detect4Balls();
    }


    public boolean detect4Balls(){
        return intakeTwoBallsDetected() && transferTwoBallsDetected();
    }

    public void updateCurrent(){
        currentTelemetry.setValue(motor.getCurrent(CurrentUnit.AMPS));
    }
}
