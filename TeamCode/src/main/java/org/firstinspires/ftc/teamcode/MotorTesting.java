package org.firstinspires.ftc.teamcode;

//import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.config.Config;
import com.pedropathing.utils.Timer;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.seattlesolvers.solverslib.hardware.motors.Motor;
import com.seattlesolvers.solverslib.hardware.motors.MotorEx;

import org.firstinspires.ftc.teamcode.constants.ConfigNames;

import java.util.concurrent.TimeUnit;

@Deprecated
@TeleOp(name = "Motor Testing")
@Config
public class MotorTesting extends OpMode {
    MotorEx motor;
    public static double power = 1;
    public static boolean useVelocity = false;
    public static double velocity = 1000;
    Timer timer;
    double previousTime = 0;
    public static double kP = 1.0;
    public static double kI = 0.0;
    public static double kD = 0.0;
    public static boolean usePIDFCoefficients = false;
    public static double kS = 0;
    public static double kV = 0.0;
    public static double kA = 0.0;
    public  boolean isInverted = true;
    public static boolean useFeedforwardCoefficients = false;
    @Override
    public void init() {
        motor = new MotorEx(hardwareMap, ConfigNames.testMotor);
        motor.setInverted(isInverted);
        timer = new Timer();
        updateFeedforward();
        updatePIDF();
    }

    @Override
    public void loop() {
        updateFeedforward();
        updatePIDF();


        if(gamepad1.left_bumper){
            motor.set(power);
        }
        if(gamepad1.right_bumper){
            motor.set(0);
        }
        if(gamepad1.xWasPressed()){
            isInverted = !isInverted;
            motor.setInverted(isInverted);
        }

        telemetry.addData("Target Power", power);
        telemetry.addData("Target Velocity", velocity);
        telemetry.addData("RunMode", useVelocity ? Motor.RunMode.VelocityControl : Motor.RunMode.RawPower);
        telemetry.addData("Power", motor.getRawPower());
        telemetry.addData("Velocity ", motor.getVelocity());
        telemetry.addData("Update Rate", 1000.0 / (timer.get(TimeUnit.MILLISECONDS) - previousTime));
        previousTime = timer.get(TimeUnit.MILLISECONDS);

        telemetry.update();

    }

    public void updateFeedforward(){
        if(useFeedforwardCoefficients){
            motor.setFeedforwardCoefficients(kS, kV, kA);
        }
    }

    public void updatePIDF(){
        if(usePIDFCoefficients){
            motor.setVeloCoefficients(kP, kI, kD);
        }

    }

    public void setIntakePower(){
        if(!useVelocity) {
            motor.setRunMode(Motor.RunMode.RawPower);
            motor.set(power);
        }
        else{
            motor.setRunMode(Motor.RunMode.VelocityControl);
            motor.setVelocity(velocity);
        }
    }
}
