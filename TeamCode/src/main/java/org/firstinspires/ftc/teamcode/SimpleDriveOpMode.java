package org.firstinspires.ftc.teamcode;

//import com.bylazar.telemetry.PanelsTelemetry;
//import com.bylazar.telemetry.TelemetryManager;
import com.acmerobotics.dashboard.config.Config;
import com.pedropathing.follower.Follower;
import com.pedropathing.math.Pose;
import com.pedropathing.utils.Timer;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.pedropathing.drivetrain.DrivePowers;
import com.pedropathing.follower.ManualDrive;
import com.seattlesolvers.solverslib.hardware.motors.Motor;
import com.seattlesolvers.solverslib.hardware.motors.MotorEx;

import org.firstinspires.ftc.teamcode.constants.Constants;
import org.firstinspires.ftc.teamcode.constants.ConfigNames;

import java.util.concurrent.TimeUnit;


@TeleOp(name = "Intake Drive")
@Config
//@Configurable
public class SimpleDriveOpMode extends OpMode {
    private Follower follower;
    boolean fieldCentric = false;
    DrivePowers powers;
    Pose robotPose;
    Timer timer;
    double previousTime = 0;

    //-------------INTAKE---------------------------------
    MotorEx motor;
    public static double power = 1.0;
    public static boolean useVelocity = false;
    public static double velocity = 1000;
    public static double kP = 1.0;
    public static double kI = 0.0;
    public static double kD = 0.0;
    public static boolean usePIDFCoefficients = false;
    public static double kS = 0;
    public static double kV = 0.0;
    public static double kA = 0.0;
    public static boolean isInverted = true;
    public static boolean useFeedforwardCoefficients = false;
    @Override
    public void init() {
        follower = Constants.create(hardwareMap);
        timer = new Timer();

        motor = new MotorEx(hardwareMap, ConfigNames.testMotor);
        motor.setInverted(isInverted);
        timer = new Timer();
        updateFeedforward();
        updatePIDF();
    }

    @Override
    public void loop() {
        if(gamepad1.leftBumperWasPressed()){
            fieldCentric = !fieldCentric;
        }

        if(!fieldCentric) {
            double forward = -gamepad1.left_stick_y;
            double lateral = -gamepad1.left_stick_x;
            double rotation = -gamepad1.right_stick_x;
            follower.manual(forward, lateral, rotation);
        } else{
             powers = ManualDrive.fieldCentric(
                    -gamepad1.left_stick_y,
                    -gamepad1.left_stick_x,
                    -gamepad1.right_stick_x,
                    follower.pose().heading()
            );
            follower.manual(powers);
        }

        if(gamepad1.left_trigger_pressed){
            fieldCentric = !fieldCentric;
        }
        follower.update();

        robotPose = follower.pose();
        updateFeedforward();
        updatePIDF();
        if(!useVelocity) {
            motor.setRunMode(Motor.RunMode.RawPower);
            motor.set(power);
        }
        else{
            motor.setRunMode(Motor.RunMode.VelocityControl);
            motor.setVelocity(velocity);
        }

        telemetry.addLine("---------------Intake ----------------");
        telemetry.addData("Target Power", power);
        telemetry.addData("Target Velocity", velocity);
        telemetry.addData("RunMode", useVelocity ? Motor.RunMode.VelocityControl : Motor.RunMode.RawPower);
        telemetry.addData("Power", motor.getRawPower());
        telemetry.addData("Velocity ", motor.getVelocity());
        telemetry.addLine("---------------Follower ----------------");
        telemetry.addData("Robot X", robotPose.x());
        telemetry.addData("Robot Y", robotPose.y());
        telemetry.addData("Robot X", Math.toDegrees(robotPose.heading()));
        telemetry.addData("Field Centric", fieldCentric);
        double deltaTime = timer.get(TimeUnit.MILLISECONDS) - previousTime;
        telemetry.addData("Update Rate", 1000/ deltaTime);


        telemetry.update();
        previousTime = timer.get(TimeUnit.MILLISECONDS);
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
}
