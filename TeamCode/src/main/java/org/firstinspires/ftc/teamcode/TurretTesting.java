package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.seattlesolvers.solverslib.hardware.motors.Motor;
import com.seattlesolvers.solverslib.hardware.motors.MotorGroup;
import com.seattlesolvers.solverslib.hardware.servos.ServoEx;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.constants.ConfigNames;
import org.firstinspires.ftc.teamcode.hardware.MotorEx;

@Config
@TeleOp(name = "ShooterTurretTesting")
public class TurretTesting extends OpMode {
    ServoEx servoLeft;
    ServoEx servoRight;
    MotorEx shooterLeft;
    MotorEx shooterRight;

    public static double position1 = 0.5;
    public static double position2 = 1;
    public static boolean SERVO_LEFT_INVERTED = false;
    public static boolean SERVO_RIGHT_INVERTED = false;
    public static boolean MOTOR_LEFT_INVERTED = true;
    public static boolean MOTOR_RIGHT_INVERTED = false;
    public static double[] pidCoefficients = new double[] {0.003, 0.0, 0.0};
    public static double[] feedforwardCoefficients = new double[]{0.0291871, 0.000355203, 0};

    public static double offset = 0.02;
    MotorGroup motorGroup;
    public static boolean usePIDCoefficients = true;
    public static boolean useFeedforwardCoefficients = true;
    public static double targetVelocity = 1500;
    Telemetry multipleTelemetry;
    public static double power = 0.5;

    @Override
    public void init() {
        multipleTelemetry = new MultipleTelemetry(
                telemetry,
                FtcDashboard.getInstance().getTelemetry()
        );
        servoLeft = new ServoEx(hardwareMap, ConfigNames.turretServo1);
        servoRight = new ServoEx(hardwareMap, ConfigNames.turretServo2);
        servoLeft.setInverted(SERVO_LEFT_INVERTED);
        servoRight.setInverted(SERVO_RIGHT_INVERTED);

        shooterLeft = new MotorEx(hardwareMap, ConfigNames.shooterLeft);
        shooterRight = new MotorEx(hardwareMap, ConfigNames.shooterRight);
        shooterLeft.setInverted(MOTOR_LEFT_INVERTED);
        shooterRight.setInverted(MOTOR_RIGHT_INVERTED);
        shooterLeft.motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        shooterRight.motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
//        motorGroup = new MotorGroup(shooterRight, shooterLeft);
//        motorGroup.setRunMode(Motor.RunMode.VelocityControl);
//
//        if(usePIDCoefficients){
//            motorGroup.setVeloCoefficients(pidCoefficients[0], pidCoefficients[1], pidCoefficients[2]);
//        }
//        if(useFeedforwardCoefficients){
//            motorGroup.setFeedforwardCoefficients(feedforwardCoefficients[0], feedforwardCoefficients[1], feedforwardCoefficients[2]);
//        }

    }

    @Override
    public void loop() {
        if(gamepad1.leftBumperWasPressed()) {
            servoLeft.set(position1);
            servoRight.set(position1 + offset);
        }
        if(gamepad1.rightBumperWasPressed()) {
            servoLeft.set(position2);
            servoRight.set(position2 + offset);
        }
        shooterLeft.setPower(power);
        shooterRight.setPower(power);
//        motorGroup.set(targetVelocity);

//        multipleTelemetry.addData("Shooter Velocity", shooterLeft.getVelocity());
//        multipleTelemetry.update();
    }
}
