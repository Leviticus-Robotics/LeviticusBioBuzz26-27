package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.seattlesolvers.solverslib.hardware.motors.Motor;
import org.firstinspires.ftc.teamcode.hardware.MotorEx;

import org.firstinspires.ftc.teamcode.constants.ConfigNames;

@TeleOp()
@Disabled
public class SimpleShooterTest extends OpMode {
    MotorEx shooter;
    public static double[] pidCoefficients = new double[] {0.0004, 0.0, 0.0};
    public static double[] feedforwardCoefficients = new double[]{0.0291871, 0.000355203, 0};
    @Override
    public void init() {
        shooter = new MotorEx(hardwareMap, ConfigNames.shooter);
        shooter.setRunMode(Motor.RunMode.VelocityControl);
        shooter.setFeedforwardCoefficients(feedforwardCoefficients[0], feedforwardCoefficients[1], feedforwardCoefficients[2]);
        shooter.setVeloCoefficients(pidCoefficients[0], pidCoefficients[1], pidCoefficients[2]);
    }

    @Override
    public void loop() {
        shooter.set(1400);
        telemetry.addData("Velocity", shooter.getVelocity());
        telemetry.addData("Acceleration", shooter.getAcceleration());
    }
}
