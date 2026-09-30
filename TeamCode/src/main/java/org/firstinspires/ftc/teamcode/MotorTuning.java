package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.pedropathing.utils.Timer;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.hardware.Motor;
import org.firstinspires.ftc.teamcode.hardware.MotorEx;
import org.firstinspires.ftc.teamcode.util.ConfigNames;

import java.util.concurrent.TimeUnit;

@Config
@TeleOp
public class MotorTuning extends OpMode {
    public static double targetPower = 0.1;
    public static double targetVelocity = 1000;
    public static boolean useFixedPower = true;
    public static double[] pidCoefficients = new double[] {1.0, 0.0, 0.0};
    public static double[] feedforwardCoefficients = new double[]{0, 0, 0};
    Telemetry dashboard;
    MotorEx motor;
    Timer timer;
    double previousTime = 0;

    @Override
    public void init() {
        timer = new Timer();
        dashboard = FtcDashboard.getInstance().getTelemetry();
        motor = new MotorEx(hardwareMap, ConfigNames.testMotor);
        if(!useFixedPower) {
            motor.setVeloCoefficients(pidCoefficients[0], pidCoefficients[1], pidCoefficients[2]);
            motor.setFeedforwardCoefficients(feedforwardCoefficients[0], feedforwardCoefficients[1], feedforwardCoefficients[2]);
        }
        motor.setRunMode(useFixedPower ? Motor.RunMode.RawPower : Motor.RunMode.VelocityControl);
    }

    @Override
    public void loop() {

        motor.set(useFixedPower ? targetPower : targetVelocity);


        telemetry.addData("Update Rate", 1000.0 / (timer.get(TimeUnit.MILLISECONDS) - previousTime));
        previousTime = timer.get(TimeUnit.MILLISECONDS);
        telemetry.addLine("--------------------------------------------");
        telemetry.addData("Motor Power", motor.motor.getPower());
        telemetry.addData("Motor Velocity", motor.getVelocity());
        dashboard.addData("Motor Power", motor.motor.getPower());
        dashboard.addData("Motor Velocity", motor.getVelocity());
        //use this class to get y-int and slopes for motor power vs motor velocity
        //slope is kV, y-intercept is kS
        //can also use this class to tune P and D gains for flywheel
        dashboard.update();
        telemetry.update();
    }
}
