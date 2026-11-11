package org.firstinspires.ftc.teamcode.localization.pedro;

import com.pedropathing.drivetrain.DrivePowers;
import com.pedropathing.drivetrain.Drivetrain;
import com.pedropathing.revhub.drivetrains.Mecanum;
import com.pedropathing.revhub.drivetrains.MecanumConfig;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import java.util.Map;
import org.firstinspires.ftc.teamcode.localization.LocalizationRuntime;
import org.firstinspires.ftc.teamcode.localization.core.ControlInput;

/** Mecanum drivetrain that records the motor powers Pedro actually applied. */
public final class FusionMecanumDrivetrain implements Drivetrain {
    private final LocalizationRuntime runtime;
    private final Mecanum drivetrain;
    private final DcMotorEx frontLeftMotor;
    private final DcMotorEx frontRightMotor;
    private final DcMotorEx backLeftMotor;
    private final DcMotorEx backRightMotor;
    private final boolean pollSensorBeforeDrive;

    public FusionMecanumDrivetrain(LocalizationRuntime runtime,
                                   HardwareMap hardwareMap, MecanumConfig config) {
        this(runtime, hardwareMap, config, false);
    }

    public FusionMecanumDrivetrain(LocalizationRuntime runtime,
                                   HardwareMap hardwareMap, MecanumConfig config,
                                   boolean pollSensorBeforeDrive) {
        this.runtime = runtime;
        this.drivetrain = new Mecanum(hardwareMap, config);
        this.frontLeftMotor = hardwareMap.get(DcMotorEx.class, config.frontLeftName.get());
        this.frontRightMotor = hardwareMap.get(DcMotorEx.class, config.frontRightName.get());
        this.backLeftMotor = hardwareMap.get(DcMotorEx.class, config.backLeftName.get());
        this.backRightMotor = hardwareMap.get(DcMotorEx.class, config.backRightName.get());
        this.pollSensorBeforeDrive = pollSensorBeforeDrive;
    }

    @Override public void drive(DrivePowers powers, boolean manual) {
        if (pollSensorBeforeDrive) runtime.updateOctoQuad();
        drivetrain.drive(powers, manual);
        recordAppliedMotorPowers();
    }

    @Override public double maxScaling(DrivePowers requested, DrivePowers current) {
        return drivetrain.maxScaling(requested, current);
    }

    @Override public void stop() {
        if (pollSensorBeforeDrive) runtime.updateOctoQuad();
        drivetrain.stop();
        recordAppliedMotorPowers();
    }

    @Override public void stop(boolean brake) {
        if (pollSensorBeforeDrive) runtime.updateOctoQuad();
        drivetrain.stop(brake);
        recordAppliedMotorPowers();
    }

    @Override public Map<String, Object> debug() { return drivetrain.debug(); }

    @Override public double interpolateVelocity(double start, double end, double t) {
        return drivetrain.interpolateVelocity(start, end, t);
    }

    private void recordAppliedMotorPowers() {
        ControlInput input = fromWheelPowers(
                frontLeftMotor.getPower(), frontRightMotor.getPower(),
                backLeftMotor.getPower(), backRightMotor.getPower());
        runtime.setDriveInput(input.forwardPower, input.strafePower, input.turnPower);
    }

    static ControlInput fromWheelPowers(double frontLeft, double frontRight,
                                        double backLeft, double backRight) {
        return new ControlInput(
                (frontLeft + frontRight + backLeft + backRight) / 4.0,
                (-frontLeft + frontRight + backLeft - backRight) / 4.0,
                (-frontLeft + frontRight - backLeft + backRight) / 4.0);
    }
}
