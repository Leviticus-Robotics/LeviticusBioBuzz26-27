package org.firstinspires.ftc.teamcode.constants;

import com.acmerobotics.dashboard.config.Config;
import com.pedropathing.algorithm.Foresight;
import com.pedropathing.algorithm.ForesightConfig;
import com.pedropathing.controllers.Controller;
import com.pedropathing.follower.Follower;
import com.pedropathing.math.Matrix;
import com.pedropathing.math.Vector2D;
import com.pedropathing.revhub.drivetrains.Mecanum;
import com.pedropathing.revhub.drivetrains.MecanumConfig;
import com.pedropathing.revhub.localizers.OctoQuadConfig;
import com.pedropathing.revhub.localizers.OctoQuadLocalizer;
import com.qualcomm.hardware.digitalchickenlabs.OctoQuad;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagSingleDetection;
import org.firstinspires.ftc.teamcode.NanoClock;
import org.firstinspires.ftc.teamcode.fusion.engine.FusionEngine;
import org.firstinspires.ftc.teamcode.fusion.core.MatrixOps;
import org.firstinspires.ftc.teamcode.localization.core.Pose2;
import org.firstinspires.ftc.teamcode.localization.core.RobotState;
import org.firstinspires.ftc.teamcode.localization.engine.PoseVelocityProcessModel;
import org.firstinspires.ftc.teamcode.localization.pedro.FusedLocalizer;
import org.firstinspires.ftc.teamcode.localization.pedro.FusionMecanumDrivetrain;
import org.firstinspires.ftc.teamcode.localization.sensor.AutomaticFusionSource;
import org.firstinspires.ftc.teamcode.localization.sensor.CommandProcessSource;
import org.firstinspires.ftc.teamcode.localization.sensor.apriltag.AprilTagFrameProcessor;
import org.firstinspires.ftc.teamcode.localization.sensor.octoquad.OctoQuadMotionAdapter;
import org.firstinspires.ftc.teamcode.localization.sensor.octoquad.OctoQuadFusionSource;
import org.firstinspires.ftc.teamcode.subsystems.Drivetrain;

import java.util.Arrays;
import java.util.List;

import dev.frozenmilk.dairy.mercurial.pedro.MercurialFollower;
import dev.frozenmilk.dairy.mercurial.pedro.MercurialLocalizer;

@Config
public class Constants {
    public static boolean useFusedLocalization = false;
    public static PoseVelocityProcessModel.Config fusionProcessModelConfig =
            PoseVelocityProcessModel.Config.defaults();
    public static org.ejml.simple.SimpleMatrix fusionInitialCovariance =
            MatrixOps.diagonal(4, 4, .25, 25, 25, 4);
    // Per-tick variance for x, y, heading, vx, vy, omega. Corresponding standard
    // deviations are .05 in, .05 in, .01 rad, .5 in/s, .5 in/s, and .1 rad/s.
    public static org.ejml.simple.SimpleMatrix octoQuadMotionCovariance =
            MatrixOps.diagonal(.0025, .0025, .0001, .25, .25, .01);
    public static org.ejml.simple.SimpleMatrix aprilTagCovariance =
            MatrixOps.diagonal(2.25, 2.25, .04);
    public static org.ejml.simple.SimpleMatrix sharedCameraCovarianceFloor =
            MatrixOps.diagonal(.25, .25, .01);
    public static double aprilTagConsensusGateSquared = 16.27;
    public static double aprilTagDisagreementInflation = 1.0;
    public static Pose2 fieldConventionTransform = Pose2.zero();
    public static long fusionHistoryNanos = FusedLocalizer.DEFAULT_HISTORY_NANOS;
    public static int[] stationaryLocalizationTagIds = new int[0];
    public static MecanumConfig driveConfig = new MecanumConfig(
        c -> {
            c.frontLeftName.set(ConfigNames.FL);
            c.backLeftName.set(ConfigNames.BL);
            c.frontRightName.set(ConfigNames.FR);
            c.backRightName.set(ConfigNames.BR);
            c.frontLeftDirection.set(DcMotorSimple.Direction.FORWARD);
            c.frontRightDirection.set(DcMotorSimple.Direction.REVERSE);
            c.backLeftDirection.set(DcMotorSimple.Direction.FORWARD);
            c.backRightDirection.set(DcMotorSimple.Direction.REVERSE);
        }
    );
    public static
    double vx = 6;//2.3;
    public static
    double vy = 2.3; // 6
    public static OctoQuadConfig localizerConfig = new OctoQuadConfig(c -> {
        c.name.set(ConfigNames.octoquad);
        c.xPodPort.set(0);
        c.yPodPort.set(1);
        c.ticksPerUnit.set(505.316944406);
        c.xPodOffset.set(-vx);//-2.3
        c.yPodOffset.set(-vy);//-5.787401574803149t#
        c.xPodDirection.set(OctoQuad.EncoderDirection.FORWARD);
        c.yPodDirection.set(OctoQuad.EncoderDirection.REVERSE);
        c.globalDistanceUnit.set(DistanceUnit.INCH);
        c.offsetUnits.set(DistanceUnit.INCH);
        c.i2cRecoveryMode.set(OctoQuad.I2cRecoveryMode.MODE_1_PERIPH_RST_ON_FRAME_ERR);
        c.localizerVelocityIntervalMS.set(35);
        c.headingScalar.set(1.02750014857118422);//1.0284592570209024
    });


    public static ForesightConfig foresightConfig = new ForesightConfig(
            c -> {
                Controller primaryTranslationalForward = Controller.proportional(0.13799587025007254);
                Controller secondaryTranslationalForward = Controller.proportional(0.050985786739838425);
                Controller primaryTranslationalLateral = Controller.proportional(0.1893968724290156);
                Controller secondaryTranslationalLateral = Controller.proportional(0.06997708358488429);

                c.forwardTranslational.set(Controller.piecewise(secondaryTranslationalForward).put(2.5, primaryTranslationalForward));
                c.strafeTranslational.set(Controller.piecewise(secondaryTranslationalLateral).put(2.5, primaryTranslationalLateral));

                c.coast.set(Controller.proportionalFeedforward(0.011865483669611175));
                c.brake.set(Controller.proportionalFeedforward(0.010085661119169498));

                c.headingFeedback.set(Controller.proportional(2.2965074919139523));
                c.headingBrakeCoefficients.set(Vector2D.cartesian(0.034928751892690506, 0.005198706331665716));

                c.linearBrakeCoefficients.set(Matrix.diag(0.041696512755505434, 0.04882470355217723));
                c.quadraticBrakeCoefficients.set(Matrix.diag(0.001243139534425205, 0.0012439390035693934));

                c.maxAchievableForwardVelocity.set(85.92625305166288);
                c.maxAchievableStrafeVelocity.set(72.48126596053376);
                c.naturalForwardDeceleration.set(47.84449917569892);
                c.naturalStrafeDeceleration.set(64.59534344120797);
            }
    );


    public static Follower create(HardwareMap h) {
        return new Follower(
                new OctoQuadLocalizer(h, localizerConfig),
                new Mecanum(h, driveConfig),
                new Foresight(foresightConfig)
        );
    }

    public static MercurialFollower createFollower(HardwareMap h) {
        return createFollower(new FusionMecanumDrivetrain(h, driveConfig));
    }

    public static MercurialFollower createFollower(
            com.pedropathing.drivetrain.Drivetrain drivetrain) {
        return new MercurialFollower(drivetrain, new Foresight(foresightConfig));
    }

    public static MercurialLocalizer createLocalizer(HardwareMap h) {
        return new MercurialLocalizer(
                new OctoQuadLocalizer(h, localizerConfig)
        );
    }

    /** Constructs and wires the configured localization implementation. */
    public static MercurialLocalizer createLocalizer(
            HardwareMap hardwareMap, Drivetrain drivetrain) {
        if (!useFusedLocalization) return createLocalizer(hardwareMap);

        return createFusedLocalizer(hardwareMap, drivetrain, createProcessSource());
    }

    public static CommandProcessSource createProcessSource() {
        return new CommandProcessSource(new PoseVelocityProcessModel(fusionProcessModelConfig));
    }

    /** Constructs fused localization while exposing its process source for diagnostic controls. */
    public static MercurialLocalizer createFusedLocalizer(
            HardwareMap hardwareMap, Drivetrain drivetrain,
            CommandProcessSource processSource) {
        if (hardwareMap == null) throw new IllegalArgumentException("hardware map is null");
        if (drivetrain == null) throw new IllegalArgumentException("drivetrain is null");
        if (processSource == null) throw new IllegalArgumentException("process source is null");
        OctoQuadFusionSource octoQuadSource = new OctoQuadFusionSource(
                new OctoQuadLocalizer(hardwareMap, localizerConfig),
                new OctoQuadMotionAdapter(
                        octoQuadMotionCovariance));
        drivetrain.addHandler(processSource);
        return new MercurialLocalizer(createLocalizer(
                Arrays.asList(processSource, octoQuadSource)));
    }

    /** Builds a fused localizer around an explicit source set, primarily for diagnostics. */
    public static FusedLocalizer createLocalizer(
            List<? extends AutomaticFusionSource> sources) {
        NanoClock clock = NanoClock.SYSTEM;
        long now = clock.now();
        RobotState initial = RobotState.create(Pose2.zero(), 0, 0, 0,
                fusionInitialCovariance);
        FusionEngine engine = new FusionEngine(
                initial.gaussian(), now, RobotState::normalize,
                fusionHistoryNanos);
        return new FusedLocalizer(engine, clock, fusionInitialCovariance, sources);
    }

    public static AprilTagFrameProcessor createAprilTagProcessor() {
        return new AprilTagFrameProcessor(
                Constants::isStationaryLocalizationTag,
                aprilTagCovariance,
                sharedCameraCovarianceFloor,
                aprilTagConsensusGateSquared,
                aprilTagDisagreementInflation,
                fieldConventionTransform);
    }

    /** Only explicitly configured stationary custom/test tags may anchor field localization. */
    public static boolean isStationaryLocalizationTag(AprilTagDetection detection) {
        if (!(detection instanceof AprilTagSingleDetection)) return false;
        AprilTagSingleDetection single = (AprilTagSingleDetection) detection;
        if (single.metadata == null || single.robotPose == null) return false;
        for (int id : stationaryLocalizationTagIds) if (id == single.id) return true;
        return false;
    }
}
