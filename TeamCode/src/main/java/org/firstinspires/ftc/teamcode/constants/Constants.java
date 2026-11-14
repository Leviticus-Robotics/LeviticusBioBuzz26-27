package org.firstinspires.ftc.teamcode.constants;

import com.acmerobotics.dashboard.config.Config;
import com.pedropathing.algorithm.Foresight;
import com.pedropathing.algorithm.ForesightConfig;
import com.pedropathing.controllers.Controller;
import com.pedropathing.follower.Follower;
import com.pedropathing.math.Matrix;
import com.pedropathing.math.Pose;
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
import org.firstinspires.ftc.teamcode.fusion.MatrixOps;
import org.firstinspires.ftc.teamcode.localization.prediction.ControlInput;
import org.firstinspires.ftc.teamcode.localization.prediction.PoseVelocityProcessModel;
import org.firstinspires.ftc.teamcode.localization.pedro.FusedLocalizer;
import org.firstinspires.ftc.teamcode.localization.pedro.FusionMecanumDrivetrain;
import org.firstinspires.ftc.teamcode.localization.pedro.PredictiveLocalizer;
import org.firstinspires.ftc.teamcode.localization.apriltag.AprilTagFrameProcessor;

import dev.frozenmilk.dairy.mercurial.pedro.MercurialFollower;
import dev.frozenmilk.dairy.mercurial.pedro.MercurialLocalizer;

@Config
public class Constants {
    public static boolean useFusedLocalization = false;
    public static PoseVelocityProcessModel.Config fusionProcessModelConfig =
            PoseVelocityProcessModel.Config.defaults();
    public static org.ejml.simple.SimpleMatrix fusionInitialCovariance =
            MatrixOps.diagonal(4, 4, .25);
    // Pose variance accumulated per inch, inch, and radian of measured motion.
    public static org.ejml.simple.SimpleMatrix octoQuadVariancePerUnit =
            MatrixOps.diagonal(.0025, .0025, .0001);
    public static org.ejml.simple.SimpleMatrix aprilTagCovariance =
            MatrixOps.diagonal(2.25, 2.25, .04);
    public static org.ejml.simple.SimpleMatrix sharedCameraCovarianceFloor =
            MatrixOps.diagonal(.25, .25, .01);
    public static double aprilTagConsensusGateSquared = 16.27;
    public static double aprilTagDisagreementInflation = 1.0;
    public static Pose fieldConventionTransform = Pose.zero();
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
    public static double vx = 6;//2.3;
    public static double vy = 2.3; // 6
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
            HardwareMap hardwareMap, FusionMecanumDrivetrain drivetrain) {
        if (!useFusedLocalization) return createLocalizer(hardwareMap);
        return new MercurialLocalizer(createFusedLocalizer(hardwareMap, drivetrain));
    }

    /** Builds pose fusion and wires drivetrain inputs into its non-feedback predictor. */
    public static PredictiveLocalizer<ControlInput> createFusedLocalizer(
            HardwareMap hardwareMap, FusionMecanumDrivetrain drivetrain) {
        if (hardwareMap == null) throw new IllegalArgumentException("hardware map is null");
        if (drivetrain == null) throw new IllegalArgumentException("drivetrain is null");
        FusedLocalizer fused = new FusedLocalizer(
                new OctoQuadLocalizer(hardwareMap, localizerConfig),
                new FusedLocalizer.Config(
                        fusionInitialCovariance,
                        octoQuadVariancePerUnit,
                        fusionHistoryNanos));
        PredictiveLocalizer<ControlInput> predictive = new PredictiveLocalizer<>(
                fused,
                new PoseVelocityProcessModel(fusionProcessModelConfig),
                ControlInput.ZERO);
        drivetrain.addHandler(predictive);
        return predictive;
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
