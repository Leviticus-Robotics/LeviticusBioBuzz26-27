package org.firstinspires.ftc.teamcode;

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
import com.pedropathing.revhub.localizers.PinpointLocalizer;
import com.qualcomm.hardware.digitalchickenlabs.OctoQuad;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;


public class Constants {
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

    public static OctoQuadConfig localizerConfig = new OctoQuadConfig(c -> {
        c.name.set(ConfigNames.OCTOQUAD);
        c.xPodPort.set(0);
        c.yPodPort.set(1);
        c.ticksPerUnit.set(505.316944406);
        c.xPodOffset.set(-2.204724409448819);
        c.yPodOffset.set(-5.787401574803149);
        c.xPodDirection.set(OctoQuad.EncoderDirection.FORWARD);
        c.yPodDirection.set(OctoQuad.EncoderDirection.REVERSE);
        c.globalDistanceUnit.set(DistanceUnit.INCH);
        c.offsetUnits.set(DistanceUnit.INCH);
        c.i2cRecoveryMode.set(OctoQuad.I2cRecoveryMode.MODE_1_PERIPH_RST_ON_FRAME_ERR);
        c.headingScalar.set(1.0284592570209024);
    });
    //new
//        c.ticksPerUnit.set(505.316944406);
//        c.xPodOffset.set(-2.204724409448819);
//        c.yPodOffset.set(-5.787401574803149);
//        c.headingScalar.set(1.0284592570209024);

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
}