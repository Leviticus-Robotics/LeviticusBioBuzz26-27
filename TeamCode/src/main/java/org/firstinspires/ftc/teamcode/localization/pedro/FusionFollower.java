package org.firstinspires.ftc.teamcode.localization.pedro;

import com.pedropathing.algorithm.Algorithm;
import com.pedropathing.follower.Follower;
import com.pedropathing.revhub.drivetrains.MecanumConfig;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.teamcode.localization.LocalizationRuntime;

/** Pedro integration that pumps sensor events outside the Localizer adapter. */
public final class FusionFollower extends Follower {
    private final LocalizationRuntime runtime;

    public FusionFollower(LocalizationRuntime runtime, HardwareMap hardwareMap,
                          MecanumConfig drivetrainConfig, Algorithm algorithm) {
        super(runtime.localizer(), new FusionMecanumDrivetrain(
                runtime, hardwareMap, drivetrainConfig), algorithm);
        this.runtime = runtime;
    }

    @Override public void update() {
        runtime.updateOctoQuad();
        super.update();
    }

    @Override public void update(double elapsedSeconds) {
        runtime.updateOctoQuad();
        super.update(elapsedSeconds);
    }

    public LocalizationRuntime localizationRuntime() { return runtime; }
}
