package org.firstinspires.ftc.teamcode.localization.pedro;

import static org.junit.jupiter.api.Assertions.assertEquals;

import org.firstinspires.ftc.teamcode.localization.core.ControlInput;
import org.junit.jupiter.api.Test;

class FusionMecanumDrivetrainTest {
    @Test void recoversAppliedChassisCommandFromMecanumWheelPowers() {
        ControlInput input = FusionMecanumDrivetrain.fromWheelPowers(-.4, 1.0, .2, .4);
        assertEquals(.3, input.forwardPower, 1e-12);
        assertEquals(.3, input.strafePower, 1e-12);
        assertEquals(.4, input.turnPower, 1e-12);
    }
}
