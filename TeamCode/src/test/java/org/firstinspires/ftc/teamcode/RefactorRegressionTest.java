package org.firstinspires.ftc.teamcode;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertNotSame;
import static org.junit.jupiter.api.Assertions.assertThrows;
import static org.junit.jupiter.api.Assertions.assertTrue;

import com.pedropathing.math.Pose;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.sensor.TransferBeamBreak;
import org.firstinspires.ftc.teamcode.subsystems.Shooter;
import org.firstinspires.ftc.teamcode.util.Angle;
import org.firstinspires.ftc.teamcode.util.Range;
import org.junit.jupiter.api.Test;

class RefactorRegressionTest {
    @Test void changingAngleUnitsDoesNotMutateTheOriginalValue() {
        Angle radians = Angle.fromRadians(Math.PI);
        Angle degrees = radians.changeUnit(AngleUnit.DEGREES);

        assertNotSame(radians, degrees);
        assertEquals(AngleUnit.RADIANS, radians.getUnit());
        assertEquals(Math.PI, radians.getValue(), 1e-12);
        assertEquals(AngleUnit.DEGREES, degrees.getUnit());
        assertEquals(180, degrees.getValue(), 1e-12);
    }

    @Test void rangesRejectZeroWidthAndMapValues() {
        assertThrows(IllegalArgumentException.class, () -> new Range(2, 2));
        assertEquals(15, new Range(0, 10).toRange(5, new Range(10, 20)), 1e-12);
    }

    @Test void beamBreakEventsExposeImmutableValues() {
        TransferBeamBreak.BeamBreakEvent event =
                new TransferBeamBreak.BeamBreakEvent(2, true);
        assertEquals(2, event.sensorNumber());
        assertTrue(event.interrupted());
        assertThrows(IllegalArgumentException.class,
                () -> new TransferBeamBreak.BeamBreakEvent(3, false));
    }

    @Test void shooterHeadingUsesPositionBearingRatherThanPoseHeadingDifference() {
        Shooter.AimCalculator calculator = new Shooter.AimCalculator();
        Pose robot = new Pose(0, 0, 1.2);
        Pose target = new Pose(0, 10, -2.4);

        assertEquals(Math.PI / 2, calculator.getTargetHeading(robot, target), 1e-12);
    }

    @Test void shooterDistanceLookupClampsToVelocityTableDomain() {
        Shooter.AimCalculator calculator = new Shooter.AimCalculator();
        double belowRange = calculator.targetVelocityForDistance(-1000);
        double aboveRange = calculator.targetVelocityForDistance(1000);

        assertTrue(belowRange >= Shooter.AimCalculator.velocities[0]);
        assertTrue(aboveRange <= Shooter.AimCalculator.velocities[
                Shooter.AimCalculator.velocities.length - 1]);
    }
}
