package org.firstinspires.ftc.teamcode.localization;

import static org.junit.jupiter.api.Assertions.*;

import com.pedropathing.localization.Localizer;
import com.pedropathing.localization.MotionState;
import com.pedropathing.math.Pose;
import com.pedropathing.math.Velocity;
import java.util.Collections;
import java.util.Map;
import java.util.concurrent.atomic.AtomicLong;
import org.firstinspires.ftc.teamcode.fusion.Gaussian;
import org.firstinspires.ftc.teamcode.fusion.MatrixOps;
import org.firstinspires.ftc.teamcode.localization.pedro.FusedLocalizer;
import org.junit.jupiter.api.Test;

class FusedLocalizerTest {
    private static FusedLocalizer create(FakeLocalizer raw, AtomicLong clock) {
        return new FusedLocalizer(raw, new FusedLocalizer.Config(
                MatrixOps.diagonal(1, 1, .1),
                MatrixOps.diagonal(.001, .001, .0001),
                10_000_000_000L), clock::get);
    }

    @Test void odometryMovesTheAuthoritativeEstimateDirectly() {
        AtomicLong clock = new AtomicLong();
        FakeLocalizer raw = new FakeLocalizer();
        FusedLocalizer fused = create(raw, clock);
        raw.state = MotionState.ofVelocity(new Pose(10, 2, .2), new Velocity(10, 2, .2));
        clock.set(1_000_000_000L);
        fused.update();
        assertEquals(10, fused.state().pose().x(), 1e-9);
        assertEquals(2, fused.state().pose().y(), 1e-9);
        assertEquals(.2, fused.state().pose().heading(), 1e-9);
        assertEquals(10, fused.state().velocity().vx, 1e-9);
    }

    @Test void delayedAbsoluteMeasurementIsInsertedAndMotionReplayed() {
        AtomicLong clock = new AtomicLong();
        FakeLocalizer raw = new FakeLocalizer();
        FusedLocalizer fused = create(raw, clock);
        raw.state = MotionState.ofVelocity(new Pose(10, 0, 0), new Velocity(10, 0, 0));
        clock.set(1_000_000_000L);
        fused.update();
        assertTrue(fused.addAbsoluteMeasurement("camera", 500_000_000L,
                new Gaussian(new double[] {4, 0, 0},
                        MatrixOps.diagonal(1e-6, 1e-6, 1e-6))));
        assertEquals(9, fused.state().pose().x(), .01);
    }

    @Test void correctionChangesGlobalFrameWithoutFightingFutureOdometry() {
        AtomicLong clock = new AtomicLong();
        FakeLocalizer raw = new FakeLocalizer();
        FusedLocalizer fused = create(raw, clock);
        raw.state = MotionState.ofVelocity(new Pose(10, 0, 0), Velocity.zero());
        clock.set(1_000_000_000L);
        fused.update();
        assertTrue(fused.addAbsoluteMeasurement("camera", 1_000_000_000L,
                new Gaussian(new double[] {20, 0, 0},
                        MatrixOps.diagonal(1e-8, 1e-8, 1e-8))));
        raw.state = MotionState.ofVelocity(new Pose(11, 0, 0), Velocity.zero());
        clock.set(2_000_000_000L);
        fused.update();
        assertEquals(21, fused.state().pose().x(), .01);
    }

    @Test void cameraFrameNewerThanLatestTickWaitsForTheNextUpdate() {
        AtomicLong clock = new AtomicLong();
        FakeLocalizer raw = new FakeLocalizer();
        FusedLocalizer fused = create(raw, clock);
        assertTrue(fused.addAbsoluteMeasurement("camera", 500_000_000L,
                new Gaussian(new double[] {4, 0, 0},
                        MatrixOps.diagonal(1e-6, 1e-6, 1e-6))));
        raw.state = MotionState.ofVelocity(new Pose(10, 0, 0), new Velocity(10, 0, 0));
        clock.set(1_000_000_000L);
        fused.update();
        assertEquals(9, fused.state().pose().x(), .01);
    }

    @Test void delayedReplayMatchesTheSameMeasurementAppliedInOrder() {
        AtomicLong firstClock = new AtomicLong();
        AtomicLong secondClock = new AtomicLong();
        FakeLocalizer firstRaw = new FakeLocalizer();
        FakeLocalizer secondRaw = new FakeLocalizer();
        FusedLocalizer inOrder = create(firstRaw, firstClock);
        FusedLocalizer delayed = create(secondRaw, secondClock);
        Gaussian camera = new Gaussian(new double[] {8, 1, .1},
                MatrixOps.diagonal(.01, .01, .001));

        firstRaw.state = MotionState.ofVelocity(new Pose(10, 0, 0), Velocity.zero());
        secondRaw.state = firstRaw.state;
        firstClock.set(1_000_000_000L);
        secondClock.set(1_000_000_000L);
        inOrder.update();
        delayed.update();
        inOrder.addAbsoluteMeasurement("camera", 1_000_000_000L, camera);

        firstRaw.state = MotionState.ofVelocity(new Pose(20, 0, 0), Velocity.zero());
        secondRaw.state = firstRaw.state;
        firstClock.set(2_000_000_000L);
        secondClock.set(2_000_000_000L);
        inOrder.update();
        delayed.update();
        delayed.addAbsoluteMeasurement("camera", 1_000_000_000L, camera);

        assertEquals(inOrder.state().pose().x(), delayed.state().pose().x(), 1e-10);
        assertEquals(inOrder.state().pose().y(), delayed.state().pose().y(), 1e-10);
        assertEquals(inOrder.state().pose().heading(), delayed.state().pose().heading(), 1e-10);
        assertTrue(inOrder.poseEstimate().covariance().isIdentical(
                delayed.poseEstimate().covariance(), 1e-10));
    }

    private static final class FakeLocalizer implements Localizer {
        MotionState state = MotionState.zero();
        @Override public MotionState state() { return state; }
        @Override public void update() { }
        @Override public void setPose(Pose pose) {
            state = MotionState.ofVelocity(pose, state.velocity());
        }
        @Override public void reset() { state = MotionState.zero(); }
        @Override public Map<String, Object> debug() { return Collections.emptyMap(); }
    }
}
