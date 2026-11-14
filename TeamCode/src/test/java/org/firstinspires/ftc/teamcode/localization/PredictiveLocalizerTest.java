package org.firstinspires.ftc.teamcode.localization;

import static org.junit.jupiter.api.Assertions.*;

import com.pedropathing.localization.Localizer;
import com.pedropathing.localization.MotionState;
import com.pedropathing.math.Pose;
import com.pedropathing.math.Velocity;
import java.util.Collections;
import java.util.Map;
import java.util.concurrent.atomic.AtomicLong;
import org.firstinspires.ftc.teamcode.fusion.MatrixOps;
import org.firstinspires.ftc.teamcode.localization.prediction.ControlInput;
import org.firstinspires.ftc.teamcode.localization.prediction.PoseVelocityProcessModel;
import org.firstinspires.ftc.teamcode.localization.pedro.FusedLocalizer;
import org.firstinspires.ftc.teamcode.localization.pedro.PredictiveLocalizer;
import org.junit.jupiter.api.Test;

class PredictiveLocalizerTest {
    @Test void predictionUsesLatestInputWithoutMutatingFusedState() {
        AtomicLong clock = new AtomicLong();
        FakeLocalizer raw = new FakeLocalizer();
        FusedLocalizer fused = new FusedLocalizer(raw, new FusedLocalizer.Config(
                MatrixOps.diagonal(1, 1, 1), MatrixOps.diagonal(.01, .01, .01),
                1_000_000_000L), clock::get);
        PredictiveLocalizer<ControlInput> predictive = new PredictiveLocalizer<>(
                fused, new PoseVelocityProcessModel(
                        new PoseVelocityProcessModel.Config(10, 20, 20, 4)),
                ControlInput.ZERO);
        predictive.handleEvent(new ControlInput(1, 0, 0));
        MotionState future = predictive.predict(.1);
        assertTrue(future.pose().x() > 0);
        assertTrue(future.velocity().vx > 0);
        assertEquals(0, predictive.state().pose().x(), 1e-12);
    }

    private static final class FakeLocalizer implements Localizer {
        MotionState state = MotionState.zero();
        @Override public MotionState state() { return state; }
        @Override public void update() { }
        @Override public void setPose(Pose pose) {
            state = MotionState.ofVelocity(pose, Velocity.zero());
        }
        @Override public void reset() { state = MotionState.zero(); }
        @Override public Map<String, Object> debug() { return Collections.emptyMap(); }
    }
}
