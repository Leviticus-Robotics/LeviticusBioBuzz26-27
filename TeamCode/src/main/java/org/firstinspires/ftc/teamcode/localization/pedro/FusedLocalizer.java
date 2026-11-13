package org.firstinspires.ftc.teamcode.localization.pedro;

import com.pedropathing.localization.Localizer;
import com.pedropathing.localization.MotionState;
import com.pedropathing.math.Pose;
import com.pedropathing.math.Velocity;
import java.util.LinkedHashMap;
import java.util.Map;
import org.ejml.simple.SimpleMatrix;
import org.firstinspires.ftc.teamcode.fusion.core.MatrixOps;
import org.firstinspires.ftc.teamcode.fusion.engine.FusionDiagnostics;
import org.firstinspires.ftc.teamcode.fusion.engine.FusionEngine;
import org.firstinspires.ftc.teamcode.helper.NanoClock;
import org.firstinspires.ftc.teamcode.localization.core.Pose2;
import org.firstinspires.ftc.teamcode.localization.core.RobotState;

/** Pedro output boundary. Sensor polling deliberately lives in LocalizationRuntime. */
public final class FusedLocalizer implements Localizer {
    private final FusionEngine<?> engine;
    private final NanoClock clock;
    private final SimpleMatrix resetCovariance;
    private final Runnable resetRelativeSources;

    public FusedLocalizer(FusionEngine<?> engine, NanoClock clock, SimpleMatrix resetCovariance) {
        this(engine, clock, resetCovariance, () -> { });
    }

    public FusedLocalizer(FusionEngine<?> engine, NanoClock clock, SimpleMatrix resetCovariance,
                          Runnable resetRelativeSources) {
        this.engine = engine;
        this.clock = clock;
        this.resetCovariance = MatrixOps.requirePositiveDefinite(
                resetCovariance, RobotState.DIMENSION, "reset covariance");
        if (resetRelativeSources == null) throw new IllegalArgumentException("reset callback is null");
        this.resetRelativeSources = resetRelativeSources;
    }

    @Override public void setPose(Pose pose) {
        RobotState previous = new RobotState(engine.state());
        reset(RobotState.create(new Pose2(pose.x(), pose.y(), pose.heading()),
                previous.vx(), previous.vy(), previous.omega(), resetCovariance));
    }

    @Override public MotionState state() {
        RobotState state = new RobotState(engine.state());
        Pose2 pose = state.pose();
        return MotionState.ofVelocity(new Pose(pose.x(), pose.y(), pose.heading()),
                new Velocity(state.vx(), state.vy(), state.omega()));
    }

    @Override public void update() { engine.predictTo(clock.now()); }

    @Override public void reset() {
        reset(RobotState.create(Pose2.zero(), 0, 0, 0, resetCovariance));
    }

    private void reset(RobotState state) {
        resetRelativeSources.run();
        engine.resetState(state.gaussian(), clock.now());
    }

    @Override public Map<String, Object> debug() {
        FusionDiagnostics<?> diagnostics = engine.diagnostics();
        RobotState state = new RobotState(engine.state());
        Map<String, Object> result = new LinkedHashMap<>();
        result.put("pose", state.pose());
        result.put("velocity", new double[] {state.vx(), state.vy(), state.omega()});
        result.put("covarianceDiagonal", covarianceDiagonal(state.covariance()));
        result.put("activeInput", diagnostics.activeInput);
        result.put("historyEvents", diagnostics.historyEventCount);
        result.put("recomputationNanos", diagnostics.lastRecomputationDurationNanos);
        result.put("acceptedFactors", diagnostics.acceptedFactorCount);
        result.put("rejectedFactors", diagnostics.rejectedFactorCount);
        result.put("latestSourceTimestamps", diagnostics.latestSourceTimestamps);
        result.put("lastDecision", diagnostics.lastDecision);
        return result;
    }

    private static double[] covarianceDiagonal(SimpleMatrix covariance) {
        double[] result = new double[covariance.numRows()];
        for (int i = 0; i < result.length; i++) result[i] = covariance.get(i, i);
        return result;
    }

    public FusionEngine<?> engine() { return engine; }
}
