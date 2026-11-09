package org.firstinspires.ftc.teamcode.localization.pedro;

import com.pedropathing.localization.Localizer;
import com.pedropathing.localization.MotionState;
import com.pedropathing.math.Pose;
import com.pedropathing.math.Velocity;
import java.util.ArrayList;
import java.util.Collections;
import java.util.LinkedHashMap;
import java.util.List;
import java.util.Map;
import org.ejml.simple.SimpleMatrix;
import org.firstinspires.ftc.teamcode.fusion.core.MatrixOps;
import org.firstinspires.ftc.teamcode.fusion.engine.FusionDiagnostics;
import org.firstinspires.ftc.teamcode.fusion.engine.FusionEngine;
import org.firstinspires.ftc.teamcode.fusion.engine.FactorApplicationResult;
import org.firstinspires.ftc.teamcode.fusion.factor.Factor;
import org.firstinspires.ftc.teamcode.NanoClock;
import org.firstinspires.ftc.teamcode.localization.core.ControlInput;
import org.firstinspires.ftc.teamcode.localization.core.Pose2;
import org.firstinspires.ftc.teamcode.localization.core.RobotState;
import org.firstinspires.ftc.teamcode.localization.sensor.AutomaticFusionSource;

/** Pedro localizer backed by a fusion engine and its automatically polled measurement sources. */
public final class FusedLocalizer implements Localizer {
    public static final long DEFAULT_HISTORY_NANOS = 2_000_000_000L;

    private final FusionEngine<ControlInput> engine;
    private final NanoClock clock;
    private final SimpleMatrix resetCovariance;
    private final List<AutomaticFusionSource> automaticSources;

    public FusedLocalizer(FusionEngine<ControlInput> engine, NanoClock clock,
                          SimpleMatrix resetCovariance) {
        this(engine, clock, resetCovariance, Collections.emptyList());
    }

    public FusedLocalizer(FusionEngine<ControlInput> engine, NanoClock clock,
                          SimpleMatrix resetCovariance,
                          List<? extends AutomaticFusionSource> automaticSources) {
        if (engine == null) throw new IllegalArgumentException("engine is null");
        if (clock == null) throw new IllegalArgumentException("clock is null");
        if (automaticSources == null) throw new IllegalArgumentException("automatic sources are null");
        this.engine = engine;
        this.clock = clock;
        this.resetCovariance = MatrixOps.requirePositiveDefinite(
                resetCovariance, RobotState.DIMENSION, "reset covariance");
        ArrayList<AutomaticFusionSource> sources = new ArrayList<>(automaticSources.size());
        for (AutomaticFusionSource source : automaticSources) {
            if (source == null) throw new IllegalArgumentException("automatic source is null");
            sources.add(source);
        }
        this.automaticSources = Collections.unmodifiableList(sources);
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

    @Override public void update() {
        for (AutomaticFusionSource source : automaticSources) {
            Factor factor = source.poll();
            if (factor != null) engine.applyFactor(factor);
        }
        engine.predictTo(clock.now());
    }

    /** Advances the process model without polling automatic measurement sources. */
    public void predictToNow() { engine.predictTo(clock.now()); }

    /** Applies an externally produced, timestamped factor such as an AprilTag frame. */
    public FactorApplicationResult applyFactor(Factor factor) {
        return engine.applyFactor(factor);
    }

    /** Records the chassis command that was actually applied by the drivetrain. */
    public void setDriveInput(double forward, double strafe, double turn) {
        engine.setInput(clock.now(), new ControlInput(forward, strafe, turn));
    }

    @Override public void reset() {
        reset(RobotState.create(Pose2.zero(), 0, 0, 0, resetCovariance));
    }

    private void reset(RobotState state) {
        for (AutomaticFusionSource source : automaticSources) source.reset();
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

    public FusionEngine<ControlInput> engine() { return engine; }
}
