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
import org.firstinspires.ftc.teamcode.fusion.engine.FactorApplicationResult;
import org.firstinspires.ftc.teamcode.fusion.engine.FusionDiagnostics;
import org.firstinspires.ftc.teamcode.fusion.engine.FusionEngine;
import org.firstinspires.ftc.teamcode.fusion.factor.Factor;
import org.firstinspires.ftc.teamcode.NanoClock;
import org.firstinspires.ftc.teamcode.localization.core.Pose2;
import org.firstinspires.ftc.teamcode.localization.core.RobotState;
import org.firstinspires.ftc.teamcode.localization.sensor.AutomaticFusionSource;

/** Pedro localizer backed by a fusion engine and its automatically polled measurement sources. */
public final class FusedLocalizer implements Localizer {
    public static final long DEFAULT_HISTORY_NANOS = 2_000_000_000L;

    private final FusionEngine engine;
    private final NanoClock clock;
    private final SimpleMatrix resetCovariance;
    private final List<AutomaticFusionSource> automaticSources;

    public FusedLocalizer(FusionEngine engine, NanoClock clock, Config config) {
        this(engine, clock, config, Collections.emptyList());
    }

    public FusedLocalizer(FusionEngine engine, NanoClock clock, Config config,
                          List<? extends AutomaticFusionSource> automaticSources) {
        if (engine == null) throw new IllegalArgumentException("engine is null");
        if (clock == null) throw new IllegalArgumentException("clock is null");
        if (config == null) throw new IllegalArgumentException("localizer config is null");
        if (automaticSources == null) throw new IllegalArgumentException("automatic sources are null");
        this.engine = engine;
        this.clock = clock;
        resetCovariance = config.initialCovariance();
        ArrayList<AutomaticFusionSource> sources = new ArrayList<>(automaticSources.size());
        for (AutomaticFusionSource source : automaticSources) {
            if (source == null) throw new IllegalArgumentException("automatic source is null");
            sources.add(source);
        }
        this.automaticSources = Collections.unmodifiableList(sources);
        for (AutomaticFusionSource source : this.automaticSources) {
            source.reset(engine.timestampNanos());
        }
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
        long tickTimestamp = clock.now();
        long previousTimestamp = engine.timestampNanos();
        List<Factor> tickFactors = new ArrayList<>();
        try {
            for (AutomaticFusionSource source : automaticSources) {
                Factor factor = source.poll(previousTimestamp, tickTimestamp);
                if (factor != null) tickFactors.add(factor);
            }
        } catch (RuntimeException exception) {
            for (AutomaticFusionSource source : automaticSources) source.rollback();
            throw exception;
        }
        FactorApplicationResult result = engine.commitTick(tickTimestamp, tickFactors);
        if (result.isAccepted()) {
            for (AutomaticFusionSource source : automaticSources) source.commit();
        } else {
            for (AutomaticFusionSource source : automaticSources) source.rollback();
            throw new IllegalStateException(result.reason);
        }
    }

    /** Queues an externally produced factor for the next atomic tick commit. */
    public FactorApplicationResult submitFactor(Factor factor) {
        return engine.submitFactor(factor);
    }

    @Override public void reset() {
        reset(RobotState.create(Pose2.zero(), 0, 0, 0, resetCovariance));
    }

    private void reset(RobotState state) {
        long timestampNanos = clock.now();
        engine.resetState(state.gaussian(), timestampNanos);
        for (AutomaticFusionSource source : automaticSources) source.reset(timestampNanos);
    }

    @Override public Map<String, Object> debug() {
        FusionDiagnostics diagnostics = engine.diagnostics();
        RobotState state = new RobotState(engine.state());
        Map<String, Object> result = new LinkedHashMap<>();
        result.put("pose", state.pose());
        result.put("velocity", new double[] {state.vx(), state.vy(), state.omega()});
        result.put("covarianceDiagonal", covarianceDiagonal(state.covariance()));
        result.put("historyItems", diagnostics.historyItemCount);
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

    public FusionEngine engine() { return engine; }

    /** Immutable fusion, sensor, history, and reset configuration. */
    public static final class Config {
        private final SimpleMatrix initialCovariance;
        private final SimpleMatrix octoQuadMotionCovariance;
        private final SimpleMatrix aprilTagCovariance;
        private final SimpleMatrix sharedCameraCovarianceFloor;
        private final double aprilTagConsensusGateSquared;
        private final double aprilTagDisagreementInflation;
        private final Pose2 fieldConventionTransform;
        private final long historyNanos;

        public Config(SimpleMatrix initialCovariance, SimpleMatrix octoQuadMotionCovariance,
                      SimpleMatrix aprilTagCovariance,
                      SimpleMatrix sharedCameraCovarianceFloor,
                      double aprilTagConsensusGateSquared,
                      double aprilTagDisagreementInflation,
                      Pose2 fieldConventionTransform, long historyNanos) {
            this.initialCovariance = MatrixOps.requirePositiveDefinite(
                    initialCovariance, RobotState.DIMENSION, "initial covariance");
            this.octoQuadMotionCovariance = MatrixOps.requirePositiveDefinite(
                    octoQuadMotionCovariance, RobotState.DIMENSION,
                    "OctoQuad motion covariance");
            this.aprilTagCovariance = MatrixOps.requirePositiveDefinite(
                    aprilTagCovariance, 3, "AprilTag covariance");
            this.sharedCameraCovarianceFloor = MatrixOps.requirePositiveDefinite(
                    sharedCameraCovarianceFloor, 3, "shared camera covariance floor");
            if (!Double.isFinite(aprilTagConsensusGateSquared)
                    || aprilTagConsensusGateSquared <= 0
                    || !Double.isFinite(aprilTagDisagreementInflation)
                    || aprilTagDisagreementInflation < 0) {
                throw new IllegalArgumentException("AprilTag gate and inflation are invalid");
            }
            if (fieldConventionTransform == null) {
                throw new IllegalArgumentException("field convention transform is null");
            }
            if (historyNanos <= 0) throw new IllegalArgumentException("history duration is invalid");
            this.aprilTagConsensusGateSquared = aprilTagConsensusGateSquared;
            this.aprilTagDisagreementInflation = aprilTagDisagreementInflation;
            this.fieldConventionTransform = fieldConventionTransform;
            this.historyNanos = historyNanos;
        }

        public static Config defaults() {
            return new Config(
                    MatrixOps.diagonal(4, 4, .25, 25, 25, 4),
                    MatrixOps.diagonal(.02, .02, .002, 4, 4, .1),
                    MatrixOps.diagonal(2.25, 2.25, .04),
                    MatrixOps.diagonal(.25, .25, .01),
                    16.27, 1.0, Pose2.zero(), DEFAULT_HISTORY_NANOS);
        }

        public Config withInitialCovariance(SimpleMatrix covariance) {
            return new Config(covariance, octoQuadMotionCovariance, aprilTagCovariance,
                    sharedCameraCovarianceFloor, aprilTagConsensusGateSquared,
                    aprilTagDisagreementInflation, fieldConventionTransform, historyNanos);
        }

        public SimpleMatrix initialCovariance() { return initialCovariance.copy(); }
        public SimpleMatrix octoQuadMotionCovariance() {
            return octoQuadMotionCovariance.copy();
        }
        public SimpleMatrix aprilTagCovariance() { return aprilTagCovariance.copy(); }
        public SimpleMatrix sharedCameraCovarianceFloor() {
            return sharedCameraCovarianceFloor.copy();
        }
        public double aprilTagConsensusGateSquared() { return aprilTagConsensusGateSquared; }
        public double aprilTagDisagreementInflation() {
            return aprilTagDisagreementInflation;
        }
        public Pose2 fieldConventionTransform() { return fieldConventionTransform; }
        public long historyNanos() { return historyNanos; }
    }
}
