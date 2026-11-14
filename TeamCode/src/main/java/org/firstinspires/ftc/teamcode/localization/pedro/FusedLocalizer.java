package org.firstinspires.ftc.teamcode.localization.pedro;

import com.pedropathing.localization.Localizer;
import com.pedropathing.localization.MotionState;
import com.pedropathing.math.Pose;
import com.pedropathing.math.Velocity;
import java.util.LinkedHashMap;
import java.util.Map;
import org.ejml.simple.SimpleMatrix;
import org.firstinspires.ftc.teamcode.fusion.Gaussian;
import org.firstinspires.ftc.teamcode.fusion.MatrixOps;
import org.firstinspires.ftc.teamcode.helper.NanoClock;
import org.firstinspires.ftc.teamcode.localization.prediction.LocalizationHistory;
import org.firstinspires.ftc.teamcode.localization.prediction.OdometryModel;
import org.firstinspires.ftc.teamcode.localization.prediction.PoseEstimate;

/** Pedro localizer that adds Gaussian absolute corrections and delayed replay. */
public final class FusedLocalizer implements Localizer {
    public static final long DEFAULT_HISTORY_NANOS = 2_000_000_000L;

    private final Localizer deadReckoning;
    private final NanoClock clock;
    private final Config config;
    private final OdometryModel odometryModel;
    private LocalizationHistory history;
    private Pose currentRawPose;
    private MotionState motionState;
    private long lastUpdateNanos;
    private String lastDecision = "initialized";

    public FusedLocalizer(Localizer deadReckoning, Config config) {
        this(deadReckoning, config, NanoClock.SYSTEM);
    }

    public FusedLocalizer(Localizer deadReckoning, Config config, NanoClock clock) {
        if (deadReckoning == null) throw new IllegalArgumentException("localizer is null");
        if (config == null) throw new IllegalArgumentException("config is null");
        if (clock == null) throw new IllegalArgumentException("clock is null");
        this.deadReckoning = deadReckoning;
        this.config = config;
        this.clock = clock;
        odometryModel = new OdometryModel(config.odometryVariancePerUnit);

        MotionState initial = deadReckoning.state();
        if (initial == null) initial = MotionState.zero();
        long now = clock.now();
        currentRawPose = initial.pose();
        motionState = initial;
        lastUpdateNanos = now;
        history = createHistory(now, initial.pose(), initial.velocity());
    }

    @Override public synchronized MotionState state() { return motionState; }

    public synchronized Gaussian poseEstimate() {
        return history.latestEstimate().gaussian();
    }

    public synchronized long timestampNanos() {
        return history.latestTimestampNanos();
    }

    @Override public synchronized void update() {
        deadReckoning.update();
        MotionState rawState = deadReckoning.state();
        if (rawState == null) return;
        long now = clock.now();
        if (now <= lastUpdateNanos) return;

        Pose rawPose = rawState.pose();
        Pose relativeTransform = currentRawPose.invert().compose(rawPose);
        history.advance(now, relativeTransform, rawState.velocity());
        currentRawPose = rawPose;
        lastUpdateNanos = now;
        publishLatest();
        lastDecision = "odometry propagated";
    }

    /** Adds a validated full field-pose Gaussian at its acquisition timestamp. */
    public synchronized boolean addAbsoluteMeasurement(
            String source, long timestampNanos, Gaussian measurement) {
        if (source == null || source.isEmpty()) {
            throw new IllegalArgumentException("measurement source is empty");
        }
        if (measurement == null || measurement.dimension() != 3) {
            throw new IllegalArgumentException("absolute pose measurement must be 3D");
        }
        measurement.validate();
        LocalizationHistory.MeasurementResult result =
                history.addAbsolute(source, timestampNanos, measurement);
        if (result.stateChanged) publishLatest();
        lastDecision = result.description;
        return result.accepted;
    }

    @Override public synchronized void setPose(Pose pose) {
        if (pose == null) throw new IllegalArgumentException("pose is null");
        deadReckoning.setPose(pose);
        Velocity velocity = motionState == null
                ? Velocity.zero() : motionState.velocity();
        resetHistory(pose, velocity);
    }

    @Override public synchronized void reset() {
        deadReckoning.reset();
        MotionState raw = deadReckoning.state();
        if (raw == null) raw = MotionState.zero();
        resetHistory(raw.pose(), raw.velocity());
    }

    @Override public synchronized Map<String, Object> debug() {
        Gaussian pose = poseEstimate();
        Map<String, Object> result = new LinkedHashMap<>();
        result.put("pose", motionState.pose());
        result.put("velocity", motionState.velocity());
        result.put("covarianceDiagonal", new double[] {
                pose.covariance().get(0, 0),
                pose.covariance().get(1, 1),
                pose.covariance().get(2, 2)
        });
        result.put("historyItems", history.size());
        result.put("pendingMeasurements", history.pendingCount());
        result.put("lastDecision", lastDecision);
        return result;
    }

    public Localizer deadReckoning() { return deadReckoning; }

    private LocalizationHistory createHistory(
            long timestampNanos, Pose pose, Velocity velocity) {
        return new LocalizationHistory(
                timestampNanos,
                PoseEstimate.at(pose, config.initialCovariance),
                velocity,
                odometryModel,
                config.maximumHistoryNanos);
    }

    private void publishLatest() {
        motionState = MotionState.ofVelocity(
                history.latestEstimate().pose(), history.latestVelocity());
    }

    private void resetHistory(Pose pose, Velocity velocity) {
        long now = clock.now();
        currentRawPose = pose;
        history = createHistory(now, pose, velocity);
        lastUpdateNanos = now;
        motionState = MotionState.ofVelocity(pose, velocity);
        lastDecision = "state reset";
    }

    /** Immutable pose-filter configuration. */
    public static final class Config {
        private final SimpleMatrix initialCovariance;
        private final SimpleMatrix odometryVariancePerUnit;
        private final long maximumHistoryNanos;

        public Config(SimpleMatrix initialCovariance,
                      SimpleMatrix odometryVariancePerUnit,
                      long maximumHistoryNanos) {
            this.initialCovariance = MatrixOps.requirePositiveDefinite(
                    initialCovariance, 3, "initial pose covariance");
            this.odometryVariancePerUnit = MatrixOps.requirePositiveDefinite(
                    odometryVariancePerUnit, 3, "odometry variance per unit");
            if (maximumHistoryNanos <= 0) {
                throw new IllegalArgumentException("history duration must be positive");
            }
            this.maximumHistoryNanos = maximumHistoryNanos;
        }
    }
}
