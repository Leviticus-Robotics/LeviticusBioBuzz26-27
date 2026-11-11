package org.firstinspires.ftc.teamcode.localization;

import com.pedropathing.revhub.localizers.OctoQuadConfig;
import com.pedropathing.revhub.localizers.OctoQuadLocalizer;
import com.qualcomm.robotcore.hardware.HardwareMap;
import java.util.List;
import java.util.function.Predicate;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.NanoClock;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.teamcode.fusion.engine.FusionDiagnostics;
import org.firstinspires.ftc.teamcode.fusion.engine.FusionEngine;
import org.firstinspires.ftc.teamcode.localization.core.ControlInput;
import org.firstinspires.ftc.teamcode.localization.core.Pose2;
import org.firstinspires.ftc.teamcode.localization.core.RobotState;
import org.firstinspires.ftc.teamcode.localization.engine.PoseVelocityProcessModel;
import org.firstinspires.ftc.teamcode.localization.factor.AbsolutePoseFactor;
import org.firstinspires.ftc.teamcode.localization.pedro.FusedLocalizer;
import org.firstinspires.ftc.teamcode.localization.sensor.apriltag.AprilTagFrameProcessor;
import org.firstinspires.ftc.teamcode.localization.sensor.octoquad.OctoQuadFactorFactory;
import org.firstinspires.ftc.teamcode.localization.sensor.octoquad.OctoQuadFusionSource;

/** Robot-level event router. Sensors are polled here, never inside FusedLocalizer. */
public final class LocalizationRuntime {
    public static final long DEFAULT_HISTORY_NANOS = 2_000_000_000L;
    public static final int DEFAULT_HISTORY_EVENTS = 256;
    private final FusionEngine<ControlInput> engine;
    private final FusedLocalizer localizer;
    private final OctoQuadFusionSource octoQuad;
    private final AprilTagFrameProcessor aprilTagFrames;
    private final NanoClock clock;

    public LocalizationRuntime(HardwareMap hardwareMap, OctoQuadConfig octoQuadConfig,
                               Predicate<AprilTagDetection> stationaryTagEligibility,
                               NanoClock clock, LocalizationConfig config, double maximumLinearVelocity,
                               double maximumAngularVelocity) {
        if (config == null) throw new IllegalArgumentException("localization config is null");
        this.clock = clock;
        long now = clock.now();
        RobotState initial = RobotState.create(Pose2.zero(), 0, 0, 0, config.initialCovariance());
        engine = new FusionEngine<>(initial.gaussian(), now,
                new PoseVelocityProcessModel(8.0, maximumLinearVelocity, maximumLinearVelocity,
                        maximumAngularVelocity, config.processNoisePerSecond()),
                ControlInput.ZERO,
                DEFAULT_HISTORY_NANOS, DEFAULT_HISTORY_EVENTS);
        OctoQuadLocalizer raw = new OctoQuadLocalizer(hardwareMap, octoQuadConfig);
        octoQuad = new OctoQuadFusionSource(raw,
                new OctoQuadFactorFactory(config.octoQuadDeltaCovariance()));
        localizer = new FusedLocalizer(engine, clock, config.initialCovariance(), octoQuad::reset);
        aprilTagFrames = new AprilTagFrameProcessor(stationaryTagEligibility, config.aprilTagCovariance(),
                config.sharedCameraCovarianceFloor(), config.aprilTagConsensusGateSquared(),
                config.aprilTagDisagreementInflation(), config.fieldConventionTransform());
    }

    public void updateOctoQuad() { octoQuad.poll(engine, clock.now()); }

    public void submitAprilTags(List<AprilTagDetection> detections) {
        AbsolutePoseFactor factor = aprilTagFrames.fromDetections(
                detections, new RobotState(engine.state()).pose());
        if (factor != null) engine.applyFactor(factor);
    }

    public void setDriveInput(double forward, double strafe, double turn) {
        engine.setInput(clock.now(), new ControlInput(forward, strafe, turn));
    }

    public void addTelemetry(Telemetry telemetry) {
        RobotState state = new RobotState(engine.state());
        FusionDiagnostics<ControlInput> diagnostics = engine.diagnostics();
        telemetry.addData("Fusion pose", state.pose());
        telemetry.addData("Fusion velocity", "%.2f, %.2f, %.2f", state.vx(), state.vy(), state.omega());
        telemetry.addData("Fusion covariance diag", diagonalString(state.covariance()));
        telemetry.addData("Fusion active input", "%.2f, %.2f, %.2f (robot)",
                diagnostics.activeInput.forwardPower, diagnostics.activeInput.strafePower,
                diagnostics.activeInput.turnPower);
        telemetry.addData("Fusion history", diagnostics.historyEventCount);
        telemetry.addData("Fusion recomputation us",
                diagnostics.lastRecomputationDurationNanos / 1000.0);
        telemetry.addData("Fusion accepted/rejected", "%d/%d",
                diagnostics.acceptedFactorCount, diagnostics.rejectedFactorCount);
        telemetry.addData("Fusion latest sources", diagnostics.latestSourceTimestamps);
        telemetry.addData("Fusion decision", diagnostics.lastDecision);
    }

    private static String diagonalString(org.ejml.simple.SimpleMatrix covariance) {
        StringBuilder result = new StringBuilder();
        for (int i = 0; i < covariance.numRows(); i++) {
            if (i > 0) result.append(", ");
            result.append(String.format("%.3g", covariance.get(i, i)));
        }
        return result.toString();
    }

    public FusedLocalizer localizer() { return localizer; }
    public FusionEngine<ControlInput> engine() { return engine; }
    public OctoQuadLocalizer rawOctoQuadLocalizer() { return octoQuad.rawLocalizer(); }
}
