package org.firstinspires.ftc.teamcode.localization.prediction;

import com.pedropathing.math.Pose;
import com.pedropathing.math.Twist;
import com.pedropathing.math.Velocity;
import java.util.ArrayList;
import java.util.Comparator;
import java.util.HashSet;
import java.util.Iterator;
import java.util.List;
import java.util.Map;
import java.util.NavigableMap;
import java.util.Set;
import java.util.TreeMap;
import org.firstinspires.ftc.teamcode.fusion.Gaussian;

/** Retained localization timeline with interpolation, delayed correction, and replay. */
public class LocalizationHistory {
    private final OdometryModel odometryModel;
    private final long maximumHistoryNanos;
    private final NavigableMap<Long, Tick> ticks = new TreeMap<>();
    private final Set<ObservationKey> observations = new HashSet<>();
    private final List<PendingMeasurement> pendingMeasurements = new ArrayList<>();
    private PoseEstimate boundaryPrior;

    public LocalizationHistory(long timestampNanos, PoseEstimate initialEstimate,
                        Velocity velocity, OdometryModel odometryModel,
                        long maximumHistoryNanos) {
        if (odometryModel == null) throw new IllegalArgumentException("odometry model is null");
        if (maximumHistoryNanos <= 0) {
            throw new IllegalArgumentException("history duration must be positive");
        }
        this.odometryModel = odometryModel;
        this.maximumHistoryNanos = maximumHistoryNanos;
        reset(timestampNanos, initialEstimate, velocity);
    }

    public void advance(long timestampNanos, Pose relativeTransform, Velocity velocity) {
        Tick previous = ticks.lastEntry().getValue();
        PoseEstimate posterior = odometryModel.propagate(
                previous.posterior, relativeTransform);
        ticks.put(timestampNanos,
                new Tick(timestampNanos, relativeTransform, velocity, posterior));
        applyReadyMeasurements(timestampNanos);
        prune();
    }

    public MeasurementResult addAbsolute(String source, long timestampNanos,
                                  Gaussian measurement) {
        if (timestampNanos < ticks.firstKey()) return MeasurementResult.OUTSIDE_HISTORY;
        ObservationKey key = new ObservationKey(source, timestampNanos);
        if (!observations.add(key)) return MeasurementResult.DUPLICATE;
        if (timestampNanos > ticks.lastKey()) {
            pendingMeasurements.add(new PendingMeasurement(
                    timestampNanos, measurement));
            return MeasurementResult.QUEUED;
        }
        attachMeasurement(timestampNanos, measurement);
        replayFrom(timestampNanos);
        return MeasurementResult.APPLIED;
    }

    void reset(long timestampNanos, PoseEstimate estimate, Velocity velocity) {
        if (estimate == null || velocity == null) {
            throw new IllegalArgumentException("initial history state is null");
        }
        boundaryPrior = estimate;
        ticks.clear();
        observations.clear();
        pendingMeasurements.clear();
        ticks.put(timestampNanos, Tick.boundary(timestampNanos, velocity, estimate));
    }

    public PoseEstimate latestEstimate() { return ticks.lastEntry().getValue().posterior; }
    public Velocity latestVelocity() { return ticks.lastEntry().getValue().velocity; }
    public long latestTimestampNanos() { return ticks.lastKey(); }
    public int size() { return ticks.size(); }
    public int pendingCount() { return pendingMeasurements.size(); }

    private void applyReadyMeasurements(long timestampNanos) {
        if (pendingMeasurements.isEmpty()) return;
        pendingMeasurements.sort(Comparator.comparingLong(
                measurement -> measurement.timestampNanos));
        long replayTimestamp = Long.MAX_VALUE;
        Iterator<PendingMeasurement> iterator = pendingMeasurements.iterator();
        while (iterator.hasNext()) {
            PendingMeasurement pending = iterator.next();
            if (pending.timestampNanos > timestampNanos) break;
            attachMeasurement(pending.timestampNanos, pending.gaussian);
            replayTimestamp = Math.min(replayTimestamp, pending.timestampNanos);
            iterator.remove();
        }
        if (replayTimestamp != Long.MAX_VALUE) replayFrom(replayTimestamp);
    }

    private void attachMeasurement(long timestampNanos, Gaussian measurement) {
        Tick tick = ticks.get(timestampNanos);
        if (tick == null) tick = insertInterpolatedTick(timestampNanos);
        tick.measurements.add(measurement);
    }

    private Tick insertInterpolatedTick(long timestampNanos) {
        Map.Entry<Long, Tick> lowerEntry = ticks.floorEntry(timestampNanos);
        Map.Entry<Long, Tick> upperEntry = ticks.ceilingEntry(timestampNanos);
        if (lowerEntry == null || upperEntry == null
                || lowerEntry.getKey().equals(upperEntry.getKey())) {
            throw new IllegalArgumentException("cannot bracket measurement timestamp");
        }
        Tick lower = lowerEntry.getValue();
        Tick upper = upperEntry.getValue();
        double ratio = (double)(timestampNanos - lower.timestampNanos)
                / (double)(upper.timestampNanos - lower.timestampNanos);
        Pose toMeasurement = interpolateTransform(
                Pose.zero(), upper.relativeTransform, ratio);
        Pose remainder = toMeasurement.invert().compose(upper.relativeTransform);
        Velocity velocity = interpolateVelocity(lower.velocity, upper.velocity, ratio);
        PoseEstimate posterior = odometryModel.propagate(
                lower.posterior, toMeasurement);
        Tick inserted = new Tick(
                timestampNanos, toMeasurement, velocity, posterior);
        upper.relativeTransform = remainder;
        ticks.put(timestampNanos, inserted);
        return inserted;
    }

    private void replayFrom(long timestampNanos) {
        Map.Entry<Long, Tick> previousEntry = ticks.lowerEntry(timestampNanos);
        PoseEstimate previousPosterior = previousEntry == null
                ? boundaryPrior : previousEntry.getValue().posterior;
        boolean startsAtBoundary = previousEntry == null;
        for (Tick tick : ticks.tailMap(timestampNanos, true).values()) {
            PoseEstimate posterior;
            if (startsAtBoundary && tick.timestampNanos == ticks.firstKey()) {
                posterior = boundaryPrior;
            } else {
                posterior = odometryModel.propagate(
                        previousPosterior, tick.relativeTransform);
            }
            for (Gaussian measurement : tick.measurements) {
                posterior = posterior.correct(measurement);
            }
            tick.posterior = posterior;
            previousPosterior = posterior;
            startsAtBoundary = false;
        }
    }

    private void prune() {
        long cutoff = ticks.lastKey() - maximumHistoryNanos;
        while (ticks.size() > 1) {
            Long first = ticks.firstKey();
            Long second = ticks.higherKey(first);
            if (second == null || second > cutoff) break;
            ticks.pollFirstEntry();
        }
        Tick boundary = ticks.firstEntry().getValue();
        boundaryPrior = boundary.posterior;
        boundary.measurements.clear();
        boundary.relativeTransform = Pose.zero();
        observations.removeIf(key -> key.timestampNanos < boundary.timestampNanos);
    }

    private static Velocity interpolateVelocity(
            Velocity from, Velocity to, double ratio) {
        return from.plus(to.minus(from).scale(ratio));
    }

    private static Pose interpolateTransform(Pose from, Pose to, double ratio) {
        Twist delta = Twist.riemannianLog(from, to).times(ratio);
        return from.compose(Pose.zero().exp(delta));
    }

    public enum MeasurementResult {
        APPLIED(true, true, "absolute measurement fused and replayed"),
        QUEUED(true, false, "absolute measurement queued for next odometry tick"),
        DUPLICATE(false, false, "duplicate absolute measurement"),
        OUTSIDE_HISTORY(false, false, "absolute measurement outside retained history");

        public final boolean accepted;
        public final boolean stateChanged;
        public final String description;

        MeasurementResult(boolean accepted, boolean stateChanged, String description) {
            this.accepted = accepted;
            this.stateChanged = stateChanged;
            this.description = description;
        }
    }

    private static final class ObservationKey {
        final String source;
        final long timestampNanos;

        ObservationKey(String source, long timestampNanos) {
            this.source = source;
            this.timestampNanos = timestampNanos;
        }

        @Override public boolean equals(Object other) {
            if (this == other) return true;
            if (!(other instanceof ObservationKey)) return false;
            ObservationKey key = (ObservationKey) other;
            return timestampNanos == key.timestampNanos && source.equals(key.source);
        }

        @Override public int hashCode() {
            return 31*source.hashCode() + Long.hashCode(timestampNanos);
        }
    }

    private static final class PendingMeasurement {
        final long timestampNanos;
        final Gaussian gaussian;

        PendingMeasurement(long timestampNanos, Gaussian gaussian) {
            this.timestampNanos = timestampNanos;
            this.gaussian = gaussian;
        }
    }

    private static final class Tick {
        final long timestampNanos;
        Pose relativeTransform;
        final Velocity velocity;
        final List<Gaussian> measurements = new ArrayList<>();
        PoseEstimate posterior;

        Tick(long timestampNanos, Pose relativeTransform,
             Velocity velocity, PoseEstimate posterior) {
            this.timestampNanos = timestampNanos;
            this.relativeTransform = relativeTransform;
            this.velocity = velocity;
            this.posterior = posterior;
        }

        static Tick boundary(long timestampNanos, Velocity velocity,
                             PoseEstimate posterior) {
            return new Tick(timestampNanos, Pose.zero(), velocity, posterior);
        }
    }
}
