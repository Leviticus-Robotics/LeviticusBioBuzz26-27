package org.firstinspires.ftc.teamcode.fusion.engine;

import java.util.ArrayList;
import java.util.Collections;
import java.util.LinkedHashMap;
import java.util.List;
import java.util.Map;
import java.util.function.UnaryOperator;
import org.ejml.simple.SimpleMatrix;
import org.firstinspires.ftc.teamcode.fusion.core.Gaussian;
import org.firstinspires.ftc.teamcode.fusion.core.GaussianUpdater;
import org.firstinspires.ftc.teamcode.fusion.factor.Factor;
import org.firstinspires.ftc.teamcode.fusion.factor.LinearizedFactor;

/** Generic fixed-lag Gaussian filter built around one polymorphic factor batch per loop. */
public final class FusionEngine {
    private final int stateDimension;
    private final UnaryOperator<double[]> normalizer;
    private final long maximumHistoryNanos;
    private final List<Factor> queuedFactors = new ArrayList<>();
    private final Map<String, Long> latestSourceTimestamps = new LinkedHashMap<>();
    private List<Tick> ticks = new ArrayList<>();
    private Gaussian boundaryPrior;
    private Gaussian estimate;
    private long acceptedFactors;
    private long rejectedFactors;
    private long recomputationCount;
    private long lastRecomputationDurationNanos;
    private String lastDecision = "initialized";

    public FusionEngine(Gaussian initialState, long initialTimestampNanos,
                        UnaryOperator<double[]> normalizer, long maximumHistoryNanos) {
        if (initialState == null) throw new IllegalArgumentException("initial state is null");
        if (normalizer == null) throw new IllegalArgumentException("state normalizer is null");
        if (initialTimestampNanos < 0) throw new IllegalArgumentException("initial timestamp is negative");
        if (maximumHistoryNanos <= 0) throw new IllegalArgumentException("history duration must be positive");
        stateDimension = initialState.dimension();
        this.normalizer = normalizer;
        this.maximumHistoryNanos = maximumHistoryNanos;
        boundaryPrior = normalize(initialState);
        estimate = boundaryPrior;
        ticks.add(Tick.initial(initialTimestampNanos, estimate));
    }

    public synchronized Gaussian state() { return estimate; }
    public synchronized long timestampNanos() { return ticks.get(ticks.size() - 1).timestampNanos; }

    /** Queues an asynchronous absolute or relative observation for the next atomic commit. */
    public synchronized boolean submitFactor(Factor factor) {
        requireValidFactor(factor);
        if (factor.timestamps()[0] < ticks.get(0).timestampNanos) {
            reject("factor timestamp is outside retained tick history");
            return false;
        }
        queuedFactors.add(factor);
        lastDecision = "queued " + factor.source();
        return true;
    }

    /** Commits one loop tick by applying every submitted factor uniformly. */
    public synchronized void commitTick(
            long timestampNanos, List<? extends Factor> tickFactors) {
        if (tickFactors == null) throw invalid("tick factors are null");
        long previousTimestamp = timestampNanos();
        if (timestampNanos < previousTimestamp) {
            throw invalid("tick timestamps must be monotonic");
        }
        boolean advances = timestampNanos > previousTimestamp;

        List<Factor> submitted = new ArrayList<>(queuedFactors);
        submitted.addAll(tickFactors);
        for (Factor factor : submitted) requireValidFactor(factor);

        List<Tick> layout = new ArrayList<>(ticks);
        if (advances) layout.add(new Tick(timestampNanos));
        boolean historicalFactor = false;
        int earliestAffected = advances ? layout.size() - 1 : Integer.MAX_VALUE;
        int[] targets = new int[submitted.size()];
        java.util.Arrays.fill(targets, -1);
        for (int factorIndex = 0; factorIndex < submitted.size(); factorIndex++) {
            Factor factor = submitted.get(factorIndex);
            long[] endpoints = factor.timestamps();
            int targetIndex;
            if (endpoints.length == 1) {
                targetIndex = nearestTick(layout, endpoints[0]);
                if (targetIndex < 0) {
                    throw invalid("factor timestamp is outside retained tick history");
                }
            } else {
                int from = exactTick(layout, endpoints[0]);
                int to = exactTick(layout, endpoints[1]);
                if (from < 0 || to != from + 1) {
                    throw invalid("relative factor must connect consecutive retained ticks");
                }
                targetIndex = to;
            }
            Tick target = layout.get(targetIndex);
            if (isIndependent(target, submitted, targets, factorIndex, targetIndex, factor)) {
                historicalFactor |= target.timestampNanos < previousTimestamp;
                targets[factorIndex] = targetIndex;
                earliestAffected = Math.min(earliestAffected, targetIndex);
            }
        }

        try {
            long start = System.nanoTime();
            if (earliestAffected == Integer.MAX_VALUE) {
                queuedFactors.clear();
                lastRecomputationDurationNanos = System.nanoTime() - start;
                lastDecision = "committed tick";
                return;
            }
            List<Tick> trial = copySuffix(layout, earliestAffected);
            for (int i = 0; i < submitted.size(); i++) {
                if (targets[i] >= 0) trial.get(targets[i]).factors.add(submitted.get(i));
            }
            replayFrom(trial, earliestAffected);
            prune(trial);
            ticks = trial;
            estimate = ticks.get(ticks.size() - 1).posterior;
            lastRecomputationDurationNanos = System.nanoTime() - start;
            if (historicalFactor) recomputationCount++;
            for (int i = 0; i < submitted.size(); i++) {
                if (targets[i] >= 0) accept(submitted.get(i));
            }
            queuedFactors.clear();
            lastDecision = "committed tick";
        } catch (RuntimeException exception) {
            reject(exception.getMessage() == null ? "tick is invalid" : exception.getMessage());
            throw exception;
        }
    }

    public synchronized void resetState(Gaussian state, long timestampNanos) {
        if (state == null) throw new IllegalArgumentException("state is null");
        if (timestampNanos < 0) throw new IllegalArgumentException("timestamp is negative");
        if (state.dimension() != stateDimension) throw new IllegalArgumentException("state dimension changed");
        boundaryPrior = normalize(state);
        estimate = boundaryPrior;
        ticks = new ArrayList<>();
        ticks.add(Tick.initial(timestampNanos, estimate));
        queuedFactors.clear();
        latestSourceTimestamps.clear();
        lastDecision = "state reset";
    }

    public synchronized Diagnostics diagnostics() {
        int retainedItems = ticks.size() + queuedFactors.size();
        for (Tick tick : ticks) retainedItems += tick.factors.size();
        return new Diagnostics(timestampNanos(), retainedItems,
                acceptedFactors, rejectedFactors, recomputationCount,
                lastRecomputationDurationNanos, lastDecision, latestSourceTimestamps);
    }

    public synchronized List<Long> retainedTickTimestamps() {
        List<Long> result = new ArrayList<>(ticks.size());
        for (Tick tick : ticks) result.add(tick.timestampNanos);
        return Collections.unmodifiableList(result);
    }

    /** Recomputes only the suffix whose posterior can have changed. */
    private void replayFrom(List<Tick> replayTicks, int firstAffected) {
        int firstTransition = firstAffected;
        if (firstAffected == 0) {
            Tick first = replayTicks.get(0);
            first.posterior = applySingleStateFactors(boundaryPrior, first.factors);
            firstTransition = 1;
        }
        for (int index = firstTransition; index < replayTicks.size(); index++) {
            Tick previous = replayTicks.get(index - 1);
            Tick current = replayTicks.get(index);
            Gaussian joint = solveJointFactors(previous.posterior, current.factors);
            Gaussian currentMarginal = GaussianUpdater.marginal(joint, stateDimension, stateDimension);
            current.posterior = normalize(currentMarginal);
        }
    }

    private Gaussian applySingleStateFactors(Gaussian prior, List<Factor> factors) {
        List<LinearizedFactor> constraints = new ArrayList<>();
        double[] mean = prior.mean();
        for (Factor factor : factors) {
            long[] timestamps = factor.timestamps();
            if (timestamps.length != 1) {
                throw new IllegalArgumentException("boundary tick cannot contain a relative factor");
            }
            LinearizedFactor linearized = factor.linearize(requested -> {
                if (requested != timestamps[0]) throw new IllegalArgumentException("undeclared timestamp");
                return mean.clone();
            });
            requireLinearization(linearized, 1);
            constraints.add(linearized);
        }
        return normalize(GaussianUpdater.applyAll(prior, constraints));
    }

    private Gaussian solveJointFactors(Gaussian previousPrior, List<Factor> factors) {
        List<LinearizedFactor> constraints = new ArrayList<>();
        double[] previous = previousPrior.mean();
        double[] current = previous.clone();
        for (Factor factor : factors) {
            long[] endpoints = factor.timestamps();
            LinearizedFactor linearized = factor.linearize(timestamp -> {
                if (endpoints.length == 1 && timestamp == endpoints[0]) return current.clone();
                if (endpoints.length == 2 && timestamp == endpoints[0]) return previous.clone();
                if (endpoints.length == 2 && timestamp == endpoints[1]) return current.clone();
                throw new IllegalArgumentException("factor requested an undeclared timestamp");
            });
            requireLinearization(linearized, endpoints.length);
            if (endpoints.length == 1) {
                SimpleMatrix zero = new SimpleMatrix(linearized.residual.length, stateDimension);
                linearized = new LinearizedFactor(linearized.residual,
                        linearized.covariance, zero, linearized.jacobians[0]);
            }
            constraints.add(linearized);
        }
        return normalizeJoint(GaussianUpdater.solveAugmented(previousPrior, current, constraints));
    }

    private Gaussian normalize(Gaussian gaussian) {
        return new Gaussian(normalizer.apply(gaussian.mean()), gaussian.covariance());
    }

    private Gaussian normalizeJoint(Gaussian joint) {
        double[] mean = joint.mean();
        double[] previous = normalizer.apply(slice(mean, 0, stateDimension));
        double[] current = normalizer.apply(slice(mean, stateDimension, stateDimension));
        System.arraycopy(previous, 0, mean, 0, stateDimension);
        System.arraycopy(current, 0, mean, stateDimension, stateDimension);
        return new Gaussian(mean, joint.covariance());
    }

    private void prune(List<Tick> source) {
        long cutoff = source.get(source.size() - 1).timestampNanos - maximumHistoryNanos;
        int firstRetained = 0;
        while (firstRetained + 1 < source.size()
                && source.get(firstRetained).timestampNanos < cutoff) firstRetained++;
        if (firstRetained == 0) return;
        Tick boundary = source.get(firstRetained);
        Tick retainedBoundary = new Tick(boundary);
        retainedBoundary.factors.clear();
        boundaryPrior = boundary.posterior;
        source.subList(0, firstRetained).clear();
        source.set(0, retainedBoundary);
    }

    private void requireValidFactor(Factor factor) {
        if (factor == null) throw invalid("factor is null");
        if (factor.source() == null || factor.source().isEmpty()) {
            throw invalid("factor source is empty");
        }
        long[] timestamps = factor.timestamps();
        if (timestamps == null || (timestamps.length != 1 && timestamps.length != 2)) {
            throw invalid("tick filter factors must connect one or two states");
        }
        if (timestamps.length == 2 && timestamps[1] <= timestamps[0]) {
            throw invalid("relative factor timestamps are invalid");
        }
    }

    private void requireLinearization(LinearizedFactor factor, int stateCount) {
        if (factor.jacobians.length != stateCount) {
            throw new IllegalArgumentException("factor linearized the wrong number of states");
        }
        for (SimpleMatrix jacobian : factor.jacobians) {
            if (jacobian.numCols() != stateDimension) {
                throw new IllegalArgumentException("factor state dimension does not match engine");
            }
        }
    }

    private static boolean isIndependent(Tick tick, List<Factor> submitted, int[] targets,
                                         int factorIndex, int targetIndex, Factor factor) {
        String observationId = factor.observationId();
        if (observationId != null) {
            for (Factor existing : tick.factors) {
                if (observationId.equals(existing.observationId())) return false;
            }
            for (int i = 0; i < factorIndex; i++) {
                if (targets[i] == targetIndex
                        && observationId.equals(submitted.get(i).observationId())) return false;
            }
        }
        return true;
    }

    private int exactTick(List<Tick> source, long timestamp) {
        for (int i = 0; i < source.size(); i++) {
            if (source.get(i).timestampNanos == timestamp) return i;
        }
        return -1;
    }

    private int nearestTick(List<Tick> source, long timestamp) {
        if (timestamp < source.get(0).timestampNanos
                || timestamp > source.get(source.size() - 1).timestampNanos) return -1;
        int nearest = 0;
        long nearestDistance = Math.abs(source.get(0).timestampNanos - timestamp);
        for (int i = 1; i < source.size(); i++) {
            long distance = Math.abs(source.get(i).timestampNanos - timestamp);
            if (distance < nearestDistance) {
                nearest = i;
                nearestDistance = distance;
            }
        }
        return nearest;
    }

    private void accept(Factor factor) {
        acceptedFactors++;
        latestSourceTimestamps.merge(factor.source(), factor.timestampNanos(), Math::max);
    }

    private void reject(String reason) {
        rejectedFactors++;
        lastDecision = "rejected: " + reason;
    }

    private IllegalArgumentException invalid(String reason) {
        reject(reason);
        return new IllegalArgumentException(reason);
    }

    private static double[] slice(double[] values, int offset, int length) {
        double[] result = new double[length];
        System.arraycopy(values, offset, result, 0, length);
        return result;
    }

    /** Shares the immutable prefix and copies only ticks that replay may mutate. */
    private static List<Tick> copySuffix(List<Tick> source, int firstCopied) {
        List<Tick> result = new ArrayList<>(source.size());
        for (int index = 0; index < source.size(); index++) {
            Tick tick = source.get(index);
            result.add(index < firstCopied ? tick : new Tick(tick));
        }
        return result;
    }

    private static final class Tick {
        final long timestampNanos;
        final List<Factor> factors = new ArrayList<>();
        Gaussian posterior;

        Tick(long timestampNanos) {
            this.timestampNanos = timestampNanos;
        }

        Tick(Tick other) {
            timestampNanos = other.timestampNanos;
            factors.addAll(other.factors);
            posterior = other.posterior;
        }

        static Tick initial(long timestampNanos, Gaussian posterior) {
            Tick result = new Tick(timestampNanos);
            result.posterior = posterior;
            return result;
        }
    }

    /** Immutable telemetry snapshot. */
    public static final class Diagnostics {
        public final long stateTimestampNanos;
        public final int historyItemCount;
        public final long acceptedFactorCount;
        public final long rejectedFactorCount;
        public final long recomputationCount;
        public final long lastRecomputationDurationNanos;
        public final String lastDecision;
        public final Map<String, Long> latestSourceTimestamps;

        private Diagnostics(long timestamp, int items, long accepted, long rejected,
                            long recomputations, long duration, String decision,
                            Map<String, Long> sourceTimestamps) {
            stateTimestampNanos = timestamp;
            historyItemCount = items;
            acceptedFactorCount = accepted;
            rejectedFactorCount = rejected;
            recomputationCount = recomputations;
            lastRecomputationDurationNanos = duration;
            lastDecision = decision;
            latestSourceTimestamps = Collections.unmodifiableMap(
                    new LinkedHashMap<>(sourceTimestamps));
        }
    }
}
