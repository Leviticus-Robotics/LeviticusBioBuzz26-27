package org.firstinspires.ftc.teamcode.fusion.engine;

import java.util.ArrayList;
import java.util.Collections;
import java.util.Comparator;
import java.util.LinkedHashMap;
import java.util.List;
import java.util.Map;
import java.util.NavigableMap;
import java.util.TreeMap;
import org.firstinspires.ftc.teamcode.fusion.core.Gaussian;
import org.firstinspires.ftc.teamcode.fusion.factor.AbsoluteFactor;
import org.firstinspires.ftc.teamcode.fusion.factor.Factor;
import org.firstinspires.ftc.teamcode.fusion.factor.RelativeFactor;

/** Timestamped Gaussian inference engine with deterministic bounded recomputation. */
public final class FusionEngine<I> {
    private final ProcessModel<I> processModel;
    private final int stateDimension;
    private final long maximumHistoryNanos;
    private final int maximumHistoryEvents;
    private final List<Event> events = new ArrayList<>();
    private final Map<String, Long> latestSourceTimestamps = new LinkedHashMap<>();
    private final NavigableMap<Long, Checkpoint<I>> checkpoints = new TreeMap<>();

    private Gaussian baseline;
    private long baselineTimestampNanos;
    private I baselineInput;
    private Gaussian estimate;
    private long estimateTimestampNanos;
    private I activeInput;
    private long nextSequence;
    private long acceptedFactors;
    private long rejectedFactors;
    private long recomputationCount;
    private long processModelEvaluationCount;
    private long lastRecomputationDurationNanos;
    private String lastDecision = "initialized";

    public FusionEngine(Gaussian initialState, long initialTimestampNanos,
                        ProcessModel<I> processModel, I initialInput,
                        long maximumHistoryNanos, int maximumHistoryEvents) {
        if (initialState == null) throw new IllegalArgumentException("initial state is null");
        if (processModel == null) throw new IllegalArgumentException("process model is null");
        if (initialTimestampNanos < 0) throw new IllegalArgumentException("initial timestamp is negative");
        if (maximumHistoryNanos <= 0 || maximumHistoryEvents <= 0) {
            throw new IllegalArgumentException("history limits must be positive");
        }
        if (initialInput == null) throw new IllegalArgumentException("initial input is null");
        this.baseline = initialState;
        this.estimate = initialState;
        this.stateDimension = initialState.dimension();
        this.baselineTimestampNanos = initialTimestampNanos;
        this.estimateTimestampNanos = initialTimestampNanos;
        this.processModel = processModel;
        this.baselineInput = initialInput;
        this.activeInput = initialInput;
        this.maximumHistoryNanos = maximumHistoryNanos;
        this.maximumHistoryEvents = maximumHistoryEvents;
        checkpoints.put(initialTimestampNanos, new Checkpoint<>(initialState, initialInput));
    }

    public synchronized Gaussian state() { return estimate; }
    public synchronized long timestampNanos() { return estimateTimestampNanos; }

    public synchronized void setInput(long timestampNanos, I input) {
        if (input == null) throw new IllegalArgumentException("input is null");
        requireSupportedTimestamp(timestampNanos);
        submit(new InputEvent<>(timestampNanos, nextSequence++, input));
    }

    public synchronized void predictTo(long timestampNanos) {
        requireSupportedTimestamp(timestampNanos);
        submit(new TickEvent(timestampNanos, nextSequence++));
    }

    public synchronized FactorApplicationResult applyFactor(Factor factor) {
        String validationError = validateFactor(factor);
        if (validationError != null) return reject(validationError);

        FactorEvent candidate = new FactorEvent(factor.timestampNanos(), nextSequence++, factor);
        if (candidate.timestampNanos > estimateTimestampNanos) {
            try {
                FactorApplicationResult result = appendForward(candidate);
                if (!result.isAccepted()) return reject(result.reason);
                acceptFactor(factor);
                return result;
            } catch (IllegalArgumentException exception) {
                return reject(messageOrDefault(exception, "factor is invalid"));
            }
        }

        List<Event> trialEvents = new ArrayList<>(events);
        trialEvents.add(candidate);
        RecomputeResult<I> trial;
        try {
            trial = recompute(trialEvents, candidate.sequence);
        } catch (IllegalArgumentException exception) {
            return reject(messageOrDefault(exception, "factor is invalid"));
        }
        if (trial.candidateResult == null || !trial.candidateResult.isAccepted()) {
            return reject(trial.candidateResult == null
                    ? "factor could not be evaluated" : trial.candidateResult.reason);
        }

        events.add(candidate);
        commitRecomputation(trial);
        acceptFactor(factor);
        return trial.candidateResult;
    }

    public synchronized void resetState(Gaussian state, long timestampNanos) {
        if (state == null) throw new IllegalArgumentException("state is null");
        if (timestampNanos < 0) throw new IllegalArgumentException("timestamp is negative");
        requireStateDimension(state, "reset state");
        baseline = state;
        estimate = state;
        baselineTimestampNanos = timestampNanos;
        estimateTimestampNanos = timestampNanos;
        baselineInput = activeInput;
        events.clear();
        checkpoints.clear();
        checkpoints.put(timestampNanos, new Checkpoint<>(state, activeInput));
        latestSourceTimestamps.clear();
        lastDecision = "state reset";
    }

    public synchronized FusionDiagnostics<I> diagnostics() {
        return new FusionDiagnostics<>(estimateTimestampNanos, events.size(),
                acceptedFactors, rejectedFactors, recomputationCount,
                processModelEvaluationCount, lastRecomputationDurationNanos,
                lastDecision, activeInput, latestSourceTimestamps);
    }

    public synchronized List<Long> retainedEventTimestamps() {
        List<Long> timestamps = new ArrayList<>();
        for (Event event : sorted(events)) timestamps.add(event.timestampNanos);
        return Collections.unmodifiableList(timestamps);
    }

    private String validateFactor(Factor factor) {
        if (factor == null) return "factor is null";
        if (!(factor instanceof AbsoluteFactor) && !(factor instanceof RelativeFactor)) {
            return "unsupported factor type";
        }
        if (factor.source() == null || factor.source().isEmpty()) return "factor source is empty";
        if (factor.timestampNanos() < 0) return "factor timestamp is invalid";
        if (factor.timestampNanos() < baselineTimestampNanos) {
            return "factor timestamp predates retained history; policy=reject";
        }
        if (factor instanceof RelativeFactor) {
            RelativeFactor relative = (RelativeFactor) factor;
            if (relative.fromTimestampNanos() < 0
                    || factor.timestampNanos() < relative.fromTimestampNanos()) {
                return "factor timestamps are invalid";
            }
            if (relative.fromTimestampNanos() < baselineTimestampNanos) {
                return "factor timestamp predates retained history; policy=reject";
            }
        }
        return null;
    }

    private void submit(Event event) {
        if (event.timestampNanos > estimateTimestampNanos) {
            appendForward(event);
        } else {
            events.add(event);
            commitRecomputation(recompute(events, -1));
        }
    }

    private FactorApplicationResult appendForward(Event event) {
        Cursor<I> cursor = new Cursor<>(estimate, estimateTimestampNanos, activeInput);
        FactorApplicationResult result = applyEvent(event, cursor, checkpoints);
        if (result != null && !result.isAccepted()) return result;

        events.add(event);
        estimate = cursor.state;
        estimateTimestampNanos = cursor.timestampNanos;
        activeInput = cursor.input;
        processModelEvaluationCount += cursor.processModelEvaluations;
        prune();
        return result;
    }

    @SuppressWarnings("unchecked")
    private FactorApplicationResult applyEvent(Event event, Cursor<I> cursor,
                                               NavigableMap<Long, Checkpoint<I>> history) {
        if (event.timestampNanos < cursor.timestampNanos) {
            throw new IllegalArgumentException("events are not chronological");
        }
        if (event.timestampNanos > cursor.timestampNanos) {
            double elapsed = (event.timestampNanos - cursor.timestampNanos) * 1e-9;
            Gaussian predicted = processModel.predict(cursor.state, cursor.input, elapsed);
            if (predicted == null) throw new IllegalArgumentException("process model returned null");
            requireStateDimension(predicted, "process model");
            cursor.state = predicted;
            cursor.timestampNanos = event.timestampNanos;
            cursor.processModelEvaluations++;
        }

        FactorApplicationResult result = null;
        if (event instanceof InputEvent) {
            cursor.input = ((InputEvent<I>) event).input;
        } else if (event instanceof FactorEvent) {
            Factor factor = ((FactorEvent) event).factor;
            Gaussian updated;
            if (factor instanceof RelativeFactor) {
                RelativeFactor relative = (RelativeFactor) factor;
                Checkpoint<I> start = history.get(relative.fromTimestampNanos());
                Gaussian startState = relative.fromTimestampNanos() == cursor.timestampNanos
                        ? cursor.state : start == null ? null : start.state;
                if (startState == null) {
                    return FactorApplicationResult.rejected(
                            "relative factor start timestamp is not retained");
                }
                updated = relative.apply(startState);
            } else if (factor instanceof AbsoluteFactor) {
                updated = ((AbsoluteFactor) factor).apply(cursor.state);
            } else {
                return FactorApplicationResult.rejected("unsupported factor type");
            }
            if (updated == null) throw new IllegalArgumentException("factor returned null");
            requireStateDimension(updated, "factor");
            cursor.state = updated;
            result = FactorApplicationResult.accepted();
        }

        history.put(cursor.timestampNanos, new Checkpoint<>(cursor.state, cursor.input));
        return result;
    }

    private RecomputeResult<I> recompute(List<Event> sourceEvents, long candidateSequence) {
        long startNanos = System.nanoTime();
        Cursor<I> cursor = new Cursor<>(baseline, baselineTimestampNanos, baselineInput);
        NavigableMap<Long, Checkpoint<I>> rebuilt = new TreeMap<>();
        rebuilt.put(cursor.timestampNanos, new Checkpoint<>(cursor.state, cursor.input));
        FactorApplicationResult candidateResult = null;

        for (Event event : sorted(sourceEvents)) {
            FactorApplicationResult result = applyEvent(event, cursor, rebuilt);
            if (event.sequence == candidateSequence) candidateResult = result;
        }
        return new RecomputeResult<>(cursor, rebuilt, candidateResult,
                System.nanoTime() - startNanos);
    }

    private void commitRecomputation(RecomputeResult<I> result) {
        estimate = result.cursor.state;
        estimateTimestampNanos = result.cursor.timestampNanos;
        activeInput = result.cursor.input;
        checkpoints.clear();
        checkpoints.putAll(result.checkpoints);
        recomputationCount++;
        processModelEvaluationCount += result.cursor.processModelEvaluations;
        lastRecomputationDurationNanos = result.durationNanos;
        prune();
    }

    private void acceptFactor(Factor factor) {
        acceptedFactors++;
        lastDecision = "accepted " + factor.source();
        latestSourceTimestamps.merge(factor.source(), factor.timestampNanos(), Math::max);
    }

    private FactorApplicationResult reject(String reason) {
        rejectedFactors++;
        lastDecision = "rejected: " + reason;
        return FactorApplicationResult.rejected(reason);
    }

    private void requireSupportedTimestamp(long timestampNanos) {
        if (timestampNanos < baselineTimestampNanos) {
            throw new IllegalArgumentException("timestamp predates retained history");
        }
    }

    private void requireStateDimension(Gaussian state, String source) {
        if (state.dimension() != stateDimension) {
            throw new IllegalArgumentException(source + " changed the state dimension");
        }
    }

    private void prune() {
        if (events.isEmpty()) return;
        List<Event> ordered = sorted(events);
        long minimumTimestamp = estimateTimestampNanos - maximumHistoryNanos;
        long cutoff = Long.MIN_VALUE;
        while (!ordered.isEmpty() && (ordered.size() > maximumHistoryEvents
                || ordered.get(0).timestampNanos < minimumTimestamp)) {
            cutoff = ordered.get(0).timestampNanos;
            final long removingTimestamp = cutoff;
            ordered.removeIf(event -> event.timestampNanos == removingTimestamp);
        }
        if (cutoff == Long.MIN_VALUE) return;

        Checkpoint<I> checkpoint = checkpoints.get(cutoff);
        if (checkpoint != null) {
            baseline = checkpoint.state;
            baselineInput = checkpoint.input;
            baselineTimestampNanos = cutoff;
            final long finalCutoff = cutoff;
            events.removeIf(event -> event.timestampNanos <= finalCutoff);
            checkpoints.headMap(finalCutoff, false).clear();
        }
    }

    private static String messageOrDefault(IllegalArgumentException exception, String fallback) {
        return exception.getMessage() == null ? fallback : exception.getMessage();
    }

    private static List<Event> sorted(List<Event> source) {
        List<Event> result = new ArrayList<>(source);
        result.sort(Comparator.comparingLong((Event event) -> event.timestampNanos)
                .thenComparingInt(Event::priority)
                .thenComparingLong(event -> event.sequence));
        return result;
    }

    private abstract static class Event {
        final long timestampNanos;
        final long sequence;
        Event(long timestampNanos, long sequence) {
            this.timestampNanos = timestampNanos;
            this.sequence = sequence;
        }
        abstract int priority();
    }

    private static final class InputEvent<I> extends Event {
        final I input;
        InputEvent(long timestampNanos, long sequence, I input) {
            super(timestampNanos, sequence);
            this.input = input;
        }
        @Override int priority() { return 0; }
    }

    private static final class FactorEvent extends Event {
        final Factor factor;
        FactorEvent(long timestampNanos, long sequence, Factor factor) {
            super(timestampNanos, sequence);
            this.factor = factor;
        }
        @Override int priority() { return factor instanceof RelativeFactor ? 1 : 2; }
    }

    private static final class TickEvent extends Event {
        TickEvent(long timestampNanos, long sequence) { super(timestampNanos, sequence); }
        @Override int priority() { return 3; }
    }

    private static final class Checkpoint<I> {
        final Gaussian state;
        final I input;
        Checkpoint(Gaussian state, I input) {
            this.state = state;
            this.input = input;
        }
    }

    private static final class Cursor<I> {
        Gaussian state;
        long timestampNanos;
        I input;
        long processModelEvaluations;
        Cursor(Gaussian state, long timestampNanos, I input) {
            this.state = state;
            this.timestampNanos = timestampNanos;
            this.input = input;
        }
    }

    private static final class RecomputeResult<I> {
        final Cursor<I> cursor;
        final NavigableMap<Long, Checkpoint<I>> checkpoints;
        final FactorApplicationResult candidateResult;
        final long durationNanos;
        RecomputeResult(Cursor<I> cursor,
                        NavigableMap<Long, Checkpoint<I>> checkpoints,
                        FactorApplicationResult candidateResult,
                        long durationNanos) {
            this.cursor = cursor;
            this.checkpoints = checkpoints;
            this.candidateResult = candidateResult;
            this.durationNanos = durationNanos;
        }
    }
}
