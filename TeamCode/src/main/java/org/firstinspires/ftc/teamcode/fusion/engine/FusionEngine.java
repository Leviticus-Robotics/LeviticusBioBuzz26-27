package org.firstinspires.ftc.teamcode.fusion.engine;

import java.util.ArrayList;
import java.util.Collections;
import java.util.Comparator;
import java.util.LinkedHashMap;
import java.util.LinkedHashSet;
import java.util.List;
import java.util.Map;
import java.util.NavigableMap;
import java.util.TreeMap;
import org.ejml.data.DMatrixRMaj;
import org.ejml.data.DMatrixSparseCSC;
import org.ejml.data.DMatrixSparseTriplet;
import org.ejml.interfaces.linsol.LinearSolverSparse;
import org.ejml.ops.DConvertMatrixStruct;
import org.ejml.simple.SimpleMatrix;
import org.ejml.sparse.FillReducing;
import org.ejml.sparse.csc.factory.LinearSolverFactory_DSCC;
import org.firstinspires.ftc.teamcode.fusion.core.Gaussian;
import org.firstinspires.ftc.teamcode.fusion.core.MatrixOps;
import org.firstinspires.ftc.teamcode.fusion.factor.BetweenFactor;
import org.firstinspires.ftc.teamcode.fusion.factor.Factor;
import org.firstinspires.ftc.teamcode.fusion.factor.LinearizedFactor;
import org.firstinspires.ftc.teamcode.fusion.factor.StateValues;

/** Fixed-lag nonlinear factor solver over timestamped Gaussian states. */
public final class FusionEngine<I> {
    private static final int MAX_ITERATIONS = 5;
    private static final double STEP_TOLERANCE = 1e-8;
    private static final double DIAGONAL_DAMPING = 1e-9;

    private final ProcessModel<I> processModel;
    private final int stateDimension;
    private final long maximumHistoryNanos;
    private final List<Event> events = new ArrayList<>();
    private final Map<String, Long> latestSourceTimestamps = new LinkedHashMap<>();

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
                        long maximumHistoryNanos) {
        if (initialState == null) throw new IllegalArgumentException("initial state is null");
        if (processModel == null) throw new IllegalArgumentException("process model is null");
        if (initialInput == null) throw new IllegalArgumentException("initial input is null");
        if (initialTimestampNanos < 0) throw new IllegalArgumentException("initial timestamp is negative");
        if (maximumHistoryNanos <= 0) throw new IllegalArgumentException("history duration must be positive");
        baseline = initialState;
        estimate = initialState;
        stateDimension = initialState.dimension();
        baselineTimestampNanos = initialTimestampNanos;
        estimateTimestampNanos = initialTimestampNanos;
        this.processModel = processModel;
        baselineInput = initialInput;
        activeInput = initialInput;
        this.maximumHistoryNanos = maximumHistoryNanos;
    }

    public synchronized Gaussian state() { return estimate; }
    public synchronized long timestampNanos() { return estimateTimestampNanos; }

    public synchronized void setInput(long timestampNanos, I input) {
        if (input == null) throw new IllegalArgumentException("input is null");
        requireSupportedTimestamp(timestampNanos);
        commitTrial(new InputEvent(timestampNanos, nextSequence++, input),
                timestampNanos <= estimateTimestampNanos);
    }

    public synchronized void predictTo(long timestampNanos) {
        requireSupportedTimestamp(timestampNanos);
        commitTrial(new TickEvent(timestampNanos, nextSequence++),
                timestampNanos <= estimateTimestampNanos);
    }

    public synchronized FactorApplicationResult applyFactor(Factor factor) {
        String error = validateFactor(factor);
        if (error != null) return reject(error);
        FactorEvent candidate = new FactorEvent(factor.timestampNanos(), nextSequence++, factor);
        List<Event> trial = new ArrayList<>(events);
        trial.add(candidate);
        long start = System.nanoTime();
        try {
            SolveResult<I> result = solve(baseline, baselineTimestampNanos, baselineInput, trial);
            events.add(candidate);
            commit(result, factor.timestampNanos() <= estimateTimestampNanos,
                    System.nanoTime() - start);
            acceptFactor(factor);
            pruneIfNeeded();
            return FactorApplicationResult.accepted();
        } catch (IllegalArgumentException exception) {
            return reject(messageOrDefault(exception, "factor is invalid"));
        }
    }

    public synchronized void resetState(Gaussian state, long timestampNanos) {
        if (state == null) throw new IllegalArgumentException("state is null");
        if (timestampNanos < 0) throw new IllegalArgumentException("timestamp is negative");
        if (state.dimension() != stateDimension) throw new IllegalArgumentException("state dimension changed");
        baseline = state;
        estimate = state;
        baselineTimestampNanos = timestampNanos;
        estimateTimestampNanos = timestampNanos;
        baselineInput = activeInput;
        events.clear();
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
        List<Long> result = new ArrayList<>();
        for (Event event : sorted(events)) result.add(event.timestampNanos);
        return Collections.unmodifiableList(result);
    }

    private void commitTrial(Event candidate, boolean delayed) {
        List<Event> trial = new ArrayList<>(events);
        trial.add(candidate);
        long start = System.nanoTime();
        SolveResult<I> result = solve(baseline, baselineTimestampNanos, baselineInput, trial);
        events.add(candidate);
        commit(result, delayed, System.nanoTime() - start);
        pruneIfNeeded();
    }

    private void commit(SolveResult<I> result, boolean delayed, long durationNanos) {
        estimate = result.latestState;
        estimateTimestampNanos = result.latestTimestamp;
        activeInput = result.activeInput;
        processModelEvaluationCount += result.processEvaluations;
        lastRecomputationDurationNanos = durationNanos;
        if (delayed) recomputationCount++;
    }

    private SolveResult<I> solve(Gaussian prior, long priorTimestamp, I priorInput,
                                 List<Event> sourceEvents) {
        ProblemBuilder builder = new ProblemBuilder(priorTimestamp, priorInput);
        for (Event event : sorted(sourceEvents)) event.contribute(builder);
        List<Long> timestamps = new ArrayList<>(builder.timestamps);
        Collections.sort(timestamps);
        Map<Long, double[]> values = initialValues(prior, priorTimestamp, priorInput,
                timestamps, builder.inputs);
        List<Factor> factors = new ArrayList<>();
        factors.add(new PriorFactor(priorTimestamp, prior));
        for (int i = 1; i < timestamps.size(); i++) {
            long from = timestamps.get(i - 1);
            long to = timestamps.get(i);
            factors.add(new ProcessFactor(from, to, inputAt(builder.inputs, from, priorInput)));
        }
        factors.addAll(builder.factors);

        LinearSystem system = null;
        long processEvaluations = 0;
        for (int iteration = 0; iteration < MAX_ITERATIONS; iteration++) {
            system = linearSystem(timestamps, values, factors);
            processEvaluations += timestamps.size() - 1;
            DMatrixRMaj correction = solve(system.matrix, negate(system.gradient));
            double maximumStep = 0;
            for (int node = 0; node < timestamps.size(); node++) {
                double[] state = values.get(timestamps.get(node)).clone();
                for (int component = 0; component < stateDimension; component++) {
                    double step = correction.get(node*stateDimension + component, 0);
                    state[component] += step;
                    maximumStep = Math.max(maximumStep, Math.abs(step));
                }
                values.put(timestamps.get(node), processModel.normalize(state));
            }
            if (maximumStep < STEP_TOLERANCE) break;
        }
        system = linearSystem(timestamps, values, factors);
        long latestTimestamp = timestamps.get(timestamps.size() - 1);
        Gaussian latest = new Gaussian(values.get(latestTimestamp),
                latestMarginal(system.matrix, timestamps.size() - 1));
        return new SolveResult<>(latest, latestTimestamp,
                inputAt(builder.inputs, latestTimestamp, priorInput), processEvaluations);
    }

    private Map<Long, double[]> initialValues(Gaussian prior, long priorTimestamp, I priorInput,
                                              List<Long> timestamps, NavigableMap<Long, I> inputs) {
        Map<Long, double[]> values = new LinkedHashMap<>();
        values.put(priorTimestamp, prior.mean());
        for (int i = 1; i < timestamps.size(); i++) {
            long from = timestamps.get(i - 1);
            long to = timestamps.get(i);
            double[] predicted = processModel.predictMean(values.get(from),
                    inputAt(inputs, from, priorInput), (to - from)*1e-9);
            values.put(to, processModel.normalize(predicted));
        }
        return values;
    }

    private LinearSystem linearSystem(List<Long> timestamps, Map<Long, double[]> values,
                                      List<Factor> factors) {
        int variables = timestamps.size()*stateDimension;
        Map<Long, Integer> nodeIndex = new LinkedHashMap<>();
        for (int i = 0; i < timestamps.size(); i++) nodeIndex.put(timestamps.get(i), i);
        StateValues lookup = new MapStateValues(values);
        Map<Long, Double> informationEntries = new LinkedHashMap<>();
        DMatrixRMaj gradient = new DMatrixRMaj(variables, 1);
        for (Factor factor : factors) {
            LinearizedFactor linearized = factor.linearize(lookup);
            SimpleMatrix information = MatrixOps.inverse(linearized.covariance);
            SimpleMatrix residual = MatrixOps.column(linearized.residual);
            for (int a = 0; a < linearized.timestamps.length; a++) {
                Integer aNode = nodeIndex.get(linearized.timestamps[a]);
                if (aNode == null) throw new IllegalArgumentException("factor references unavailable state");
                SimpleMatrix ja = linearized.jacobians[a];
                requireJacobianDimension(ja);
                SimpleMatrix ga = ja.transpose().mult(information).mult(residual);
                int aOffset = aNode*stateDimension;
                for (int row = 0; row < stateDimension; row++) {
                    gradient.add(aOffset + row, 0, ga.get(row, 0));
                }
                for (int b = 0; b < linearized.timestamps.length; b++) {
                    Integer bNode = nodeIndex.get(linearized.timestamps[b]);
                    if (bNode == null) throw new IllegalArgumentException("factor references unavailable state");
                    SimpleMatrix jb = linearized.jacobians[b];
                    requireJacobianDimension(jb);
                    SimpleMatrix block = ja.transpose().mult(information).mult(jb);
                    int bOffset = bNode*stateDimension;
                    for (int row = 0; row < stateDimension; row++) {
                        for (int col = 0; col < stateDimension; col++) {
                            double value = block.get(row, col);
                            if (value != 0) {
                                int matrixRow = aOffset + row;
                                int matrixCol = bOffset + col;
                                long key = ((long)matrixRow)*variables + matrixCol;
                                informationEntries.merge(key, value, Double::sum);
                            }
                        }
                    }
                }
            }
        }
        for (int i = 0; i < variables; i++) {
            long key = ((long)i)*variables + i;
            informationEntries.merge(key, DIAGONAL_DAMPING, Double::sum);
        }
        DMatrixSparseTriplet triplet = new DMatrixSparseTriplet(
                variables, variables, informationEntries.size());
        for (Map.Entry<Long, Double> entry : informationEntries.entrySet()) {
            int row = (int)(entry.getKey()/variables);
            int col = (int)(entry.getKey()%variables);
            triplet.addItem(row, col, entry.getValue());
        }
        return new LinearSystem(DConvertMatrixStruct.convert(
                triplet, new DMatrixSparseCSC(variables, variables, triplet.nz_length)), gradient);
    }

    private DMatrixRMaj solve(DMatrixSparseCSC matrix, DMatrixRMaj rightHandSide) {
        LinearSolverSparse<DMatrixSparseCSC, DMatrixRMaj> solver =
                LinearSolverFactory_DSCC.cholesky(FillReducing.NONE);
        if (!solver.setA(matrix)) throw new IllegalArgumentException("factor information is singular");
        DMatrixRMaj result = new DMatrixRMaj(matrix.numCols, rightHandSide.numCols);
        solver.solve(rightHandSide, result);
        return result;
    }

    private SimpleMatrix latestMarginal(DMatrixSparseCSC matrix, int node) {
        int offset = node*stateDimension;
        DMatrixRMaj selector = new DMatrixRMaj(matrix.numRows, stateDimension);
        for (int i = 0; i < stateDimension; i++) selector.set(offset + i, i, 1);
        DMatrixRMaj columns = solve(matrix, selector);
        SimpleMatrix covariance = new SimpleMatrix(stateDimension, stateDimension);
        for (int row = 0; row < stateDimension; row++) {
            for (int col = 0; col < stateDimension; col++) {
                covariance.set(row, col, columns.get(offset + row, col));
            }
        }
        return MatrixOps.symmetrize(covariance);
    }

    private void pruneIfNeeded() {
        if (events.isEmpty()) return;
        List<Event> ordered = sorted(events);
        long minimumTimestamp = estimateTimestampNanos - maximumHistoryNanos;
        long cutoff = Long.MIN_VALUE;
        for (Event event : ordered) {
            if (event.timestampNanos < minimumTimestamp) {
                cutoff = Math.max(cutoff, event.timestampNanos);
            }
        }
        if (cutoff == Long.MIN_VALUE || cutoff <= baselineTimestampNanos) return;
        for (Event event : events) cutoff = event.protectCutoff(cutoff);
        if (cutoff <= baselineTimestampNanos) return;
        List<Event> prefix = new ArrayList<>();
        for (Event event : events) if (event.discardableAt(cutoff)) prefix.add(event);
        SolveResult<I> summarized = solve(baseline, baselineTimestampNanos, baselineInput, prefix);
        baseline = summarized.latestState;
        baselineTimestampNanos = summarized.latestTimestamp;
        baselineInput = summarized.activeInput;
        final long finalCutoff = cutoff;
        events.removeIf(event -> event.discardableAt(finalCutoff));
        SolveResult<I> retained = solve(baseline, baselineTimestampNanos, baselineInput, events);
        commit(retained, false, lastRecomputationDurationNanos);
    }

    private String validateFactor(Factor factor) {
        if (factor == null) return "factor is null";
        if (factor.source() == null || factor.source().isEmpty()) return "factor source is empty";
        long[] timestamps = factor.timestamps();
        if (timestamps == null || timestamps.length == 0) return "factor has no timestamps";
        for (long timestamp : timestamps) {
            if (timestamp < baselineTimestampNanos) {
                return "factor timestamp predates retained history; policy=reject";
            }
        }
        return null;
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

    private void requireJacobianDimension(SimpleMatrix jacobian) {
        if (jacobian.numCols() != stateDimension) {
            throw new IllegalArgumentException("factor Jacobian state dimension is invalid");
        }
    }

    private I inputAt(NavigableMap<Long, I> inputs, long timestamp, I fallback) {
        Map.Entry<Long, I> entry = inputs.floorEntry(timestamp);
        return entry == null ? fallback : entry.getValue();
    }

    private static DMatrixRMaj negate(DMatrixRMaj source) {
        DMatrixRMaj result = source.copy();
        for (int i = 0; i < result.getNumElements(); i++) result.data[i] = -result.data[i];
        return result;
    }

    private static String messageOrDefault(IllegalArgumentException exception, String fallback) {
        return exception.getMessage() == null ? fallback : exception.getMessage();
    }

    private List<Event> sorted(List<Event> source) {
        List<Event> result = new ArrayList<>(source);
        result.sort(Comparator.comparingLong((Event event) -> event.timestampNanos)
                .thenComparingLong(event -> event.sequence));
        return result;
    }

    private final class ProblemBuilder {
        final LinkedHashSet<Long> timestamps = new LinkedHashSet<>();
        final NavigableMap<Long, I> inputs = new TreeMap<>();
        final List<Factor> factors = new ArrayList<>();
        ProblemBuilder(long priorTimestamp, I priorInput) {
            timestamps.add(priorTimestamp);
            inputs.put(priorTimestamp, priorInput);
        }
    }

    private abstract class Event {
        final long timestampNanos;
        final long sequence;
        Event(long timestampNanos, long sequence) {
            this.timestampNanos = timestampNanos;
            this.sequence = sequence;
        }
        abstract void contribute(ProblemBuilder builder);
        abstract boolean discardableAt(long cutoff);
        long protectCutoff(long cutoff) { return cutoff; }
    }

    private final class InputEvent extends Event {
        final I input;
        InputEvent(long timestampNanos, long sequence, I input) {
            super(timestampNanos, sequence); this.input = input;
        }
        @Override void contribute(ProblemBuilder builder) {
            builder.timestamps.add(timestampNanos);
            builder.inputs.put(timestampNanos, input);
        }
        @Override boolean discardableAt(long cutoff) { return timestampNanos <= cutoff; }
    }

    private final class TickEvent extends Event {
        TickEvent(long timestampNanos, long sequence) { super(timestampNanos, sequence); }
        @Override void contribute(ProblemBuilder builder) { builder.timestamps.add(timestampNanos); }
        @Override boolean discardableAt(long cutoff) { return timestampNanos <= cutoff; }
    }

    private final class FactorEvent extends Event {
        final Factor factor;
        FactorEvent(long timestampNanos, long sequence, Factor factor) {
            super(timestampNanos, sequence); this.factor = factor;
        }
        @Override void contribute(ProblemBuilder builder) {
            builder.factors.add(factor);
            for (long timestamp : factor.timestamps()) builder.timestamps.add(timestamp);
        }
        @Override boolean discardableAt(long cutoff) {
            for (long timestamp : factor.timestamps()) if (timestamp > cutoff) return false;
            return true;
        }
        @Override long protectCutoff(long cutoff) {
            long minimum = Long.MAX_VALUE;
            long maximum = Long.MIN_VALUE;
            for (long timestamp : factor.timestamps()) {
                minimum = Math.min(minimum, timestamp);
                maximum = Math.max(maximum, timestamp);
            }
            return minimum < cutoff && maximum > cutoff ? minimum : cutoff;
        }
    }

    private final class PriorFactor implements Factor {
        final long timestamp;
        final Gaussian prior;
        PriorFactor(long timestamp, Gaussian prior) { this.timestamp = timestamp; this.prior = prior; }
        @Override public String source() { return "retained-prior"; }
        @Override public long[] timestamps() { return new long[] {timestamp}; }
        @Override public double[] residual(StateValues values) {
            double[] current = values.state(timestamp);
            return processModel.residual(prior.mean(), current);
        }
        @Override public SimpleMatrix covariance(StateValues values) { return prior.covariance(); }
        @Override public double residualDifference(int row, double high, double low) {
            return processModel.residualDifference(row, high, low);
        }
    }

    private final class ProcessFactor implements BetweenFactor {
        final long from;
        final long to;
        final I input;
        ProcessFactor(long from, long to, I input) { this.from = from; this.to = to; this.input = input; }
        @Override public String source() { return "process-model"; }
        @Override public long fromTimestampNanos() { return from; }
        @Override public long toTimestampNanos() { return to; }
        @Override public double[] residual(StateValues values) {
            double[] fromState = values.state(from);
            double[] toState = values.state(to);
            double dt = (to - from)*1e-9;
            double[] predicted = processModel.predictMean(fromState, input, dt);
            return processModel.residual(predicted, toState);
        }
        @Override public SimpleMatrix covariance(StateValues values) {
            double[] fromState = values.state(from);
            return processModel.processNoise(fromState, input, (to - from)*1e-9);
        }
        @Override public double residualDifference(int row, double high, double low) {
            return processModel.residualDifference(row, high, low);
        }
    }

    private static final class MapStateValues implements StateValues {
        final Map<Long, double[]> values;
        MapStateValues(Map<Long, double[]> values) { this.values = values; }
        @Override public double[] state(long timestampNanos) {
            double[] state = values.get(timestampNanos);
            if (state == null) throw new IllegalArgumentException("state timestamp is unavailable");
            return state.clone();
        }
    }

    private static final class LinearSystem {
        final DMatrixSparseCSC matrix;
        final DMatrixRMaj gradient;
        LinearSystem(DMatrixSparseCSC matrix, DMatrixRMaj gradient) {
            this.matrix = matrix; this.gradient = gradient;
        }
    }

    private static final class SolveResult<I> {
        final Gaussian latestState;
        final long latestTimestamp;
        final I activeInput;
        final long processEvaluations;
        SolveResult(Gaussian latestState, long latestTimestamp, I activeInput,
                    long processEvaluations) {
            this.latestState = latestState;
            this.latestTimestamp = latestTimestamp;
            this.activeInput = activeInput;
            this.processEvaluations = processEvaluations;
        }
    }
}
