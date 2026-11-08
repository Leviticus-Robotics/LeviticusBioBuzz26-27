package org.firstinspires.ftc.teamcode.fusion.engine;

import java.util.Collections;
import java.util.LinkedHashMap;
import java.util.Map;

/** Immutable telemetry snapshot from a fusion engine. */
public final class FusionDiagnostics<I> {
    public final long stateTimestampNanos;
    public final int historyEventCount;
    public final long acceptedFactorCount;
    public final long rejectedFactorCount;
    public final long recomputationCount;
    public final long processModelEvaluationCount;
    public final long lastRecomputationDurationNanos;
    public final String lastDecision;
    public final I activeInput;
    public final Map<String, Long> latestSourceTimestamps;

    FusionDiagnostics(long stateTimestampNanos, int historyEventCount, long acceptedFactorCount,
                      long rejectedFactorCount, long recomputationCount,
                      long processModelEvaluationCount, long lastRecomputationDurationNanos,
                      String lastDecision, I activeInput,
                      Map<String, Long> latestSourceTimestamps) {
        this.stateTimestampNanos = stateTimestampNanos;
        this.historyEventCount = historyEventCount;
        this.acceptedFactorCount = acceptedFactorCount;
        this.rejectedFactorCount = rejectedFactorCount;
        this.recomputationCount = recomputationCount;
        this.processModelEvaluationCount = processModelEvaluationCount;
        this.lastRecomputationDurationNanos = lastRecomputationDurationNanos;
        this.lastDecision = lastDecision;
        this.activeInput = activeInput;
        this.latestSourceTimestamps = Collections.unmodifiableMap(new LinkedHashMap<>(latestSourceTimestamps));
    }
}
