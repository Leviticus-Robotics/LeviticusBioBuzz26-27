package org.firstinspires.ftc.teamcode.fusion.engine;

import java.util.Collections;
import java.util.LinkedHashMap;
import java.util.Map;

/** Immutable telemetry snapshot from a fusion engine. */
public final class FusionDiagnostics {
    public final long stateTimestampNanos;
    public final int historyItemCount;
    public final long acceptedFactorCount;
    public final long rejectedFactorCount;
    public final long recomputationCount;
    public final long lastRecomputationDurationNanos;
    public final String lastDecision;
    public final Map<String, Long> latestSourceTimestamps;

    FusionDiagnostics(long stateTimestampNanos, int historyItemCount, long acceptedFactorCount,
                      long rejectedFactorCount, long recomputationCount,
                      long lastRecomputationDurationNanos,
                      String lastDecision,
                      Map<String, Long> latestSourceTimestamps) {
        this.stateTimestampNanos = stateTimestampNanos;
        this.historyItemCount = historyItemCount;
        this.acceptedFactorCount = acceptedFactorCount;
        this.rejectedFactorCount = rejectedFactorCount;
        this.recomputationCount = recomputationCount;
        this.lastRecomputationDurationNanos = lastRecomputationDurationNanos;
        this.lastDecision = lastDecision;
        this.latestSourceTimestamps = Collections.unmodifiableMap(new LinkedHashMap<>(latestSourceTimestamps));
    }
}
