# Engine lifecycle and timing

## One tick is one atomic inference step

`FusionEngine.commitTick(timestamp, factors)` is the central operation.

For an advancing timestamp, all process and sensor factors are supplied together. The previous posterior is the only prior. The factors must collectively make the new state observable. A same-timestamp commit applies newly queued observations without creating another state.

At a high level, commit performs:

1. Validate timestamp monotonicity, factor endpoints, and dimensions.
2. Combine asynchronously queued factors with factors collected during this loop.
3. Resolve each factor to a retained tick.
4. Deduplicate factors according to `observationId()`.
5. Find the earliest tick whose posterior can change.
6. Copy only the affected history suffix.
7. Replay from the earliest affected tick.
8. Prune history older than the configured time window.
9. Publish the trial estimate only if every operation succeeds.

This trial-and-publish behavior prevents a rejected factor from partially mutating the live estimate.

## Factor attachment rules

An absolute factor declares one timestamp. The engine attaches it to the nearest retained tick. The timestamp must fall within the retained time range.

A relative factor declares two timestamps. Both timestamps must exactly match retained ticks, and the destination must immediately follow the source. Arbitrary long-range relative factors are rejected.

These rules are why automatic sensors are sampled once per robot loop: normal relative observations naturally span consecutive ticks.

An important consequence is that a camera timestamp between loop timestamps is currently associated with the nearest tick. The engine does not insert an interpolated state at the exact camera time.

## Forward fast path

For a normal new tick with no delayed data, every old posterior is already valid. The engine evaluates only:

```text
cached posterior X(k-1)
       └── current factor batch ──> posterior X(k)
```

It does not replay the retained history. The untouched tick prefix is shared, and only the new tick is copied into the trial suffix.

## Delayed measurements

`submitFactor()` queues a timestamped observation. This is how an AprilTag result can arrive asynchronously after its image was captured.

Suppose the current state is at \(t_5\), and a factor captured at \(t_3\) arrives:

```text
t0 ─ t1 ─ t2 ─ t3 ─ t4 ─ t5
               ▲
          delayed factor
```

At the next commit, the engine:

1. Reuses the cached posterior at \(t_2\).
2. Recomputes \(t_3\) with the new factor.
3. Replays every stored factor for \(t_4\) and \(t_5\).
4. Replaces the live suffix with the corrected suffix.

It must replay subsequent ticks because each posterior depends on the preceding posterior. It does not need to replay \(t_0\) through \(t_2\), because nothing before the delayed factor changed.

This is chronological out-of-sequence measurement handling. It gives the same result as if the delayed factor had been available at its assigned tick, subject to floating-point arithmetic and the same one-pass linearization policy.

## What is stored per tick

Each retained tick stores:

- Timestamp.
- Every process and observation factor assigned to the tick.
- The resulting posterior Gaussian.

The process input is already captured inside `ProcessFactor`; the engine does not separately store an input field.

The cached posterior is both a result and a replay checkpoint.

## History pruning

The engine retains a time window controlled by `maximumHistoryNanos`. The default fused localizer window is two seconds.

When old ticks leave the window, the first retained posterior becomes `boundaryPrior`. Its old factor list is cleared because that information is already summarized by the posterior.

```text
discarded history       retained history
      ... ── [summary prior] ─ t(k+1) ─ t(k+2)
```

This includes the prior exactly once. Keeping both the summarized posterior and its original factors would double-count old evidence during replay.

A factor older than the retained boundary is rejected because the information needed to reconstruct that historical state is no longer available.

## Replay cost

Let \(N\) be retained ticks and \(d\) be the number of ticks from a delayed observation to the present:

- Ordinary advancing tick: approximately \(O(1)\) tick evaluations.
- Same-timestamp duplicate/no-op: no replay.
- Delayed observation: approximately \(O(d)\).
- Worst case at the history boundary: approximately \(O(N)\).

The implementation copies only the affected suffix and copies only the new boundary tick during pruning.

In the host benchmark with a 50 Hz loop, two-second history, OctoQuad every loop, and 10 Hz AprilTags delayed by 100 ms, the current central-difference process Jacobian measured:

| Workload | Mean | p50 | p95 |
|---|---:|---:|---:|
| All loops | 0.191 ms | 0.100 ms | 0.605 ms |
| Process + OctoQuad | 0.108 ms | 0.084 ms | 0.173 ms |
| Process + OctoQuad + delayed AprilTag | 0.525 ms | 0.456 ms | 0.882 ms |

A prior alternating A/B benchmark found only a small end-to-end median advantage for a hand-derived analytic process Jacobian. Dense Gaussian updates dominate this workload, so the implementation keeps the simpler numerical derivative. JVM scheduling can still produce occasional host outliers; medians, percentiles, and repeated robot-side measurements are more useful than one maximum.

These are desktop JVM results, not guaranteed Control Hub timings.

## Diagnostics

`FusionDiagnostics` exposes:

- Current state timestamp.
- Retained tick/factor count.
- Accepted and rejected factor counts.
- Number of historical recomputations.
- Duration of the last computation.
- Last decision text.
- Latest accepted timestamp by source.

## Rejection conditions

Common rejection reasons include:

- A new state whose factor batch leaves its information matrix singular.
- A stale timestamp outside retained history.
- A relative factor not connecting consecutive retained ticks.
- An invalid covariance or Jacobian dimension.

Rejected commits leave asynchronously queued observations available for retry. Automatically polled sources roll back their staged state, so a rejected tick cannot advance an odometry baseline independently of the engine. Callers should surface the rejection reason rather than silently continuing.

