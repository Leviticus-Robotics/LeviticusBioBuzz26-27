package org.firstinspires.ftc.teamcode.fusion.factor;

import org.ejml.simple.SimpleMatrix;
import org.firstinspires.ftc.teamcode.fusion.engine.ProcessModel;

/** One timestamped conditional process distribution expressed as an ordinary relative factor. */
public final class ProcessFactor<I> implements RelativeFactor {
    private final String source;
    private final long fromTimestampNanos;
    private final long toTimestampNanos;
    private final ProcessModel<I> model;
    private final I input;
    private final double elapsedSeconds;

    public ProcessFactor(String source, long fromTimestampNanos, long toTimestampNanos,
                         ProcessModel<I> model, I input) {
        if (source == null || source.isEmpty()) throw new IllegalArgumentException("source is empty");
        if (fromTimestampNanos < 0 || toTimestampNanos <= fromTimestampNanos) {
            throw new IllegalArgumentException("process timestamps are invalid");
        }
        if (model == null || input == null) {
            throw new IllegalArgumentException("process model or input is null");
        }
        this.source = source;
        this.fromTimestampNanos = fromTimestampNanos;
        this.toTimestampNanos = toTimestampNanos;
        this.model = model;
        this.input = input;
        elapsedSeconds = (toTimestampNanos - fromTimestampNanos)*1e-9;
    }

    @Override public String source() { return source; }
    @Override public long fromTimestampNanos() { return fromTimestampNanos; }
    @Override public long toTimestampNanos() { return toTimestampNanos; }
    @Override public String observationId() {
        return source + ":" + fromTimestampNanos + ":" + toTimestampNanos;
    }

    @Override public LinearizedFactor linearize(StateValues values) {
        double[] from = values.state(fromTimestampNanos);
        double[] predicted = model.normalize(model.predictMean(from, input, elapsedSeconds));
        SimpleMatrix transition = model.transitionJacobian(from, input, elapsedSeconds);
        return new LinearizedFactor(
                model.residual(predicted, values.state(toTimestampNanos)),
                model.processNoise(from, input, elapsedSeconds),
                transition.negative(), SimpleMatrix.identity(from.length));
    }
}
