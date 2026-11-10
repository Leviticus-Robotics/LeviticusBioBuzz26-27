package org.firstinspires.ftc.teamcode.localization.sensor;

import org.firstinspires.ftc.teamcode.fusion.engine.ProcessModel;
import org.firstinspires.ftc.teamcode.fusion.factor.ProcessFactor;
import org.firstinspires.ftc.teamcode.localization.core.ControlInput;

/** Produces one ordinary process factor from the command active over each loop interval. */
public final class CommandProcessSource implements AutomaticFusionSource {
    private final ProcessModel<ControlInput> model;
    private ControlInput activeInput = ControlInput.ZERO;

    public CommandProcessSource(ProcessModel<ControlInput> model) {
        if (model == null) throw new IllegalArgumentException("process model is null");
        this.model = model;
    }

    public void setInput(double forward, double strafe, double turn) {
        activeInput = new ControlInput(forward, strafe, turn);
    }

    public ControlInput activeInput() { return activeInput; }

    @Override public ProcessFactor<ControlInput> poll(long fromTimestampNanos,
                                                       long tickTimestampNanos) {
        if (tickTimestampNanos <= fromTimestampNanos) return null;
        return model.factor(fromTimestampNanos, tickTimestampNanos, activeInput);
    }

    @Override public void reset(long timestampNanos) { activeInput = ControlInput.ZERO; }
}
