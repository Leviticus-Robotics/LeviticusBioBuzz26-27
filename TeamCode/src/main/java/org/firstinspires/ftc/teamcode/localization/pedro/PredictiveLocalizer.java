package org.firstinspires.ftc.teamcode.localization.pedro;

import com.pedropathing.localization.Localizer;
import com.pedropathing.localization.MotionState;
import com.pedropathing.math.Pose;
import java.util.LinkedHashMap;
import java.util.Map;
import org.firstinspires.ftc.teamcode.fusion.Gaussian;
import org.firstinspires.ftc.teamcode.localization.prediction.ProcessModel;
import dev.frozenmilk.dairy.mercurial.processes.EventManager;

/** Authoritative fused localization plus an explicitly requested, non-feedback prediction. */
public final class PredictiveLocalizer<I> implements Localizer, EventManager.Handler<I> {
    private final FusedLocalizer fusedLocalizer;
    private final ProcessModel<I> processModel;
    private I currentInput;

    public PredictiveLocalizer(FusedLocalizer fusedLocalizer,
                               ProcessModel<I> processModel, I initialInput) {
        if (fusedLocalizer == null) throw new IllegalArgumentException("fused localizer is null");
        if (processModel == null) throw new IllegalArgumentException("process model is null");
        if (initialInput == null) throw new IllegalArgumentException("initial input is null");
        this.fusedLocalizer = fusedLocalizer;
        this.processModel = processModel;
        currentInput = initialInput;
    }

    /** Predicts beyond the latest corrected state without feeding the result back. */
    public synchronized MotionState predict(double elapsedSeconds) {
        return processModel.predict(fusedLocalizer.state(), currentInput, elapsedSeconds);
    }

    public synchronized I currentInput() { return currentInput; }

    public synchronized void updateInput(I input) {
        if (input == null) throw new IllegalArgumentException("input is null");
        currentInput = input;
    }

    @Override public EventManager.EventHandled<I> handleEvent(I input) {
        updateInput(input);
        return EventManager.EventHandled.ok();
    }

    @Override public void update() { fusedLocalizer.update(); }
    @Override public MotionState state() { return fusedLocalizer.state(); }
    @Override public void setPose(Pose pose) { fusedLocalizer.setPose(pose); }
    @Override public void reset() { fusedLocalizer.reset(); }
    @Override public Map<String, Object> debug() {
        Map<String, Object> result = new LinkedHashMap<>(fusedLocalizer.debug());
        result.put("predictionInput", currentInput);
        return result;
    }

    public boolean addAbsoluteMeasurement(String source, long timestampNanos,
                                          Gaussian measurement) {
        return fusedLocalizer.addAbsoluteMeasurement(source, timestampNanos, measurement);
    }

    public FusedLocalizer fusedLocalizer() { return fusedLocalizer; }
}
