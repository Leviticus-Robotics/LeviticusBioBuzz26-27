package org.firstinspires.ftc.teamcode.fusion.factor;

import org.ejml.simple.SimpleMatrix;
import org.firstinspires.ftc.teamcode.fusion.core.Gaussian;
import org.firstinspires.ftc.teamcode.fusion.core.GaussianFusion;

/** A Gaussian measurement of a linear projection of the current state. */
public interface MeasurementFactor extends AbsoluteFactor {
    Gaussian measurement(Gaussian predictedState);
    SimpleMatrix stateToMeasurement(Gaussian predictedState);

    @Override default Gaussian apply(Gaussian currentState) {
        return GaussianFusion.fuse(
                currentState, measurement(currentState), stateToMeasurement(currentState));
    }
}
