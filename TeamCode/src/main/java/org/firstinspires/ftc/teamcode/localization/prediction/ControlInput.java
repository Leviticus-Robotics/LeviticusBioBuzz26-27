package org.firstinspires.ftc.teamcode.localization.prediction;

/** Applied robot-centric chassis commands. Contains no predicted motion. */
public final class ControlInput {
    public static final ControlInput ZERO = new ControlInput(0, 0, 0);
    public final double forwardPower;
    public final double strafePower;
    public final double turnPower;

    public ControlInput(double forwardPower, double strafePower, double turnPower) {
        if (!Double.isFinite(forwardPower) || !Double.isFinite(strafePower)
                || !Double.isFinite(turnPower)) {
            throw new IllegalArgumentException("control input must be finite");
        }
        this.forwardPower = forwardPower;
        this.strafePower = strafePower;
        this.turnPower = turnPower;
    }
}
