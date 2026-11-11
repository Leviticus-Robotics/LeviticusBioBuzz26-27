package org.firstinspires.ftc.teamcode.fusion.engine;

/** Result of attempting to apply a factor. */
public final class FactorApplicationResult {
    public enum Status { ACCEPTED, REJECTED }

    public final Status status;
    public final String reason;

    private FactorApplicationResult(Status status, String reason) {
        this.status = status;
        this.reason = reason;
    }

    public static FactorApplicationResult accepted() {
        return new FactorApplicationResult(Status.ACCEPTED, "accepted");
    }

    public static FactorApplicationResult rejected(String reason) {
        return new FactorApplicationResult(Status.REJECTED, reason);
    }

    public boolean isAccepted() { return status == Status.ACCEPTED; }
}
