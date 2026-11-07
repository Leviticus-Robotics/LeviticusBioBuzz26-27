package org.firstinspires.ftc.teamcode;

/**Specifically object for intake ball sensors class
 *
 */
public class BallSlot {
    public enum Slot{
        LEFT,
        RIGHT
    }

    private Slot ballSlot;
    private boolean detectedBall;//there is a ball in the slot

    public BallSlot(Slot ballSpot, boolean detectedBall){
        this.ballSlot = ballSpot;
        this.detectedBall = detectedBall;
    }


    public Slot getBallSlot(){
        return ballSlot;
    }
    public boolean getBallDetected(){
        return detectedBall;
    }
}
