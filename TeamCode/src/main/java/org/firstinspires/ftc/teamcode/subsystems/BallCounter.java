package org.firstinspires.ftc.teamcode.subsystems;

public class BallCounter {
    Transfer transfer;
    Intake intake;
    int totalBallCount;
    public BallCounter(Intake intake, Transfer transfer){
        this.intake = intake;
        this.transfer = transfer;
    }

    public int totalBalls(){
        return transfer.numBallsDetected() + intake.numBallsDetected();
    }

    public boolean moreThanXBalls(int numBalls){
        return totalBalls() > numBalls;
    }

    public boolean noBalls(){
        return totalBalls() == 0;
    }

}
