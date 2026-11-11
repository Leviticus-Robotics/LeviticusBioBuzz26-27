package org.firstinspires.ftc.teamcode.subsystems;

import org.firstinspires.ftc.teamcode.game.SIDE;

//connect vision to this too
public class SideMachine {
    private SIDE currentSide;
    public SideMachine(SIDE startSide){
        this.currentSide = startSide;
    }

    public void setSide(SIDE side){
        this.currentSide = side;
    }

    public SIDE getSide(){
        return currentSide;
    }
}
