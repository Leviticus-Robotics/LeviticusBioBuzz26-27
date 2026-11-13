package org.firstinspires.ftc.teamcode.helper;

import org.firstinspires.ftc.teamcode.game.HiveTargetSide;

//connect vision to this too
public class HiveSideHelper {
    private HiveTargetSide currentSide;
    public HiveSideHelper(HiveTargetSide startSide){
        this.currentSide = startSide;
    }

    public void setSide(HiveTargetSide side){
        this.currentSide = side;
    }

    public HiveTargetSide getSide(){
        return currentSide;
    }
}
