package org.firstinspires.ftc.teamcode.sensor;

import org.firstinspires.ftc.teamcode.game.HiveTargetSide;

import dev.frozenmilk.dairy.mercurial.processes.Channel;
import dev.frozenmilk.dairy.mercurial.processes.EventManager;

//connect vision to this too
public class HiveSideHelper extends EventManager<HiveTargetSide> {
    private Channel<HiveTargetSide> currentSideChannel = Channel.single();
    private HiveTargetSide lastHiveTargetSide;
    //create channel, that way only updates when there is a change

    public HiveSideHelper(HiveTargetSide startSide){
        currentSideChannel.send(startSide);
    }

    @Override
    protected void eventSource(){
        if(currentSideChannel != null){
            HiveTargetSide currentHiveTargetSide = currentSideChannel.query();
            if(currentHiveTargetSide != lastHiveTargetSide && currentHiveTargetSide != null) {
                lastHiveTargetSide = currentHiveTargetSide;
                notify(currentHiveTargetSide);
            }
        }
    }
    public void setSide(HiveTargetSide side){
        if(side != null && side != currentSideChannel.query()) {
            currentSideChannel.send(side);
        }
    }

    public HiveTargetSide getSide(){
        return currentSideChannel.query();
    }
}
