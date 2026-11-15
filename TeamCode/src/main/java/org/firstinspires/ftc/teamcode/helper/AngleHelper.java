package org.firstinspires.ftc.teamcode.helper;

import org.firstinspires.ftc.teamcode.util.Angle;

/**Specifically for turret encoder class to send both absolute and incrmental data in one notification
 *
 */
public class AngleHelper {
    public enum AngleType{
        Absolute,
        Incremental
    }

    private AngleType angleType;
    private Angle angle;//there is a ball in the slot

    public AngleHelper(Angle angle, AngleType angleType){
        this.angle = angle;
        this.angleType = angleType;
    }


    public Angle getAngle(){
        return angle;
    }
    public AngleType getAngleType(){
        return angleType;
    }
}
