package org.firstinspires.ftc.teamcode.util;

public class MathFunctions {
    public static double normalizeAngle(double angle) {
        while (angle >= Math.PI) angle -= 2 * Math.PI;
        while (angle < -Math.PI) angle += 2 * Math.PI;
        return angle;
    }
    //0 to 2 PI

    public static double normAngle0To2PI(double val){
        while(val < 0){
            val += Math.PI * 2;
        }
        while(val >= 2 * Math.PI){
            val -= Math.PI * 2;
        }
        return val;
    }
}
