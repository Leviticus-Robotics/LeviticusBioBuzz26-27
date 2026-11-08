package org.firstinspires.ftc.teamcode.hardware;

public enum RotationDirection {
    //enum for direction
    FORWARD(1), REVERSE(-1);
    private int val;

    RotationDirection(int value){
        val = value;
    }

    public int getMultiplier(){
        return val;
    }
}