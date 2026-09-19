package org.firstinspires.ftc.teamcode;

import com.pedropathing.follower.Follower;
import com.pedropathing.math.Pose;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.pedropathing.drivetrain.DrivePowers;
import com.pedropathing.follower.ManualDrive;


@TeleOp(name = "Simple Drive Op Mode")
public class SimpleDriveOpMode extends OpMode {
    private Follower follower;
    boolean fieldCentric = false;
    DrivePowers powers;
    Pose robotPose;
    @Override
    public void init() {
        follower = Constants.create(hardwareMap);
    }

    @Override
    public void loop() {
        if(gamepad1.leftBumperWasPressed()){
            fieldCentric = !fieldCentric;
        }

        if(!fieldCentric) {
            double forward = -gamepad1.left_stick_y;
            double lateral = -gamepad1.left_stick_x;
            double rotation = gamepad1.right_stick_x;
            follower.manual(forward, lateral, rotation);
        } else{
             powers = ManualDrive.fieldCentric(
                    -gamepad1.left_stick_y,
                    -gamepad1.left_stick_x,
                    -gamepad1.right_stick_x,
                    follower.pose().heading()
            );
            follower.manual(powers);
        }
        follower.update();

        robotPose = follower.pose();

        telemetry.addData("Robot X", robotPose.x());
        telemetry.addData("Robot Y", robotPose.y());
        telemetry.addData("Robot X", Math.toDegrees(robotPose.heading()));
        telemetry.addData("Field Centric", fieldCentric);
    }
}
