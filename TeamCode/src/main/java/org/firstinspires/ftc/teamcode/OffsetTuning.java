package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.canvas.Canvas;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.pedropathing.follower.Follower;
import com.pedropathing.math.Pose;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.constants.Constants;

@TeleOp
@Config
public class OffsetTuning extends OpMode {
    FtcDashboard dashboard = FtcDashboard.getInstance();
    Follower follower;
    public static Pose startPose = new Pose(0, 0, 0);//heading in deg
    Pose robotPose;

    @Override
    public void init() {
        follower = Constants.create(hardwareMap);
        follower.setPose(startPose.withHeading(Math.toRadians(startPose.heading())));

    }

    @Override
    public void loop() {
        follower.update();
        robotPose = follower.pose();
        drawRobotPose(robotPose);

        telemetry.addData("X (in)", "%.2f", robotPose.x());
        telemetry.addData("Y (in)", "%.2f", robotPose.y());
        telemetry.addData("Heading (deg)", Math.toDegrees(robotPose.heading()));
        telemetry.update();
    }

    private void drawRobotPose(Pose pose) {
        double x = pose.x();
        double y = pose.y();
        double heading = pose.heading(); // radians

        TelemetryPacket packet = new TelemetryPacket();
        Canvas canvas = packet.fieldOverlay();


        canvas.setStroke("#00FF00")
                .setStrokeWidth(2)
                .strokeCircle(x, y, 9);

        packet.put("X (in)", pose.x());
        packet.put("Y (in)", pose.y());
        packet.put("T (in)", Constants.vx);
        packet.put("Heading (deg)", Math.toDegrees(heading));

        dashboard.sendTelemetryPacket(packet);
    }

}
