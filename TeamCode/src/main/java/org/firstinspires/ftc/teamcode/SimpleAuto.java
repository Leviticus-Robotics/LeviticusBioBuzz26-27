package org.firstinspires.ftc.teamcode;


import com.pedropathing.follower.Follower;
import com.pedropathing.ivy.Scheduler;
import com.pedropathing.utils.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

@Autonomous(name = "Simple Auto")
//@Configurable
public class SimpleAuto extends OpMode {
    Follower follower;
    Timer timer;

    @Override
    public void init() {
        Scheduler.reset();
        follower = Constants.create(hardwareMap);
        timer = new Timer();
    }

    @Override
    public void loop() {

    }
}
