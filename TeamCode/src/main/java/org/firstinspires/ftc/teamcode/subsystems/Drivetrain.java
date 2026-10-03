package org.firstinspires.ftc.teamcode.subsystems;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.config.Config;
import com.pedropathing.drivetrain.DrivePowers;
import com.pedropathing.localization.MotionState;
import com.pedropathing.math.Pose;

import org.jetbrains.annotations.NotNull;

import dev.frozenmilk.dairy.mercurial.pedro.MercurialFollower;
import dev.frozenmilk.dairy.mercurial.processes.Channel;
import dev.frozenmilk.dairy.mercurial.processes.EventManager;
import dev.frozenmilk.dairy.mercurial.processes.EventManager.EventHandled;
import dev.frozenmilk.dairy.mercurial.processes.StateMachine;
import kotlin.time.TimeMark;

@Config
public class Drivetrain extends StateMachine {
    private MercurialFollower   follower;
    private Channel<MotionState> motionStates = Channel.single();//handle localization input
    private Channel<Mode> switches = Channel.queue();//handle mode switching
    private Channel<DrivePowers> drivePowers = Channel.single();//handle drive inputs

    public Drivetrain(MercurialFollower follower){
        this.follower = follower;
        follower.localizationEventManager().addHandler(motionStateForwarder);
    }

    private Mode manual = new Mode(){
        @NonNull
        @Override
        public Mode eval() {
            DrivePowers powers = drivePowers.poll();
            if(powers != null){
                follower.manual(powers);
            }

            Mode nextMode = switches.poll();
            if(nextMode == null){
                return this;
            } return null;
        }
        @Override
        public String toString(){
            return "Manual";
        }
    };

    private Mode idle = new Mode(){
        @Override
        public void enter(@NotNull Mode previousMode){
            follower.idle();
        }

        @NonNull
        @Override
        public Mode eval() {
            Mode nextMode = switches.poll();
            if(nextMode == null) {
                return this;
            } return null;
        }

        @Override
        public String toString(){
            return "Idle";
        }

    };


    private class Hold implements Mode{
        Pose pose;
        boolean useScaling;
        TimeMark timeMark;//tracks time since last updated -> used for pedro hold

        private Hold(Pose pose, boolean useScaling, TimeMark timeMark){
            this.pose = pose;
            this.useScaling = useScaling;
            this.timeMark = timeMark;
        }

        private Hold(Pose pose, boolean useScaling){
            this.pose = pose;
            this.useScaling = useScaling;
            this.timeMark = null;
        }

        @NonNull
        @Override
        public Mode eval() {
            MotionState motionState = motionStates.poll();
            if(motionState != null){
                TimeMark timeMark2 = follower.hold(
                        pose,
                        useScaling,
                        motionState,
                        timeMark
                );
                Mode nextMode = switches.poll();
                if(nextMode == null){
                    return new Hold(pose, useScaling, timeMark2);
                } return null;
            }

            Mode nextMode = switches.poll();
            if(nextMode == null){
                return this;
            } return null;
        }

        @Override
        public String toString(){
            return "Hold Pose";
        }
    };


    EventManager.Handler<MotionState> motionStateForwarder = motionState -> {
        motionStates.send(motionState);
        if(this.fiber().status().alive()) {
            return EventHandled.ok();
        }
        return EventHandled.remove();
    };


    @NonNull
    @Override
    protected Mode init() {
        return manual;
    }

    public void sendDrivePowers(DrivePowers powers){
        drivePowers.send(powers);
    }

    public void manual(){
        switches.send(manual);
    }

    public void idle(){
        switches.send(idle);
    }
}
