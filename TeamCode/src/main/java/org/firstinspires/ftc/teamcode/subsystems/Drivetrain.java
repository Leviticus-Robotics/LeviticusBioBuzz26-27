package org.firstinspires.ftc.teamcode.subsystems;

import androidx.annotation.NonNull;

import com.pedropathing.drivetrain.DrivePowers;
import com.pedropathing.follower.ManualDrive;
import com.pedropathing.localization.MotionState;
import com.pedropathing.math.Pose;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.PathTracker;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.constants.Constants;
import org.firstinspires.ftc.teamcode.localization.core.ControlInput;
import org.firstinspires.ftc.teamcode.localization.pedro.FusionMecanumDrivetrain;

import dev.frozenmilk.dairy.mercurial.continuations.Continuation;
import dev.frozenmilk.dairy.mercurial.continuations.Continuations;
import dev.frozenmilk.dairy.mercurial.environments.Reference;
import dev.frozenmilk.dairy.mercurial.ftc.Context;
import dev.frozenmilk.dairy.mercurial.pedro.MercurialFollower;
import dev.frozenmilk.dairy.mercurial.processes.Channel;
import dev.frozenmilk.dairy.mercurial.processes.EventManager;
import dev.frozenmilk.dairy.mercurial.processes.SwitchedStateMachine;
import dev.frozenmilk.dairy.mercurial.processes.Timer;
import kotlin.time.TimeMark;
import static dev.frozenmilk.dairy.mercurial.continuations.Continuations.expression;

/** Drivetrain
 *  FieldCentric, robotCentric driving modes
 *  Robot Holding / Path Following modes
 *  Uses: 4 Motors
 */
public class Drivetrain extends SwitchedStateMachine {
    //check Mercuiral GitHub, most of the code the same, added timeouts for follow paths
    private final MercurialFollower follower;
    private final FusionMecanumDrivetrain innerDrivetrain;
    private Channel<MotionState> motionStates = linkedSingleChannel();//handle localization input
    private Channel<DrivePowers> drivePowers = linkedSingleChannel();//handle drive inputs
    //linked single channel so that if the state machine stops, the channel closes automatically
    //attached to the state machine fiber

    private Telemetry.Item stateTelemetry;
    Mode idle;
    Mode fieldCentric;
    Mode robotCentric;
    public Drivetrain(Context ctx){
        this(ctx, new FusionMecanumDrivetrain(ctx.hardwareMap(), Constants.driveConfig));
    }

    public Drivetrain(Context ctx, FusionMecanumDrivetrain drivetrain){
        if (drivetrain == null) throw new IllegalArgumentException("drivetrain is null");
        innerDrivetrain = drivetrain;
        follower = Constants.createFollower(drivetrain);
        stateTelemetry = ctx.telemetry().addData("Drivetrain/State", "unknown");//can't use normal telemetry
        //enter idle mode when start
        idle = entranceMode(from -> {
            follower.idle();
            stateTelemetry.setValue("idle");
        });

        fieldCentric = mode(self -> {
            DrivePowers powers = drivePowers.poll();
            MotionState motionState = motionStates.query();//query does not remove most recent motion state
            if(powers != null && motionState != null){
                DrivePowers fieldCentricPowers = ManualDrive.fieldCentric(
                        powers,
                        motionState.pose().heading()
                );
                follower.manual(fieldCentricPowers);//tell the follower to drive w/ field centric powers
            }
            stateTelemetry.setValue("Field Centric");
            return self;
        });

        robotCentric = mode(self -> {
            DrivePowers powers = drivePowers.poll();
            if(powers != null){
                follower.manual(powers);//tell the follower to drive w/ field centric powers
            }
            stateTelemetry.setValue("Robot Centric");
            return self;
        });

    }

    public FusionMecanumDrivetrain getInnerDrivetrain() {
        return innerDrivetrain;
    }

    private Mode holdPoseMode(TimeMark timeMark, Pose pose){
        return mode(self -> {
            stateTelemetry.setValue("holding pose");
            //pool from motionStates so that we only update when new motion state arrives
            MotionState motionState = motionStates.poll();
            if(motionState != null){
                TimeMark nextTimeMark = follower.hold(
                        pose,
                        true,
                        motionState,
                        timeMark
                );
                return holdPoseMode(nextTimeMark, pose);
                //keep holding next time mark
            }
            return self;
        });
    }
    private Mode holdPoseMode(Pose pose){
        return holdPoseMode(null, pose);
        //starting time mark
    }

    private Mode followPathMode(
            TimeMark timeMark,
            boolean holdEnd,
            PathTracker tracker,
            Channel<FollowResult> resultChannel,
            Timer<FollowResult> timeoutTimer,
            Channel<FollowResult> timeoutChannel
    ) {
        return egressMode(
                self -> {
                    stateTelemetry.setValue("following path");
                    // we poll the motion state,
                    // so that we only update when a new motion state arrives
                    MotionState motionState = motionStates.poll();
                    //check if path following has timedout
                    if(timeoutTimer != null && timeoutChannel.poll() == FollowResult.Timeout){
                        if(resultChannel != null){
                            resultChannel.sendIfEmpty(FollowResult.Timeout);
                        }
                        if(holdEnd) return holdPoseMode(tracker.endPose());
                        else return idle;
                    }


                    //follow next time mark
                    if (motionState != null) {
                        TimeMark nextTimeMark = follower.follow(
                                tracker,
                                motionState,
                                timeMark
                        );
                        // follower.follow will return null when its finished
                        if (nextTimeMark == null) {
                            // tell the awaiting process to move on,
                            if (resultChannel != null) resultChannel.sendIfEmpty(FollowResult.Done);
                            // if hold end, move to holding the end
                            if (holdEnd) return holdPoseMode(tracker.endPose());
                                // or idle
                            else return idle;
                        }
                        // if we're not finished, move to the next state
                        else return followPathMode(
                                nextTimeMark,
                                holdEnd,
                                tracker,
                                resultChannel,
                                timeoutTimer,
                                timeoutChannel
                        );
                    }
                    return self;
                },
                toMode -> {
                    // this function gets run when we exit following due to a switch
                    if(timeoutTimer != null) timeoutTimer.cancel();
                    //only sends to result channel that it "cancelled" if its empty(hasn't been timedout)
                    if (resultChannel != null) resultChannel.sendIfEmpty(FollowResult.Cancelled);
                    // make sure to release the tracker
                    tracker.release();
                }
        );
    }

    private Mode followPathMode(boolean holdEnd, Path path, Channel<FollowResult> resultChannel){//without a timeoutseconds
        return followPathMode(null, holdEnd, new PathTracker(path), resultChannel, null, null);
    }



    private Mode followPathMode(boolean holdEnd, Path path, Channel<FollowResult> resultChannel, double timeoutSeconds) {
        Channel<FollowResult> timeoutChannel = null;
        Timer<FollowResult> timeoutTimer = null;
        //invalid timeoutSeconds will be treated as no timeout, instead of throwing an exception
        if(!Double.isNaN(timeoutSeconds) && timeoutSeconds > 0 && !Double.isInfinite(timeoutSeconds)){
            timeoutChannel = Channel.single();

            timeoutTimer = Timer.sendAfter(
                    timeoutSeconds,
                    timeoutChannel,
                    FollowResult.Timeout
            );
            return followPathMode(null, holdEnd, new PathTracker(path), resultChannel, timeoutTimer, timeoutChannel);
        }
        //treat invalid inputs of timeoutSeconds as no timeout
        return followPathMode(holdEnd, path, resultChannel);
    }



    public Continuation.Builder<FollowResult> followPathSync(Path path, boolean holdEnd) {
        return expression((scope, self) -> {
            // local channel variable
            Reference.O<Channel<FollowResult>> channel = scope.o(Channel::single);
            // run async
            scope.runExec(() -> followPathAsync(holdEnd, path, channel.get()));
            // wait for a response
            return Continuations.<FollowResult>receive().receive(channel);
        });
    }


    @NonNull
    @Override
    protected Mode init() {
        return idle;
    }

    public void sendDrivePowers(DrivePowers powers) {
        drivePowers.send(powers);
    }

    public void sendDrivePowers(double forward, double strafe, double turn) {
        drivePowers.send(new DrivePowers(forward, strafe, turn));
    }

    public void sendMotionState(MotionState motionState) {
        motionStates.send(motionState);
    }

    /** Receives the commands actually applied by Pedro after drivetrain scaling. */
    public void addHandler(EventManager.Handler<ControlInput> handler) {
        innerDrivetrain.addHandler(handler);
    }

    public void fieldCentric() {
        switchTo(fieldCentric);
    }

    public void idle() {
        switchTo(idle);
    }
    public void robotCentric(){
        switchTo(robotCentric);
    }

    private void holdPose(Pose pose){
        switchTo(holdPoseMode(pose));
    }

    public void followPathAsync(boolean holdEnd, Path path, Channel<FollowResult> resultChannel) {
        switchTo(followPathMode(holdEnd, path, resultChannel));
    }
    public void followPathAsync(boolean holdEnd, Path path, Channel<FollowResult> resultChannel, double timeoutSeconds){
        switchTo(followPathMode(holdEnd, path, resultChannel, timeoutSeconds));
    }
    public void followPathAsync(Path path, boolean holdEnd) {
        followPathAsync(holdEnd, path, null);
    }

    public enum FollowResult {
        Done,
        Cancelled,
        Timeout,
    }


}
