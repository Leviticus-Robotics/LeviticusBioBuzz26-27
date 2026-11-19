package org.firstinspires.ftc.teamcode.robot;

import static dev.frozenmilk.dairy.mercurial.continuations.Continuations.exec;
import static dev.frozenmilk.dairy.mercurial.continuations.Continuations.loop;
import static dev.frozenmilk.dairy.mercurial.continuations.Continuations.race;
import static dev.frozenmilk.dairy.mercurial.continuations.Continuations.seconds;
import static dev.frozenmilk.dairy.mercurial.continuations.Continuations.sequence;
import static dev.frozenmilk.dairy.mercurial.continuations.Continuations.waitFor;
import static dev.frozenmilk.dairy.mercurial.continuations.Continuations.waitUntil;

import com.acmerobotics.dashboard.config.Config;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.helper.AngleHelper;
import org.firstinspires.ftc.teamcode.helper.BallSlot;
import org.firstinspires.ftc.teamcode.constants.Constants;
import org.firstinspires.ftc.teamcode.game.ALLIANCE;
import org.firstinspires.ftc.teamcode.game.HiveTargetSide;
import org.firstinspires.ftc.teamcode.helper.HiveSideHelper;
import org.firstinspires.ftc.teamcode.sensor.IntakeBallSensors;
import org.firstinspires.ftc.teamcode.sensor.TransferBeamBreak;
import org.firstinspires.ftc.teamcode.sensor.TurretEncoder;
import org.firstinspires.ftc.teamcode.sensor.VoltageMonitor;
import org.firstinspires.ftc.teamcode.helper.BallCounter;
import org.firstinspires.ftc.teamcode.subsystems.Drivetrain;
import org.firstinspires.ftc.teamcode.subsystems.FlowerWedge;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.Shooter;
import org.firstinspires.ftc.teamcode.subsystems.Transfer;
import org.firstinspires.ftc.teamcode.subsystems.TransferGate;
import org.firstinspires.ftc.teamcode.subsystems.Turret;

import dev.frozenmilk.dairy.mercurial.continuations.Continuation;
import dev.frozenmilk.dairy.mercurial.ftc.Context;
import dev.frozenmilk.dairy.mercurial.pedro.MercurialLocalizer;
import dev.frozenmilk.dairy.mercurial.processes.EventManager;
import dev.frozenmilk.dairy.mercurial.processes.ExitReason;
import dev.frozenmilk.dairy.mercurial.processes.Fiber;

/** Robot class combining multiple subsystems
 *
 */

@Config
public class RobotCombined {
    MercurialLocalizer localizer;
    Drivetrain drivetrain;
    Intake intake;
    Transfer transfer;
    Shooter shooter;
    FlowerWedge flowerWedge;
    Turret turret;
    IntakeBallSensors intakeSensors;
    TransferBeamBreak transferSensors;
    VoltageMonitor voltageMonitor;
    TurretEncoder turretEncoder;
    TransferGate transferGate;
    Telemetry.Item poseTelemetry;
    HiveSideHelper sideMachine;
    public static double POST_SHOT_DELAY = 0.5;
    public static double TURRET_TOLERANCE = Math.toRadians(1.5);//can be up to 1.5 deg off when starting
    private boolean earlyShotRequested;//shoots balls early
    private boolean driverBallDetect;//driver says we already have balls
    private enum ActiveBehavior{
        NONE,
        DIRECT_MANUAL,
        ASSISTED_WAITING,
        ASSISTED_SHOOTING,
        AUTOMATIC_COLLECTING,
        AUTOMATIC_WAITING,
        AUTOMATIC_SHOOTING
    }
    private Fiber<?> activeFiber;
    private Fiber<?> ballSensorUpdateFiber;
    private ActiveBehavior activeBehavior = ActiveBehavior.NONE;
    BallCounter ballCounter;
    public static double GATE_EMPTY_DELAY = 0.8; //seconds
    private Fiber<?> gateCloseTimer;
    private boolean gateOpenCommanded = false;
    private boolean ballsSeenSinceGateOpened = false;

    private boolean ballSensorStateChanged;


    public RobotCombined(Context ctx, HiveTargetSide startSide){
        sideMachine = new HiveSideHelper(startSide);
        poseTelemetry = ctx.telemetry().addData("Localizer Pose: ", "unknown");
        localizer = new MercurialLocalizer(Constants.createLocalizer(ctx.hardwareMap()));

        turretEncoder = new TurretEncoder(ctx);
        voltageMonitor = new VoltageMonitor(ctx);

        intake = new Intake(ctx);
        transfer = new Transfer(ctx);

        transferGate = new TransferGate(ctx);
        ballCounter = new BallCounter(intake, transfer);

        //send intake sensors over to the intake
        intakeSensors = new IntakeBallSensors(ctx);
        intakeSensors.addHandler(event -> {
            if(event.getBallSlot()== BallSlot.Slot.LEFT){
                intake.updateIntakeSpot1(event.getBallDetected());
            } else{
                intake.updateIntakeSpot2(event.getBallDetected());
            }
            return EventManager.EventHandled.ok();
        });


        //send transfer beam breakers to transfer & intake
        transferSensors = new TransferBeamBreak(ctx);
        transferSensors.addHandler(event -> {
            if(event.sensorNum == 1){
                transfer.updateBreaker1(event.interrupted);
                intake.updateBreaker1(event.interrupted);
            } else{
                transfer.updateBreaker2(event.interrupted);
                intake.updateBreaker2(event.interrupted);
            }
            return EventManager.EventHandled.ok();
        });



        //send localizer pose to telemetry
        //update other telemetry items here
        localizer.addHandler(event -> {
            poseTelemetry.setValue(localizer.pose());
            return EventManager.EventHandled.ok();
        });


        drivetrain = new Drivetrain(ctx);
        //send localizer pose to drivetrain
        localizer.addHandler(EventManager.forwardTo(
                drivetrain,
                drivetrain::sendMotionState
        ));


        shooter = new Shooter(ctx);
        //send localizer pose to shooter state machine
        voltageMonitor.addHandler(EventManager.forwardTo(
                shooter,
                shooter::updateVoltage
        ));

        localizer.addHandler(EventManager.forwardTo(
                shooter,
                shooter::sendMotionState
        ));

        flowerWedge = new FlowerWedge(ctx);


        turret = new Turret(ctx, true);
        //send the turret encoder over to the turret
        turretEncoder.addHandler(event -> {
            if(event.getAngleType() == AngleHelper.AngleType.Absolute){
                turret.updateAbsoluteAngle(event.getAngle());
            }
            else if(event.getAngleType() == AngleHelper.AngleType.Incremental){
                turret.updateIncrementalAngle(event.getAngle());
            }
            return EventManager.EventHandled.ok();
        });
        //link the shooter turret angle to the turret receiving channel
        shooter.setHeadingChannel(turret.getTargetHeadingsChannel());

        //run this ball update fiber constantly
        ballSensorUpdateFiber =
            loop(
                    exec(() -> onBallSensorsUpdate())
            ).spawnable().spawnLink();

    }



    //waits for X balls OR timeout OR driver skips ball detection OR driver sends early shot
    private Continuation.Builder<?> waitForXBallsOrTimeout(int minBall, double timeoutSec){
        //temporarily removed race condition b/c doesnt work for now
//        return race(
                return waitUntil(() -> ballCounter.totalBalls() >= minBall
                        || driverBallDetect
                        || earlyShotRequested
                );
                //drive requesting early shot is equivalent to having enough balls
//                waitFor.bind(seconds(timeoutSec))

    }


    private void shoot(){
        openTransferGate();
        intake.fixedPower(1.0);
        transfer.fixedPower(1.0);
    }



    //assumes we already have balls
    //doesn't check any conditions for criteria for shooting
    private Continuation.Builder<?> assistedShootSequence(ALLIANCE alliance, HiveTargetSide side){
        return sequence(
                //start the shooter in default shooting mode, but still calculate w/ lutSOTM
                exec(() -> {
                    activeBehavior = ActiveBehavior.ASSISTED_WAITING;
                    shooter.lutSOTM(alliance, side);
                    shooter.setUseDefaultVelocity(true);
                }),
                //going to assume we have balls to shoot
                waitUntil(() -> isReadyToShoot(side)),
                exec(() -> activeBehavior = ActiveBehavior.ASSISTED_SHOOTING),
                exec(this::shoot),//shoot & lower transfer gate

                //wait until no balls left
                waitUntil(ballCounter::noBalls),
                //buffer time
                waitFor.bind(seconds(POST_SHOT_DELAY)),

                //reset
                exec(this::finishShootingSequence),
                exec(()-> activeBehavior = ActiveBehavior.NONE)
        );
    }

    //does full cycle autonomously
    //assumes transfer gate already closed
    //doesnt autodrive though
    private Continuation.Builder<?> automaticCycle(ALLIANCE alliance, HiveTargetSide side, double waitBallsTimeout){
        return sequence(
            exec(() -> {
                activeBehavior = ActiveBehavior.AUTOMATIC_COLLECTING;
                closeTransferGate();
                intake.runPowerTillDetectionMode(1.0);
                transfer.runPowerTillDetection(1.0);

                shooter.lutSOTM(alliance, side);
                shooter.setUseDefaultVelocity(transfer.numBallsDetected() == 0);
            }),
            waitForXBallsOrTimeout(4, waitBallsTimeout),
            exec(() -> activeBehavior = ActiveBehavior.AUTOMATIC_WAITING),
            waitUntil(() -> isReadyToShoot(side)),
            exec(()-> {activeBehavior = ActiveBehavior.AUTOMATIC_SHOOTING;
                shoot();
            }),
            //wait until no balls left
            waitUntil(ballCounter::noBalls),
            //buffer time
            waitFor.bind(seconds(POST_SHOT_DELAY)),
            //reset
            exec(this::finishShootingSequence)

        );
    }

    private Continuation.Builder<?> automaticMode(ALLIANCE alliance, HiveTargetSide side, double waitBallsTimeout, boolean repeat){
        Continuation.Builder<?> cycle = automaticCycle(alliance, side, waitBallsTimeout);
        return repeat ? loop(cycle)
                : sequence(cycle, exec(() -> activeBehavior = ActiveBehavior.NONE));
    }




    private boolean isReadyToShoot(HiveTargetSide side){
        //maybe add the camera tilt angle check here
        return (shooter.isValidShootPosition(side)
                && shooter.isAtShootingTargetVelocity(Shooter.DEFAULT_TOLERANCE)
                && turret.isAtAngle(TURRET_TOLERANCE)) ||
                earlyShotRequested;
    }


    public void requestEarlyShot(){
        earlyShotRequested = true;
    }

    public void confirmBallsLoaded(){
        driverBallDetect = true;
    }

    public boolean isAutomaticRunning(){
        return (activeBehavior == ActiveBehavior.AUTOMATIC_SHOOTING ||
                activeBehavior == ActiveBehavior.AUTOMATIC_WAITING ||
                activeBehavior == ActiveBehavior.AUTOMATIC_COLLECTING)
                && activeFiber != null
                && isFiberActive();
    }
    public boolean isAssistedRunning(){
        return (activeBehavior == ActiveBehavior.ASSISTED_WAITING ||
                activeBehavior == ActiveBehavior.ASSISTED_SHOOTING)
                && activeFiber != null
                && isFiberActive();
    }

    //whether any shooting sequence running
    public boolean isShootingBehaviorRunning(){
        return isAutomaticRunning() || isAssistedRunning();
    }




    private void cancelActiveProcesses(){
        if(activeFiber != null && activeFiber.status().alive()){
            activeFiber.unlink();
            activeFiber.exit(ExitReason.interrupt);
        }
        activeFiber = null;
        activeBehavior = ActiveBehavior.NONE;
        earlyShotRequested = false;
        driverBallDetect = false;
    }
    private void stopFeed(){
        intake.off();
        transfer.off();
        closeTransferGate();
    }

    private void prepareFeed(){
        stopFeed();
        earlyShotRequested = false;
        driverBallDetect = false;
    }

    private void finishShootingSequence(){
        stopFeed();
        shooter.setUseDefaultVelocity(true);
        earlyShotRequested = false;
        driverBallDetect = false;
    }



    public void startAssistedShoot(ALLIANCE alliance, HiveTargetSide side){
        cancelActiveProcesses();
        prepareFeed();
        activeBehavior = ActiveBehavior.ASSISTED_WAITING;
        activeFiber = assistedShootSequence(alliance, side)
                .spawnable()
                .spawnLink();
    }

    public void startAutomatic(ALLIANCE alliance, HiveTargetSide side, double timeout, boolean repeat){
        cancelActiveProcesses();
        prepareFeed();
        activeBehavior = ActiveBehavior.AUTOMATIC_COLLECTING;
        activeFiber = automaticMode(alliance, side, timeout, repeat)
                .spawnable()
                .spawnLink();
    }

    public void requestManualControl(double power){
        if(activeBehavior != ActiveBehavior.DIRECT_MANUAL){
            cancelActiveProcesses();
            closeTransferGate();//just for safety
            activeBehavior = ActiveBehavior.DIRECT_MANUAL;
        }

        if(gateOpenCommanded) {
            intake.fixedPower(power);
            transfer.fixedPower(power);
        } else{
            intake.runPowerTillDetection(power);
            transfer.runPowerTillDetection(power);
        }
    }

    public void endDirectManualFeed(){
        if (activeBehavior == ActiveBehavior.DIRECT_MANUAL) {
            intake.off();
            transfer.off();
            closeTransferGate();
            activeBehavior = ActiveBehavior.NONE;
            shooter.setUseDefaultVelocity(transfer.numBallsDetected() == 0);
        }
    }
    public void handleTransferGateTrigger(ALLIANCE alliance, HiveTargetSide side, boolean manualIntakeActive) {
        //Case 1: Driver manually operating intake/transfer and gate button pressed
        if (activeBehavior == ActiveBehavior.DIRECT_MANUAL){
            if (manualIntakeActive && !gateOpenCommanded) {
                openTransferGate();
            }
            return;
        }

        //Case 2: Start assisted shooting from idle
        if(activeBehavior == ActiveBehavior.NONE){
            startAssistedShoot(alliance, side);
            return;
        }

        //Case 3: Driver trying to bypass shooter readiness
        //Must attempt to open gate again when it is already opened
        if(activeBehavior == ActiveBehavior.ASSISTED_WAITING ||
            activeBehavior == ActiveBehavior.AUTOMATIC_COLLECTING ||
            activeBehavior == ActiveBehavior.AUTOMATIC_WAITING){
            requestEarlyShot();
        }
    }


    private boolean isShootingFeed(){
        return activeBehavior == ActiveBehavior.ASSISTED_SHOOTING ||
                activeBehavior == ActiveBehavior.AUTOMATIC_SHOOTING;
    }

    private void openTransferGate(){
        cancelGateCloseTimer();
        transferGate.setOpen();
        gateOpenCommanded = true;
        //sets balls seen immediantly after gate opens depending on if it seees any balls
        ballsSeenSinceGateOpened = !ballCounter.noBalls();
        if (activeBehavior == ActiveBehavior.DIRECT_MANUAL) {
            shooter.setUseDefaultVelocity(false);
        }
    }

    private void closeTransferGate(){
        cancelGateCloseTimer();
        transferGate.setClosed();
        gateOpenCommanded = false;
        ballsSeenSinceGateOpened = false;
    }

    private void cancelGateCloseTimer(){
        if(gateCloseTimer != null && gateCloseTimer.status().alive()){
            gateCloseTimer.unlink();
            gateCloseTimer.exit(ExitReason.interrupt);
        }
        gateCloseTimer = null;
    }

    //closes gate in manual mode after no more balls detected
    private void scheduleGateClose(){
        if(gateCloseTimer != null && gateCloseTimer.status().alive()){
            return;
        }
        gateCloseTimer = sequence(
                //wait in case leftover balls coming through
                waitFor.bind(seconds(GATE_EMPTY_DELAY)),
                exec(() -> {
                    if(gateOpenCommanded
                            && ballsSeenSinceGateOpened//must have seen balls after the gate opened for this to close
                            && activeBehavior == ActiveBehavior.DIRECT_MANUAL
                            && ballCounter.noBalls()){

                        transferGate.setClosed();
                        gateOpenCommanded = false;
                        ballsSeenSinceGateOpened = false;
                    }
                    //reset usedefault velocity for shooter after closing gate in manual shooting
                    if (activeBehavior == ActiveBehavior.DIRECT_MANUAL) {
                        shooter.setUseDefaultVelocity(
                                transfer.numBallsDetected() == 0
                        );
                    }

                    gateCloseTimer = null;
                })
        ).spawnable().spawnLink();
    }

    boolean manualFeeding;
    public void onBallSensorsUpdate(){
        //update default shooting velocity depending on whether there are balls
        //only do this in the manual mode
        boolean manualFeeding = activeBehavior == ActiveBehavior.DIRECT_MANUAL && gateOpenCommanded;
        if (!isShootingFeed()) {
            if (manualFeeding) {//if manually feeding, use regular shooting, it closes in scheduleGateClose()
                if (transfer.numBallsDetected() > 0) {
                    shooter.setUseDefaultVelocity(false);
                }
            } else {
                //normal outside manual feeding
                shooter.setUseDefaultVelocity(
                        transfer.numBallsDetected() == 0
                );
            }
        }
        //leave if gate is not open
        if(!gateOpenCommanded || isShootingFeed()){
            cancelGateCloseTimer();
            return;
        }
        //cancel the gate automatic closing timer if we see new balls
        if(!ballCounter.noBalls()){
            ballsSeenSinceGateOpened = true;
            cancelGateCloseTimer();
        } else if(ballsSeenSinceGateOpened){
            scheduleGateClose();
        }
        //only schedule gate close if there are no more balls detected
    }


    private boolean isFiberActive(){
        return activeFiber != null && activeFiber.status().alive();
    }


}
