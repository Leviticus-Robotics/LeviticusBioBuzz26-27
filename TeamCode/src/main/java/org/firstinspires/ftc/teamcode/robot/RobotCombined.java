package org.firstinspires.ftc.teamcode.robot;

import static dev.frozenmilk.dairy.mercurial.continuations.Continuations.exec;
import static dev.frozenmilk.dairy.mercurial.continuations.Continuations.ifThen;
import static dev.frozenmilk.dairy.mercurial.continuations.Continuations.loop;
import static dev.frozenmilk.dairy.mercurial.continuations.Continuations.noop;
import static dev.frozenmilk.dairy.mercurial.continuations.Continuations.race;
import static dev.frozenmilk.dairy.mercurial.continuations.Continuations.seconds;
import static dev.frozenmilk.dairy.mercurial.continuations.Continuations.sequence;
import static dev.frozenmilk.dairy.mercurial.continuations.Continuations.waitFor;
import static dev.frozenmilk.dairy.mercurial.continuations.Continuations.waitUntil;

import com.acmerobotics.dashboard.config.Config;

import org.firstinspires.ftc.robotcore.external.Telemetry;
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
import org.firstinspires.ftc.teamcode.helper.HiveSideHelper;
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
    public static double POST_SHOOT_DELAY = 0.5;
    public static double SHOOTER_SPINUP_TIME = 0.5;
    public static double TURRET_TOLERANCE = Math.toRadians(1.5);//can be up to 1.5 deg off when starting
    private boolean driverRequestedEarlyShot;//shoots balls early
    private boolean driverBallDetect;//driver says we already have balls
    private enum ActiveBehavior{
        NONE,
        ASSISTED_SHOOT,
        AUTOMATIC,
        DIRECT_MANUAL
    }
    private Fiber<?> activeFiber;
    private ActiveBehavior activeBehavior = ActiveBehavior.NONE;
    BallCounter ballCounter;



    public RobotCombined(Context ctx, HiveTargetSide startSide){
        sideMachine = new HiveSideHelper(startSide);
        poseTelemetry = ctx.telemetry().addData("Localizer Pose: ", "unknown");
        localizer = new MercurialLocalizer(Constants.createLocalizer(ctx.hardwareMap()));

        turretEncoder = new TurretEncoder(ctx);
        voltageMonitor = new VoltageMonitor(ctx);

        intake = new Intake(ctx);
        transfer = new Transfer(ctx);

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


        //send transfer xbeam breakers to transfer
        transferSensors = new TransferBeamBreak(ctx);
        transferSensors.addHandler(event -> {
            if(event.sensorNum == 1){
                transfer.updateBreaker1(event.interrupted);
            } else{
                transfer.updateBreaker2(event.interrupted);
            }
            return EventManager.EventHandled.ok();
        });


        //send transfer beam breakers to intake
        //need this for some of the more complicated logic
        transferSensors.addHandler(event -> {
            if(event.sensorNum == 1){
                intake.updateBreaker1(event.interrupted);
            } else{
                intake.updateBreaker2(event.interrupted);
            }
            return EventManager.EventHandled.ok();
        });


        //send localizer pose to telemetry
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


        //pass in the voltage to intake state machine
        voltageMonitor.addHandler(EventManager.forwardTo(
                intake,
                intake::updateVoltage
        ));

        //pass in the voltage to transfer state machine
        voltageMonitor.addHandler(EventManager.forwardTo(
                transfer,
                transfer::updateVoltage
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
        turretEncoder.addHandler(EventManager.forwardTo(
                turret,
                turret::updateCurrentTurretAngle
        ));
        //link the shooter turret angle to the turret receiving channel
        shooter.setHeadingChannel(turret.getTargetHeadingsChannel());

        transferGate = new TransferGate(ctx);
        ballCounter = new BallCounter(intake, transfer);
    }



    //a driver trigger will escape these
    private Continuation.Builder<?> waitForXBallsOrTimeout(int minBall, double timeoutSec){
        return race(
                waitUntil(() -> ballCounter.totalBalls() >= minBall || driverBallDetect),
                //drive requesting early shot is equivalent to having enough balls
                waitFor.bind(seconds(timeoutSec))
        );
    }

    private Continuation.Builder<?> waitForXBallsOrDriver(int minBall){
        return waitUntil(() -> ballCounter.totalBalls() >= minBall || driverBallDetect);
    }

    //move this into seperate class



    private void shoot(){
        transferGate.setOpen();
        intake.fixedPower(1.0);
        transfer.fixedPower(1.0);
    }
    //move this into seperate class




    //assumes we already have balls
    //doesn't check any conditions for criteria for shooting
    private Continuation.Builder<?> assistedShootSequence(ALLIANCE alliance, HiveTargetSide side){
        return sequence(
                //start the shooter in default shooting mode, but still calculate w/ lutSOTM
                exec(() -> {
                    shooter.setUseDefaultVelocity(true);
                    shooter.lutSOTM(alliance, side);
                }),
                //dont do anything if no balls detected
                ifThen(
                        () -> ballCounter.totalBalls() > 0,
                        sequence(
                                exec(()-> shooter.setUseDefaultVelocity(false)),
                                waitFor.bind(seconds(SHOOTER_SPINUP_TIME)),
                                exec(this::shoot),//shoot & lower transfer gate

                                //wait until no balls left
                                waitUntil(ballCounter::noBalls),
                                //buffer time
                                waitFor.bind(seconds(POST_SHOOT_DELAY))
                        ),
                        noop
                ),
                //reset
                exec(this::resetProcesses)
        );
    }

    //does full cycle autonomously
    //assumes transfer gate already closed
    //doesnt autodrive though
    private Continuation.Builder<?> automaticCycle(ALLIANCE alliance, HiveTargetSide side, double waitBallsTimeout){
        return sequence(
            exec(() -> {
                transferGate.setClosed();
                intake.runPowerTillDetectionMode(1.0);
                transfer.runPowerTillDetectionMode(1.0);
                shooter.setUseDefaultVelocity(true);
                shooter.lutSOTM(alliance, side);
            }),
            waitForXBallsOrDriver(1),
            exec(()-> shooter.setUseDefaultVelocity(false)),
            //wait until we have 4 balls & we are ready to shoot
            waitForXBallsOrTimeout(4, waitBallsTimeout),
            waitUntil(() -> isReadyToShoot(side)),
            exec(this::shoot),
            //wait until no balls left
            waitUntil(ballCounter::noBalls),
            //buffer time
            waitFor.bind(seconds(POST_SHOOT_DELAY)),
            //reset
            exec(this::resetProcesses)

        );
    }

    private Continuation.Builder<?> automaticMode(ALLIANCE alliance, HiveTargetSide side, double waitBallsTimeout, boolean repeat){
        Continuation.Builder<?> cycle = automaticCycle(alliance, side, waitBallsTimeout);
        return repeat ? loop(cycle) : cycle;
    }




    private boolean isReadyToShoot(HiveTargetSide side){
        //maybe add the camera tilt angle check here
        return (shooter.isValidShootPosition(side)
                && shooter.isAtShootingTargetVelocity(Shooter.DEFAULT_TOLERANCE)
                && turret.isAtAngle(TURRET_TOLERANCE)) ||
                driverRequestedEarlyShot;
    }


    public void requestEarlyShot(){
        driverRequestedEarlyShot = true;
    }

    public void requestBallDetect(){
        driverBallDetect = true;
    }

    public boolean isAutomaticRunning(){
        return activeBehavior == ActiveBehavior.AUTOMATIC
                && activeFiber != null
                && activeFiber.status().alive();
    }
    public boolean isAssistedRunning(){
        return activeBehavior == ActiveBehavior.ASSISTED_SHOOT
                && activeFiber != null
                && activeFiber.status().alive();
    }

    //whether any shooting sequence running
    public boolean isShootingBehaviorRunning(){
        return activeFiber != null && activeFiber.status().alive();
    }


    private void cancelActiveProcesses(){
        if(activeFiber != null && activeFiber.status().alive()){
            activeFiber.unlink();
            activeFiber.exit(ExitReason.interrupt);
        }
        activeFiber = null;
        activeBehavior = ActiveBehavior.NONE;
        driverRequestedEarlyShot = false;
        driverBallDetect = false;
    }

    private void resetProcesses(){
        intake.off();
        transfer.off();
        transferGate.setClosed();
        shooter.setUseDefaultVelocity(true);
        driverRequestedEarlyShot = false;
        driverBallDetect = false;
    }

    public void startAssistedShoot(ALLIANCE alliance, HiveTargetSide side){
        cancelActiveProcesses();//make sure nothing running before
        resetProcesses();
        activeFiber = assistedShootSequence(alliance, side)
                .spawnable()
                .spawnLink();
        activeBehavior = ActiveBehavior.ASSISTED_SHOOT;
    }

    public void startAutomatic(ALLIANCE alliance, HiveTargetSide side, double timeout, boolean repeat){
        cancelActiveProcesses();
        resetProcesses();
        activeFiber = automaticMode(alliance, side, timeout, repeat)
                .spawnable()
                .spawnLink();
        activeBehavior = ActiveBehavior.AUTOMATIC;
    }


    public void requestManaulShoot(ALLIANCE alliance,
                                   HiveTargetSide side,
                                   boolean driverHasManuallyFed,
                                   double intakePower,
                                   double transferPower
    ){
        //two possibilities -> if driver has started intaking/transfer, then just give ful manual control
        //-> otherwise by default start the assisted shot
        //cancel active processes if in automatic mode
        if(activeBehavior == ActiveBehavior.AUTOMATIC) {
            cancelActiveProcesses();
        }
        if(driverHasManuallyFed){
            beginDirectManualFeed(intakePower, transferPower);
            transferGate.setOpen();
        } else{
            startAssistedShoot(alliance, side);
        }
    }

    public void beginDirectManualFeed(double intakePower, double transferPower){
        if (activeBehavior != ActiveBehavior.DIRECT_MANUAL){
            cancelActiveProcesses();
            activeBehavior = ActiveBehavior.DIRECT_MANUAL;
        }

        setDirectManualPowers(intakePower, transferPower);
    }

    public void updateDirectManualFeed(double intakePower, double transferPower){
        if (activeBehavior == ActiveBehavior.DIRECT_MANUAL){
            setDirectManualPowers(intakePower, transferPower);
        }
    }

    public void endDirectManualFeed(){
        if (activeBehavior == ActiveBehavior.DIRECT_MANUAL) {
            intake.off();
            transfer.off();
            activeBehavior = ActiveBehavior.NONE;
        }
    }

    private void setDirectManualPowers(double intakePower, double transferPower) {
        intake.fixedPower(intakePower);
        transfer.fixedPower(transferPower);
    }

    //next thing to model is more complex shooting logic, manual and automatic
}
