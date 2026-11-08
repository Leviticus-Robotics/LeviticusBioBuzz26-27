package org.firstinspires.ftc.teamcode.robot;

import static dev.frozenmilk.dairy.mercurial.continuations.Continuations.command;
import static dev.frozenmilk.dairy.mercurial.continuations.Continuations.noop;

import androidx.annotation.NonNull;

import com.seattlesolvers.solverslib.command.SequentialCommandGroup;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.BallSlot;
import org.firstinspires.ftc.teamcode.constants.Constants;
import org.firstinspires.ftc.teamcode.game.ALLIANCE;
import org.firstinspires.ftc.teamcode.game.SIDE;
import org.firstinspires.ftc.teamcode.sensor.IntakeBallSensors;
import org.firstinspires.ftc.teamcode.sensor.TransferBeamBreak;
import org.firstinspires.ftc.teamcode.sensor.TurretEncoder;
import org.firstinspires.ftc.teamcode.sensor.VoltageMonitor;
import org.firstinspires.ftc.teamcode.subsystems.Drivetrain;
import org.firstinspires.ftc.teamcode.subsystems.FlowerWedge;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.Shooter;
import org.firstinspires.ftc.teamcode.subsystems.Transfer;
import org.firstinspires.ftc.teamcode.subsystems.TransferGate;
import org.firstinspires.ftc.teamcode.subsystems.Turret;

import dev.frozenmilk.dairy.mercurial.continuations.Command;
import dev.frozenmilk.dairy.mercurial.continuations.Continuation;
import dev.frozenmilk.dairy.mercurial.ftc.Context;
import dev.frozenmilk.dairy.mercurial.pedro.MercurialLocalizer;
import dev.frozenmilk.dairy.mercurial.processes.EventManager;
import dev.frozenmilk.dairy.mercurial.processes.SwitchedStateMachine;

/** Robot class combining multiple subsystems
 *
 */
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

    public enum ShootingMode{
        Manual, //need to manually trigger the transfer gate to acitvate/inactivate
        Automatic//will automatically trigger transfer gate accodingly, though its dependent on sensors
    }
    private enum AutomaticState{
        INTAKING_BALLS,
        WAITING_FOR_SHOOTER_READY,
        SHOOTING,
        RESETTING
    }
    //for now assume that we are intkaing balls right now(so it triggers the sensor logic)
    AutomaticState automaticState = AutomaticState.INTAKING_BALLS;
    public RobotCombined(Context ctx){
        poseTelemetry = ctx.telemetry().addData("Localizer Pose: ", "unknown");
        localizer = new MercurialLocalizer(Constants.createLocalizer(ctx.hardwareMap()));

        turretEncoder = new TurretEncoder(ctx);
        voltageMonitor = new VoltageMonitor(ctx);

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


        //send transfer beam breakers to transfer
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

        intake = new Intake(ctx);
        //pass in the voltage to intake state machine
        voltageMonitor.addHandler(EventManager.forwardTo(
                intake,
                intake::updateVoltage
        ));

        transfer = new Transfer(ctx);
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

    }


    boolean firstShooting;
    double startTime = 0;
    public static double WAIT_BEFORE_RESETTING = 1000;//in ms

    //want to turn this into command version

//    public Command update(ALLIANCE alliance, SIDE side, double velTolerance){
//        return new SequentialCommandGroup()
//    }
    public void updatingShootingLogic(ALLIANCE alliance, SIDE side, double velTolerance){
        switch (automaticState){
            case INTAKING_BALLS:
                transfer.runPowerTillDetection(1);
                intake.runPowerTillDetection(1);
                shooter.lutSOTM(alliance, side, velTolerance);
                if(intake.numBallsDetected() + transfer.numBallsDetected() == 4){
                    automaticState = AutomaticState.WAITING_FOR_SHOOTER_READY;
                }
                break;
            case WAITING_FOR_SHOOTER_READY:
                //keep shooter in the same mode
                //at this point both intake and transfer should be turned off b/c detect all 4
                if(shooter.isValidShootPosition(side) && shooter.isAtVelocity(velTolerance)){
                    automaticState = AutomaticState.SHOOTING;
                }
                break;
            case SHOOTING:
                transferGate.setInactive();
                if(!firstShooting && intake.numBallsDetected() + transfer.numBallsDetected() == 0){
                    firstShooting = true;
                    startTime = System.nanoTime();
                }
                if(System.nanoTime() - startTime > WAIT_BEFORE_RESETTING){
                    automaticState = AutomaticState.RESETTING;
                }
                break;
            case RESETTING:
                firstShooting = false;
                automaticState = AutomaticState.INTAKING_BALLS;
                break;
        }
    }

    //next thing to model is more complex shooting logic, manual and automatic
}