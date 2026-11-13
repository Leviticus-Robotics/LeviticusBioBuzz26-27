package org.firstinspires.ftc.teamcode.robot;

import static dev.frozenmilk.dairy.mercurial.continuations.Continuations.expression;

import com.acmerobotics.dashboard.config.Config;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.constants.Constants;
import org.firstinspires.ftc.teamcode.game.ALLIANCE;
import org.firstinspires.ftc.teamcode.game.HiveTargetSide;
import org.firstinspires.ftc.teamcode.helper.HiveSideHelper;
import org.firstinspires.ftc.teamcode.sensor.TurretEncoder;
import org.firstinspires.ftc.teamcode.sensor.VoltageMonitor;
import org.firstinspires.ftc.teamcode.subsystems.Drivetrain;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.Shooter;
import org.firstinspires.ftc.teamcode.subsystems.Transfer;
import org.firstinspires.ftc.teamcode.subsystems.TransferGate;
import org.firstinspires.ftc.teamcode.subsystems.Turret;
import org.firstinspires.ftc.teamcode.util.Angle;

import dev.frozenmilk.dairy.mercurial.ftc.Context;
import dev.frozenmilk.dairy.mercurial.pedro.MercurialLocalizer;
import dev.frozenmilk.dairy.mercurial.processes.EventManager;
import dev.frozenmilk.dairy.mercurial.processes.Fiber;

//this robot class is for tuning the shooter in TeleOp
//shooter P, kS, kV constants, shooter LUT, SOTM
//uses simple intake transfer shoot(no ball detectors being used)

@Config
public class RobotShooterTuning {
    Shooter shooter;
    Turret turret;
    Intake intake;
    Transfer transfer;
    TransferGate transferGate;
    Telemetry.Item poseTelemetry;
    Telemetry.Item velocityTelemetry;
    Telemetry.Item turretPositionTelemetry;
    Telemetry.Item turretAngleTelemetry;

    HiveSideHelper sideMachine;
    MercurialLocalizer localizer;
    Drivetrain drivetrain;
    TurretEncoder turretEncoder;
    VoltageMonitor voltageMonitor;
    public static double targetVelocity = 1500;//ticks per second
    public static double TURRET_TEST_ANGLE = 0;//degrees
    public static double TURRET_TEST_POSITION = 0.5;
    ALLIANCE alliance;
    public RobotShooterTuning(Context ctx, ALLIANCE alliance, HiveTargetSide startSide){
        sideMachine = new HiveSideHelper(startSide);
        poseTelemetry = ctx.telemetry().addData("Localizer Pose: ", "unknown");
        velocityTelemetry = ctx.telemetry().addData("Shooter Velocity (ticks/sec): ", 0);
        turretPositionTelemetry = ctx.telemetry().addData("Turret Position: ", 0);
        turretAngleTelemetry = ctx.telemetry().addData("Turret Enocder Angle", 0);
        localizer = new MercurialLocalizer(Constants.createLocalizer(ctx.hardwareMap()));

        turretEncoder = new TurretEncoder(ctx);
        voltageMonitor = new VoltageMonitor(ctx);

        intake = new Intake(ctx);
        transfer = new Transfer(ctx);

        shooter = new Shooter(ctx);
        turret = new Turret(ctx, true);
        transferGate = new TransferGate(ctx);
        drivetrain = new Drivetrain(ctx);


        //send localizer pose to telemetry
        localizer.addHandler(event -> {
            poseTelemetry.setValue(localizer.pose());
            return EventManager.EventHandled.ok();
        });


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


        //send localizer pose to shooter state machine
        voltageMonitor.addHandler(EventManager.forwardTo(
                shooter,
                shooter::updateVoltage
        ));

        localizer.addHandler(EventManager.forwardTo(
                shooter,
                shooter::sendMotionState
        ));

        //send the turret encoder over to the turret
        turretEncoder.addHandler(EventManager.forwardTo(
                turret,
                turret::updateCurrentTurretAngle
        ));
        //link the shooter turret angle to the turret receiving channel
        shooter.setHeadingChannel(turret.getTargetHeadingsChannel());

        transferGate = new TransferGate(ctx);
        //resetting encoder has to be in teleop main class
    }

    public void updateTelemetry() {
        velocityTelemetry.setValue(shooter.getVelocity());
        poseTelemetry.setValue(localizer.pose());
        turretPositionTelemetry.setValue(turret.servoGroup.getRawPosition());
        turretAngleTelemetry.setValue(turret.currentTurretAngle());
    }


    public void shoot(){
        transferGate.setOpen();
        intake.fixedPower(1.0);
        transfer.fixedPower(1.0);
    }
    public void resetGate(){
        transferGate.setClosed();
    }
    public void intake(){
        intake.fixedPower(1.0);
        transfer.fixedPower(1.0);
    }
    double power = 0;
    double interval = 0.1;
    //use Desmos to graph the chart between x: power & y: velocity * (current voltage/ target voltage)
    //y-intercept is the kS term, m is the kV term
    public void tuneKsKvNextInterval(){
        power += interval;
        shooter.fixedPower(power);
        velocityTelemetry.setValue(shooter.getVelocity());
    }

    //run target velocity, tune P and maybe D
    public void tunePID(){
        shooter.setUseDefaultVelocity(false);
        shooter.fixedVelocity(targetVelocity, false);
    }

    //plug in different target velocities and obeserve the distance from goal
    //pick velocities that work at each point, and add to the LUT
    public void tuneDistVelLUT(double targetVelocity){
        shooter.setUseDefaultVelocity(false);
        shooter.fixedVelocity(targetVelocity, false);
    }
    public void tuneDistHoodLUT(double hoodPos){
        shooter.lutStatic(alliance, sideMachine.getSide());
    }

    public static double hoodTestPosition = 0.5;

    //set hood to different position to get for hood LUT
    //observe whether balls go in and the distance from goal
    public void tuneHood() {
        shooter.setHood(hoodTestPosition);
    }


    public void testRawTurretAngle(){
        Angle target = Angle.fromDegrees(TURRET_TEST_ANGLE);
        turret.setRawAngle(target);
    }
    public void testOptimizedTurretAngle(){
        Angle target = Angle.fromDegrees(TURRET_TEST_ANGLE);
        turret.setOptimizedAngle(target);
    }
    //set turret from 0 to 1 and find total encoder range
    public void setTurretSetPosition(){
        turret.setRawPosition(TURRET_TEST_POSITION);
    }
}

