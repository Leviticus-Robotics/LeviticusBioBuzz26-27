package org.firstinspires.ftc.teamcode.subsystems;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.config.Config;
import com.pedropathing.revhub.localizers.Encoder;
import com.qualcomm.hardware.digitalchickenlabs.OctoQuad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.seattlesolvers.solverslib.hardware.motors.Motor;
import com.seattlesolvers.solverslib.hardware.servos.ServoEx;
import com.seattlesolvers.solverslib.hardware.servos.ServoExGroup;
import com.seattlesolvers.solverslib.util.Direction;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.constants.ConfigNames;
import org.firstinspires.ftc.teamcode.hardware.MotorEx;
import org.firstinspires.ftc.teamcode.util.Angle;
import org.firstinspires.ftc.teamcode.util.Range;
import org.jetbrains.annotations.NotNull;

import java.util.ArrayList;
import java.util.List;

import dev.frozenmilk.dairy.mercurial.ftc.Context;
import dev.frozenmilk.dairy.mercurial.processes.Channel;
import dev.frozenmilk.dairy.mercurial.processes.StateMachine;
import dev.frozenmilk.dairy.mercurial.processes.SwitchedStateMachine;

/** Turret
 *
 *  Robot Holding / Path Following modes
 *  Uses: 2 Servos (1 Encoder -> which comes in as a sensor)
 *  1) Left & Right Servo when staring from front intake area looking straight at the turret
 *  2) Turret servo centered faces away from intake -> turret angle frame is 0 degrees, pos = ccw, neg = cw
 *  3) Encoder Absolute cannot be reset, so need to deal with this angle frame too
 */
@Config
public class Turret extends SwitchedStateMachine {
    ServoEx servoLeft;
    ServoEx servoRight;
    public static Range servoRange = new Range(0, 1);
    public static double SERVO_CENTER_POSITION = 0.5;//this is facing away from the intake(180)
    public static Angle ANALOG_ENCODER_CENTER_POS = Angle.fromRadians(Math.toRadians(237));//need to find this value
    public static Angle TURRET_CENTER_ANGLE = Angle.fromRadians(-Math.PI);

    public static Angle TURRET_HALF_TRAVEL = Angle.fromRadians(Math.toRadians(194));
    public static Range angleRange = new Range(
            TURRET_CENTER_ANGLE.sub(TURRET_HALF_TRAVEL).toRadians(),
            TURRET_CENTER_ANGLE.add(TURRET_HALF_TRAVEL).toRadians()
    );
    //this angle range is assuming the intake facing forward is zero degrees
    //center might not be 0 degrees


    private Channel<Angle> targetHeadingsChannel = linkedSingleChannel();//receives a channel of target heading inputs for turret for sotm/lut
    private Angle absoluteAngle = new Angle();
    private Angle rawAbsoluteAngle = new Angle();
    private Angle incrementalAngle = new Angle();
    private Angle rawIncrementalAngle = new Angle();
    private Angle targetTurretAngle; // current turret angle from turret frame
    public static boolean SERVO_LEFT_INVERTED = false;
    public static boolean SERVO_RIGHT_INVERTED = false;

    Mode turnOffServos;
    Mode noop;
    Mode startMode;
    Telemetry.Item stateTelemetry;
    Telemetry.Item absoluteAngleTelemetry;
    Telemetry.Item incrementalAngleTelemetry;
    public static double offsetServos = 0.02;

    //check if need to pass in start position from encoder to set to default position because can reset to absolute position
    //passing in true to reset zero moves turret back to center position
    public Turret(Context ctx, boolean resetCenter){
        stateTelemetry = ctx.telemetry().addData("Turret/State", "unknown");
        absoluteAngleTelemetry = ctx.telemetry().addData("Turret/Absolute(Anaolog)-Deg", "unknown");
        incrementalAngleTelemetry = ctx.telemetry().addData("Turret/Incremental-Deg", "unknown");
        servoLeft = new ServoEx(ctx.hardwareMap(), ConfigNames.turretServo1);
        servoRight = new ServoEx(ctx.hardwareMap(), ConfigNames.turretServo2);
        servoLeft.setInverted(SERVO_LEFT_INVERTED);
        servoRight.setInverted(SERVO_RIGHT_INVERTED);

        turnOffServos = mode(self -> {
            stateTelemetry.setValue("off");
            servoLeft.disable();
            servoRight.disable();
            return self;
        });
        noop = mode(self -> {
            stateTelemetry.setValue("noop");
            return self;
        });
        if(resetCenter){
            startMode = setRawPositionMode(SERVO_CENTER_POSITION);
        } else{
            startMode = noop;
        }

    }

    public Channel<Angle> getTargetHeadingsChannel(){
        return targetHeadingsChannel;
    }


    private Mode setRawPositionMode(double servoPosition){
        return mode(self -> {
            stateTelemetry.setValue("Raw Position Set ( " + servoPosition + ")");
            setServos(servoPosition);
            return self;
        });
    }

    public void setServos(double pos){
        servoLeft.set(pos);
        servoRight.set(pos + offsetServos);
    }

    //unoptimized angle setting
    private Mode setRawAngleMode(Angle intakeAngle){
        return mode(self -> {
            Angle turretAngle = convertIntakeAngleToTurretAngle(intakeAngle);
            targetTurretAngle = optimizeAngle(turretAngle);//convert from intake target angle to turret angle
            stateTelemetry.setValue("Raw Angle Set ( " + targetTurretAngle.toDegrees() + " deg)");
            setServos(convertAngleToServoRange(targetTurretAngle));
            return self;
        });
    }

    private Mode setOptimizedAngleMode(Angle intakeAngle){
        return mode(self -> {
            Angle turretAngle = convertIntakeAngleToTurretAngle(intakeAngle);
            targetTurretAngle = optimizeAngle(turretAngle);
            stateTelemetry.setValue("Optimized Turret Angle: " + targetTurretAngle.toDegrees() + " deg");
            setServos(convertAngleToServoRange(targetTurretAngle));
            return self;
        });
    }

    //continually sets position to target heading received through channel
    //1) receive target heading (rad)
    //2) convert that turret target heading position to angle
    //3) convert that angle to a turret set position
    //4) set servo group to that turret set position
    private Mode trackingMode(){
        return mode(self -> {
            if(targetHeadingsChannel != null){
                Angle intakeTarget = targetHeadingsChannel.poll();

                if(intakeTarget != null) {
                    Angle turretTarget = convertIntakeAngleToTurretAngle(intakeTarget);
                    targetTurretAngle = optimizeAngle(turretTarget);
                    stateTelemetry.setValue("Tracking Angle ( " + targetTurretAngle.toDegrees() + " deg)");
                    setServos(convertAngleToServoRange(targetTurretAngle));
                } else{
                    stateTelemetry.setValue("Tracking Angle: null");
                }
            }
            return self;
        });
    }

    public void setRawPosition(double servoPosition){
        switchTo(setRawPositionMode(servoPosition));
    }
    public void setRawAngle(Angle angle){
        switchTo(setRawAngleMode(angle));
    }
    //this uses the optimized algorithm to minimize distance traveled considering warp arounds
    public void setOptimizedAngle(Angle angle){
        switchTo(setOptimizedAngleMode(angle));
    }

    public void setTrackingMode(){
        switchTo(trackingMode());
    }

    public void turnOffServos(){
        switchTo(turnOffServos);
    }
    public void noop(){
        switchTo(noop);
    }


    //pass in intakeAngle we want turret to face
    public Angle convertIntakeAngleToTurretAngle(Angle intakeAngle){
        Angle turretCorrectSetAngle = Angle.fromRadians(intakeAngle.getValue() - TURRET_CENTER_ANGLE.toRadians());
        return turretCorrectSetAngle;
    }


    @NonNull
    @Override
    protected Mode init() {
        return startMode;
    }

    private Angle optimizeAngle(Angle targetAngle) {
        Angle bestCandidate = null;
        Angle targetAngleWrapped = targetAngle.wrap();
        Angle minGap = Angle.fromRadians(Double.MAX_VALUE);
        Angle[] candidates = {
                targetAngleWrapped,
                targetAngleWrapped.sub(Angle.fromRadians(2 * Math.PI)),
                targetAngleWrapped.add(Angle.fromRadians(2 * Math.PI))
        };

        for (Angle candidate : candidates) {
            if (!angleRange.contains(candidate.toRadians())) continue;
            Angle gap = incrementalAngle.sub(candidate).abs();
            if (gap.less(minGap)) {
                minGap = gap;
                bestCandidate = candidate;
            }
        }

        if (bestCandidate != null) return bestCandidate;

        Angle low = Angle.fromRadians(angleRange.low());
        Angle high = Angle.fromRadians(angleRange.high());
        return incrementalAngle.distance(low).le(incrementalAngle.distance(high))
                ? low : high;
    }

    public double convertAngleToServoRange(Angle angle){
        //get as fraction of max angle range
        double fraction = ((angle.toRadians()) + TURRET_HALF_TRAVEL.toRadians()) / (2 * TURRET_HALF_TRAVEL.toRadians());
        return servoRange.low() + fraction * (servoRange.high() - servoRange.low());
    }

    public void updateIncrementalAngle(Angle rawPosition){
        this.rawIncrementalAngle = rawPosition;
        incrementalAngle = Angle.fromRadians(
                TURRET_CENTER_ANGLE.getValue() + rawPosition.getValue() - ANALOG_ENCODER_CENTER_POS.getValue()
        );
    }

    public Angle updateAbsoluteAngle(Angle rawAngle){
        this.rawAbsoluteAngle = rawAngle;
        double correctedAngle = rawAngle.toRadians() - ANALOG_ENCODER_CENTER_POS.toRadians();
        absoluteAngle = Angle.fromRadians(TURRET_CENTER_ANGLE.toRadians() + correctedAngle).wrap();
        return absoluteAngle;
    }

    public boolean isAtAngle(Angle angle, double toleranceRad){
        return getTurretHeadingError(angle) < toleranceRad;
    }

    public boolean isAtAngle(double toleranceRad){
        return Math.abs(targetTurretAngle.toRadians() - incrementalAngle.toRadians()) < toleranceRad;
    }

    public double getTurretHeadingError(Angle angle){
        return  Math.abs(angle.toRadians() - incrementalAngle.toRadians());
    }

    public Angle currentIncrementalPosition(){
        return incrementalAngle;
    }
    public Angle currentAbsolutePosition(){
        return absoluteAngle;
    }

    public void updateTelemetry(){
        absoluteAngleTelemetry.setValue(absoluteAngle.toDegrees());
        incrementalAngleTelemetry.setValue(incrementalAngle.toDegrees());
    }


    public Angle getRawAbsoluteAngle(){
        return rawAbsoluteAngle;
    }


    public Angle getRawIncrementalAngle(){
        return rawIncrementalAngle;
    }


}
