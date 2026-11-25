package org.firstinspires.ftc.teamcode.subsystems;

import static dev.frozenmilk.dairy.mercurial.continuations.Continuations.expression;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.config.Config;
import com.pedropathing.paths.Path;
import com.pedropathing.revhub.localizers.Encoder;
import com.qualcomm.hardware.digitalchickenlabs.OctoQuad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.seattlesolvers.solverslib.hardware.motors.Motor;
import com.seattlesolvers.solverslib.hardware.servos.ServoEx;
import com.seattlesolvers.solverslib.hardware.servos.ServoExGroup;
import com.seattlesolvers.solverslib.util.Direction;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.constants.ConfigNames;
import org.firstinspires.ftc.teamcode.hardware.CachedServoEx;
import org.firstinspires.ftc.teamcode.hardware.MotorEx;
import org.firstinspires.ftc.teamcode.util.Angle;
import org.firstinspires.ftc.teamcode.util.Range;
import org.jetbrains.annotations.NotNull;

import java.util.ArrayList;
import java.util.List;

import dev.frozenmilk.dairy.mercurial.continuations.Continuation;
import dev.frozenmilk.dairy.mercurial.continuations.Continuations;
import dev.frozenmilk.dairy.mercurial.environments.Reference;
import dev.frozenmilk.dairy.mercurial.ftc.Context;
import dev.frozenmilk.dairy.mercurial.processes.Channel;
import dev.frozenmilk.dairy.mercurial.processes.StateMachine;
import dev.frozenmilk.dairy.mercurial.processes.SwitchedStateMachine;

/** Turret
 *
 *  3 different coordinate planes
 *  1) Encoders -> Incremental Encoder(-max to +max), no need to wrap here, in terms of turret angle
 *  2) Intake-Relative angles -> (-PI to +PI), 0 is intake facing forward
 *  3) Turret-Relatvie angle -> (-PI to +PI), 0 is turret facing away from intake, also the turret CENTER
 *  To set an angle -> set the intake angle -> transform to turret relative -> set to servos
 *  To get encoder angles -> convert the encoder count to the turret relative angle
 *  Absolute can only go (-PI to PI) but not full 360 from Turret gear perspective
 *  Incremental can go infinity either ways
 */
@Config
public class Turret extends SwitchedStateMachine {
    public CachedServoEx servoLeft;
    public CachedServoEx servoRight;
    public static Range servoRange = new Range(0 , 1);
    public static Range availableRange = new Range(0 , 1);
    public static double SERVO_CENTER_POSITION = 0.5;//this is facing away from the intake(180)
    public static Angle ANALOG_ENCODER_CENTER_POS = Angle.fromRadians(-2.0 * Math.PI / 180.0);//need to find this value
    public static Angle TURRET_CENTER_ANGLE = Angle.fromRadians(-Math.PI).wrap();

    public static Angle TURRET_HALF_TRAVEL = Angle.fromRadians(173.0 * Math.PI / 180.0);
    public static double TURRET_OFFSET_SERVO_DELTA = 0;

    //goes -120 and +120 from the servo center
//    public static Range angleRange = new Range(
//            TURRET_CENTER_ANGLE.sub(TURRET_HALF_TRAVEL).toRadians(),
//            TURRET_CENTER_ANGLE.add(TURRET_HALF_TRAVEL).toRadians()
//    );
    //this angle range is assuming the intake facing forward is zero degrees
    //center might not be 0 degrees


    private Channel<Angle> targetHeadingsChannel = linkedSingleChannel();//receives a channel of target heading inputs for turret for sotm/lut
    private Angle absoluteAngle = new Angle();
    private Angle rawAbsoluteAngle = new Angle();
    private Angle incrementalAngle = new Angle();
    private Angle rawIncrementalAngle = new Angle();
    public Angle targetTurretAngle; // current turret angle from turret frame
    public static boolean SERVO_LEFT_INVERTED = false;
    public static boolean SERVO_RIGHT_INVERTED = false;

    Mode turnOffServos;
    Mode noop;
    Mode startMode;
    private Telemetry.Line startTelemetry;
    private Telemetry.Item stateTelemetry;
    private Telemetry.Item trackingIntakeAngleTelemetry;
    private Telemetry.Item trimDegreeTelemetry;
    private Telemetry.Item trackingCorrectedAngleTelemetry;//with trims
    private Telemetry.Item absoluteAngleTelemetry;
    private Telemetry.Item incrementalAngleTelemetry;
    private Telemetry.Item turretRelativeTelemetry;
    private Telemetry.Item servoPositionL;
    private Telemetry.Item servoPositionR;
    private Telemetry.Line endBreakLine;

    public static double offsetServos = 0.01;

    private double aimTrimRadians = 0; //manual correction
    public static double AIM_TRIM_STEP_DEG = 2;

    public double turretAngleErrorRad = 0;


    //check if need to pass in start position from encoder to set to default position because can reset to absolute position
    //passing in true to reset zero moves turret back to center position
    public Turret(Context ctx, boolean resetCenter){
        servoLeft = new CachedServoEx(ctx.hardwareMap(), ConfigNames.turretServo1);
        servoRight = new CachedServoEx(ctx.hardwareMap(), ConfigNames.turretServo2);
        startTelemetry = ctx.telemetry().addLine("Turret");
        stateTelemetry = ctx.telemetry().addData("State", "unknown");
        trackingIntakeAngleTelemetry = ctx.telemetry().addData("Tracking intake angle(deg)", "not used");
        trimDegreeTelemetry = ctx.telemetry().addData("Trim(deg)", aimTrimRadians);
        trackingCorrectedAngleTelemetry = ctx.telemetry().addData("Tracking corrected(deg)", "not used");
        turretRelativeTelemetry = ctx.telemetry().addData("Turret Rel(deg)", "unknown");
        servoPositionL = ctx.telemetry().addData("Pos Left", servoLeft.get());
        servoPositionR = ctx.telemetry().addData("Pos Right", servoRight.get());
        absoluteAngleTelemetry = ctx.telemetry().addData("Abs-Deg", "unknown");
        incrementalAngleTelemetry = ctx.telemetry().addData("Incremental-Deg", "unknown");
        endBreakLine = ctx.telemetry().addLine("-----------------------");
        servoLeft.setInverted(SERVO_LEFT_INVERTED);
        servoRight.setInverted(SERVO_RIGHT_INVERTED);

        turnOffServos = mode(self -> {
            stateTelemetry.setValue("disabled");
            servoLeft.disable();
            servoRight.disable();
            return self;
        });
        noop = mode(self -> {
            stateTelemetry.setValue("noop");
            return self;
        });
        if(resetCenter){
            startMode = setRawPositionMode(SERVO_CENTER_POSITION + TURRET_OFFSET_SERVO_DELTA, false);
        } else{
            startMode = noop;
        }

    }

    public Channel<Angle> getTargetHeadingsChannel(){
        return targetHeadingsChannel;
    }


    private Mode setRawPositionMode(double servoPosition, boolean shouldOffsetServos){
        return entranceMode(self -> {
            stateTelemetry.setValue("Raw Position Set ( " + servoPosition + ")");
            setServos(servoPosition, shouldOffsetServos);
        });
    }
    private void updateEncoderTelemetry(){
        absoluteAngleTelemetry.setValue(absoluteAngle.toDegrees());
        incrementalAngleTelemetry.setValue(incrementalAngle.toDegrees());
    }

    public void setServos(double pos, boolean shouldOffsetServos){
        //update the encoder values
        updateEncoderTelemetry();
        servoLeft.set(pos);
        servoRight.set(shouldOffsetServos ? pos + offsetServos : pos);
        servoPositionL.setValue(pos);
        servoPositionR.setValue(shouldOffsetServos ? pos + offsetServos : pos);
    }

    //unoptimized angle setting
    private Mode setRawAngleMode(Angle intakeAngle, boolean shouldOffsetServos){
        return entranceMode(self -> {
            Angle turretAngle = convertIntakeAngleToTurretAngle(intakeAngle);
//            targetTurretAngle = optimizeAngle(turretAngle);//convert from intake target angle to turret angle
            stateTelemetry.setValue("Raw Angle Set ( " + turretAngle.toDegrees() + " deg)");
            setServos(convertAngleToServoRange(turretAngle), shouldOffsetServos);
        });
    }

//    private Mode setOptimizedAngleMode(Angle intakeAngle){
//        return mode(self -> {
//            Angle turretAngle = convertIntakeAngleToTurretAngle(intakeAngle);
//            targetTurretAngle = optimizeAngle(turretAngle);
//            stateTelemetry.setValue("Optimized Turret Angle: " + targetTurretAngle.toDegrees() + " deg");
//            setServos(convertAngleToServoRange(targetTurretAngle));
//            return self;
//        });
//    }

    //continually sets position to target heading received through channel
    //1) receive target heading (rad)
    //2) convert that turret target heading position to angle
    //3) convert that angle to a turret set position
    //4) set servo group to that turret set position
    private Mode trackingMode(boolean shouldOffsetServos){
        return entranceMode(self -> {
            stateTelemetry.setValue("Tracking Angle");
        }, self -> {
            if(targetHeadingsChannel != null){
                Angle intakeTarget = targetHeadingsChannel.query();
                if(intakeTarget != null){
                    stateTelemetry.setValue("Tracking");
                    intakeTarget.wrap();
                } else{
                    stateTelemetry.setValue("null");
                    return self;
                }

                Angle turretTarget = convertIntakeAngleToTurretAngle(intakeTarget).wrap();
                double adjustedTurretRad = turretTarget.toRadians() + aimTrimRadians;
                targetTurretAngle = Angle.fromRadians(adjustedTurretRad).wrap();
                //make sure to wrap this -PI to PI

                //update telemetry
                trackingIntakeAngleTelemetry.setValue(intakeTarget.toDegrees());
                trimDegreeTelemetry.setValue(Math.toDegrees(aimTrimRadians));
                trackingCorrectedAngleTelemetry.setValue(Math.toDegrees(adjustedTurretRad));
                turretRelativeTelemetry.setValue(targetTurretAngle.toDegrees());

                setServos(convertAngleToServoRange(targetTurretAngle), shouldOffsetServos);
            }
            return self;
        });
    }

    public void setRawPosition(double servoPosition, boolean shouldOffsetServos){
        switchTo(setRawPositionMode(servoPosition, shouldOffsetServos));
    }
    public void setRawAngle(Angle angle, boolean shouldOffsetServos){
        switchTo(setRawAngleMode(angle, shouldOffsetServos));
    }
    //this uses the optimized algorithm to minimize distance traveled considering warp arounds
//    public void setOptimizedAngle(Angle angle){
//        switchTo(setOptimizedAngleMode(angle));
//    }

    public void setTrackingMode(boolean shouldOffsetServos){
        switchTo(trackingMode(shouldOffsetServos));
    }

    public Continuation.Builder setTrackingModeSync(boolean shouldOffsetServos) {
        return expression((scope, self) -> {
           scope.runExec(() -> setTrackingMode(shouldOffsetServos));
           return Continuations.noop;
        });
    }

    public void turnOffServos(){
        switchTo(turnOffServos);
    }
    public void noop(){
        switchTo(noop);
    }


    //this method converts it from intake angle range (-180 <-> 180), to the turret relative range(-180, 180)
    public Angle convertIntakeAngleToTurretAngle(Angle intakeAngle){
        Angle turretCorrectSetAngle = Angle.fromRadians(intakeAngle.toRadians() - TURRET_CENTER_ANGLE.toRadians());
        return turretCorrectSetAngle.wrap();//wraps from -180 to 180
    }


    @NonNull
    @Override
    protected Mode init() {
        return startMode;
    }

//    private Angle optimizeAngle(Angle targetAngle) {
//        Angle bestCandidate = null;
//        Angle targetAngleWrapped = targetAngle.wrap();
//        Angle minGap = Angle.fromRadians(Double.MAX_VALUE);
//        Angle[] candidates = {
//                targetAngleWrapped,
//                targetAngleWrapped.sub(Angle.fromRadians(2 * Math.PI)),
//                targetAngleWrapped.add(Angle.fromRadians(2 * Math.PI))
//        };
//
//        for (Angle candidate : candidates) {
//            if (!angleRange.contains(candidate.toRadians())) continue;
//            Angle gap = incrementalAngle.sub(candidate).abs();
//            if (gap.less(minGap)) {
//                minGap = gap;
//                bestCandidate = candidate;
//            }
//        }
//
//        if (bestCandidate != null) return bestCandidate;
//
//        Angle low = Angle.fromRadians(angleRange.low());
//        Angle high = Angle.fromRadians(angleRange.high());
//        return incrementalAngle.distance(low).le(incrementalAngle.distance(high))
//                ? low : high;
//    }

    //assume that this is from the turret perspective, not the intake perspective
    //0 is from the turret perspective(facing away from the intake)
    //-60 is 120 from the intake perspective
    //60 is 240 from the intake perspective
    //assume the servo range is going from 0 (clockwise from center position) to 1(counterclockwise from center position)
    //servo travels counterclockwise as goes from 0 -> 1
    public double convertAngleToServoRange(Angle turretRelativeAngle){
        //get as fraction of max angle range
        double fraction = ((turretRelativeAngle.toRadians()) + TURRET_HALF_TRAVEL.toRadians()) / (2 * TURRET_HALF_TRAVEL.toRadians());
//        fraction = Math.min(Math.max(fraction, availableRange.low()), availableRange.high());//make sure servos not going outside range
        return servoRange.low() + fraction * (servoRange.high() - servoRange.low()) + TURRET_OFFSET_SERVO_DELTA;
    }//converts into servo range


    //same thing here, we want this relative to the intake perspective(-180 to 180)
    public void updateIncrementalAngle(Angle position){
        this.rawIncrementalAngle = position;
        incrementalAngle = Angle.fromRadians(
                TURRET_CENTER_ANGLE.getValue() + position.getValue() - ANALOG_ENCODER_CENTER_POS.getValue()
        ).wrap();
    }

    //incremental and absolute angle functions return the angle relative to the intake angle to be make most simple(-180 to 180) intake perspective
    //0 = 0 degrees intake perspective
    //40 = 40 degrees intake perspective
    public Angle updateAbsoluteAngle(Angle rawAngle){
        this.rawAbsoluteAngle = rawAngle;
        absoluteAngle = Angle.fromRadians(TURRET_CENTER_ANGLE.toRadians() +
                rawAngle.toRadians() - ANALOG_ENCODER_CENTER_POS.toRadians()).wrap();//converts to -180 to 180 from turret center
        return absoluteAngle;
    }

    //REFERENCE
    //targetTurretAngle - turret relative
    //Angle angle, incrementalAngle - intake relative
    //intake relative angle being passed in
    public boolean isAtAngle(Angle angle, double toleranceRad){
        return Math.abs(getTurretHeadingError(angle)) <= toleranceRad;
    }

    public boolean isAtAngle(double toleranceRad){
        if(targetTurretAngle == null){
            return false;
        }
        Angle currentTurretAngle = convertIntakeAngleToTurretAngle(incrementalAngle);
        Angle delta = targetTurretAngle.sub(currentTurretAngle).wrap();
        turretAngleErrorRad = delta.toRadians();
        return Math.abs(turretAngleErrorRad) <= toleranceRad;
    }

    //intake relative difference
    public double getTurretHeadingError(Angle angle){
        Angle delta = angle.sub(incrementalAngle).wrap();
        turretAngleErrorRad = delta.toRadians();
        return turretAngleErrorRad;
    }

    public Angle currentIncrementalPosition(){
        return incrementalAngle;
    }
    public Angle currentAbsolutePosition(){
        return absoluteAngle;
    }



    public void adjustAimTrimDeg(double deltaDegrees){
        aimTrimRadians = aimTrimRadians + Math.toRadians(deltaDegrees);
    }

    public void resetAimTrim(){
        aimTrimRadians = 0;
    }

    public double getAimTrimDegrees(){
        return Math.toDegrees(aimTrimRadians);
    }

    public Angle getRawAbsoluteAngle(){
        return rawAbsoluteAngle;
    }


    public Angle getRawIncrementalAngle(){
        return rawIncrementalAngle;
    }



}
