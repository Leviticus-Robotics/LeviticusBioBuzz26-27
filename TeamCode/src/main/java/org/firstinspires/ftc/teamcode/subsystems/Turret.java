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
 */
@Config
public class Turret extends SwitchedStateMachine {
    ServoEx servoLeft;
    ServoEx servoRight;
    ServoExGroup servoGroup;
    public static Range servoRange = new Range(0, 1);
    public static double SERVO_CENTER_POSITION = 0.5;
    //can reference this if want to set to center in nonTurret class

    //this is the servo position where angle 0 is defined
    //turret launches ball in the direction of the intake <-> that is angle 0 and servo centered position value
    public static boolean leftInverted = false, rightInverted = false;
    public static Range angleRange = new Range(Math.toRadians(-194), Math.toRadians(194));
    //Changing servo center position will require re-adjusting angle range accordingly


    private Channel<Angle> targetHeadingsChannel = linkedSingleChannel();//receives a channel of target heading inputs for turret for sotm/lut
    private Angle currentTurretAngle  = new Angle();
    //obtained through encoder
    Mode off;
    protected Telemetry.Item stateTelemetry;

    //check if need to pass in start position from encoder to set to default position because can reset to absolute position
    public Turret(Context ctx){
        stateTelemetry = ctx.telemetry().addData("Turret/State", "unknown");
        servoLeft = new ServoEx(ctx.hardwareMap(), ConfigNames.turretServo1);
        servoRight = new ServoEx(ctx.hardwareMap(), ConfigNames.turretServo2);

        servoGroup = new ServoExGroup(servoLeft, servoRight);

        off = mode(self -> {
            stateTelemetry.setValue("off");
            servoGroup.disable();
            return self;
        });
    }

    public Channel<Angle> getTargetHeadingsChannel(){
        return targetHeadingsChannel;
    }


    public Mode setRawPositionMode(double servoPosition){
        return mode(self -> {
            stateTelemetry.setValue("Raw Position Set ( " + servoPosition + ")");
            servoGroup.set(servoPosition);
            return self;
        });
    }


    //unoptimized angle setting
    public Mode setRawAngleMode(Angle angle){
        return mode(self -> {
            stateTelemetry.setValue("Raw Angle Set ( " + angle.toDegrees() + " deg)");
            servoGroup.set(convertAngleToServoRange(angle));
            return self;
        });
    }

    public Mode setOptimizedAngleMode(Angle angle){
        return mode(self -> {
            stateTelemetry.setValue("Optimized Angle Set ( " + angle.toDegrees() + " deg)");
            servoGroup.set(convertAngleToServoRange(optimizeAngle(angle)));
            return self;
        });
    }

    //continually sets position to target heading received through channel
    //1) receive target heading (rad)
    //2) convert that turret target heading position to angle
    //3) convert that angle to a turret set position
    //4) set servo group to that turret set position
    public Mode trackingMode(){
        return mode(self -> {
            if(targetHeadingsChannel != null){
                Angle targetAngle = targetHeadingsChannel.poll();

                if(targetAngle != null) {
                    stateTelemetry.setValue("Tracking Angle ( " + targetAngle.toDegrees() + " deg)");
                    servoGroup.set(convertAngleToServoRange(optimizeAngle(targetAngle)));
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

    public void setTrackignMode(){
        switchTo(trackingMode());
    }

    public void off(){
        switchTo(off);
    }



    @NonNull
    @Override
    protected Mode init() {
        return off;
    }

    private Angle optimizeAngle(Angle targetAngle) {
        Angle bestCandidate = new Angle();
        Angle targetAngleWrapped = targetAngle.wrap();
        Angle minGap = Angle.fromRadians(Double.MAX_VALUE);
        Angle[] candidates = {
                targetAngleWrapped,
                targetAngleWrapped.sub(Angle.fromRadians(2 * Math.PI)),
                targetAngleWrapped.add(Angle.fromRadians(2 * Math.PI))
        };

        for (Angle candidate : candidates) {
            if (!angleRange.contains(candidate.toRadians())) continue;
            Angle gap = currentTurretAngle.sub(candidate).abs();
            if (gap.less(minGap)) {
                minGap = gap;
                bestCandidate = candidate;
            }
        }

        return bestCandidate;
    }

    public double convertAngleToServoRange(Angle angle){
        return angleRange.toRange(angle.toRadians(), servoRange);
    }



}
