package org.firstinspires.ftc.teamcode.subsystems;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.config.Config;
import com.pedropathing.localization.MotionState;
import com.pedropathing.math.Pose;
import com.pedropathing.math.Vector;
import com.pedropathing.math.Vector2D;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.seattlesolvers.solverslib.hardware.motors.Motor;
import com.seattlesolvers.solverslib.hardware.motors.MotorGroup;
import com.seattlesolvers.solverslib.hardware.servos.ServoEx;
import com.seattlesolvers.solverslib.util.InterpLUT;
import com.seattlesolvers.solverslib.util.MathUtils;


import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.game.SIDE;
import org.firstinspires.ftc.teamcode.hardware.MotorEx;
import org.firstinspires.ftc.teamcode.constants.ConfigNames;
import org.firstinspires.ftc.teamcode.util.Angle;
import org.firstinspires.ftc.teamcode.util.MathFunctions;
import dev.frozenmilk.dairy.mercurial.ftc.Context;
import dev.frozenmilk.dairy.mercurial.processes.Channel;
import dev.frozenmilk.dairy.mercurial.processes.SwitchedStateMachine;

import org.firstinspires.ftc.teamcode.game.ALLIANCE;


/** Shooter
 * Modes: SOTM w/ LUT, Static LUT, Raw Power, Raw Velocity, Off
 * Uses: 2 motors (shooter) attached to the same axle
 * Using Motor Group so that only PID one motor, set the other to the same values
 */
@Config
//Single Shooter Motor: LUT Mode, SOTM, Regular Velocity, Regular Power
public class Shooter extends SwitchedStateMachine {
    MotorGroup motorGroup;
    public MotorEx shooterLeft; // motor wire is left
    public MotorEx shooterRight; // motor wire is right
    ServoEx hoodServo;
    Channel<MotionState> motionStates = linkedSingleChannel();
    public static double[] pidCoefficients = new double[] {0.003, 0.0, 0.0};
    public static double[] feedforwardCoefficients = new double[]{0.0291871, 0.000355203, 0};
    public static boolean usePIDCoefficients = true;
    public static boolean useFeedforwardCoefficients = true;
    public static boolean HOOD_SERVO_INVERTED = false;
    public static double HOOD_DEFAULT = 0.5;

//    public static boolean DIRECTION_REVERSE = false;

    public static Pose AIM_BLUE_CLOSE = new Pose(61, 55);
    public static Pose AIM_BLUE_FAR = new Pose(61, 67);
    public static Pose AIM_RED_CLOSE = new Pose(141.5 - 61, 55);
    public static Pose AIM_RED_FAR = new Pose(141.5 - 61, 67);
    public double currentVoltage;
    private double currentTargetVelocity;//velocity that is currently being sent to the motors
    private double shootingTargetVelocity;//desired velocity for shooting (not default velocity)
    boolean useDefaultVelocity = true; //set it to true initally
    private Telemetry.Item stateTelemetry;
    Mode off;
    AimCalculator aimCalculator;
    public static double DEFAULT_TOLERANCE = 50;
    Channel<Angle> targetHeadingChannel;

    //Express Constants in terms of lines (array of 4): (m1, b1, m2, b2)
    //This is for close side, far side is just the opposite but uses the same constants
    public static double[] WITHIN_SHOOTING_RANGE = new double[]{1, 10, -1, 154};
    public static double DEFAULT_VELOCITY = 500; // ticks/sec -> this is the velocity the shooter will go to when we don't need to shoot
    public Shooter(Context ctx){
        stateTelemetry = ctx.telemetry().addData("Shooter/State", "unknown");
        aimCalculator = new AimCalculator();

        shooterLeft = new MotorEx(ctx.hardwareMap(), ConfigNames.shooterLeft);
        shooterRight = new MotorEx(ctx.hardwareMap(), ConfigNames.shooterRight);
        shooterLeft.motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        shooterRight.motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        shooterRight.setInverted(true);//need to invert one of them first
        hoodServo = new ServoEx(ctx.hardwareMap(), ConfigNames.hoodServo);
        hoodServo.setInverted(HOOD_SERVO_INVERTED);

        motorGroup = new MotorGroup(shooterLeft, shooterRight);
        motorGroup.setRunMode(Motor.RunMode.VelocityControl);

        if(usePIDCoefficients){
            motorGroup.setVeloCoefficients(pidCoefficients[0], pidCoefficients[1], pidCoefficients[2]);
        }
        if(useFeedforwardCoefficients){
            motorGroup.setFeedforwardCoefficients(feedforwardCoefficients[0], feedforwardCoefficients[1], feedforwardCoefficients[2]);
        }


        off = entranceMode(from -> {
            motorGroup.setRunMode(Motor.RunMode.RawPower);
            motorGroup.set(0);
            setHood(HOOD_DEFAULT);
            stateTelemetry.setValue("off");
        });
    }


    @Config
    public static class AimCalculator{
        public InterpLUT distToVel;
        public InterpLUT distToKCorrection;
        public InterpLUT distToHoodPos;
        public static double[] distances =           new double[]{20,   30,   40,   50,   60,   70,  80};
        public static double[] velocities =          new double[]{700, 900, 1000, 1050, 1100, 1200, 1250};
        public static double[] velCorrectionFactor = new double[]{0.4, 0.5, 0.6,  0.54, 0.85, 0.9,  0.96}; // take time in the air and then subtract a bit
        public static double[] hoodPositions =       new double[]{0.2, 0.3,  0.4,  0.6,  0.65, 0.7,  0.8};
        public static int iterations = 10;
        public static double robotHeadingAimCorrectionFactor = -0;
        public AimCalculator() {
            distToVel = new InterpLUT();
            distToKCorrection = new InterpLUT();
            distToHoodPos = new InterpLUT();
            for (int i = 0; i < distances.length; i++) {
                distToVel.add(distances[i], velocities[i]);
                distToKCorrection.add(distances[i], velCorrectionFactor[i]);
                distToHoodPos.add(distances[i], hoodPositions[i]);
            }
            distToVel.createLUT();
            distToKCorrection.createLUT();
            distToHoodPos.createLUT();
        }


        //reference code: madness last year repo(TwoWheelShooter2 class) https://github.com/MidKnightMadness/MidknightMadnessDecode25-26/settings/access
        //Ryan double check the math/vector logic
        /**
         * Calculate target powers and heading.<br>
         * Algorithm:<br>
         * 1. Act like you're aiming for a distance d<br>
         * 2. Simulate error for shooting with distance d, making sure the parallel component of the error is 0<br>
         * 3. Update d by adding the error<br>
         * 4. Repeat steps 1-3 to iterative refine d
         * @param pose     The robot pose
         * @param velocity The robot velocity
         * @return velocity, heading
         */
        public double[] targetPowersHeading(Pose pose, Vector velocity, Pose targetPose) {
            Vector2D gap = targetPose.minus(pose).toVector2D();
            Vector2D dirParallel = gap.normalized();
            Vector2D dirPerp = Vector2D.cartesian(dirParallel.x(), dirParallel.y());
            dirPerp.rotate(-Math.PI / 2);
            double velParallel = velocity.dot(dirParallel.toVector()); // component parallel to line from robot to target pose
            double velPerp = velocity.dot(dirPerp.toVector()); // left is negative, right is positive

            double realDist = targetPose.distance(pose);
            double targetDist = realDist;
            double headingCorrection = 0;
            for (int i = 0; i < iterations; i++) {
                // Steps 1-2: simulation
                targetDist = MathUtils.clamp(targetDist, distances[0]+0.01, distances[distances.length - 1]-0.01); // find a better way later
                double kCorr = distToKCorrection.get(targetDist);
                double predict = targetDist + kCorr * velParallel;
                double predictPerp = kCorr * velPerp;
                double predictParallel = Math.sqrt(
                        Math.max(0, predict * predict - predictPerp * predictPerp)
                );
                headingCorrection = Math.atan2(predictPerp, predictParallel);
                // Steps 3-4: error and update
                double distanceIdeal = predictParallel != 0 ? (realDist / predictParallel) * predict : realDist;//changed for divide by zero error
                // d + correction + error = ideal d
                // (d + error) + correction = ideal d, update d += error
                targetDist += distanceIdeal - predict;
            }

            targetDist = MathUtils.clamp(targetDist, distances[0]+0.01, distances[distances.length - 1]-0.01); // find a better way later

            return new double[]{
                    distToVel.get(targetDist),
                    MathFunctions.normalizeAngle(targetPose.minus(pose).heading() + headingCorrection),
                    distToHoodPos.get(targetDist)
            };
        }

        public double getTargetHeading(Pose pose, Pose targetPose){
            return MathFunctions.normalizeAngle(targetPose.minus(pose).heading());
        }

    }

    public Mode lutSOTMMode(
            ALLIANCE alliance,
            SIDE side,
            Channel<Shooter.VelocityStatus> resultChannel,
            double tolerance
        ){
        boolean[] notified = new boolean[1];
        return egressMode(self -> {//used so that if it switches mode, it performs second block logic
                    stateTelemetry.setValue("LUT SOTM (" + alliance + " " + side);
                    motorGroup.setRunMode(MotorEx.RunMode.VelocityControl);
                    MotionState motionState = motionStates.query();//query to keep for now
                    if(motionState != null) {
                        Pose robotPose = motionState.pose();
                        Pose targetShootPose = getPose(alliance, side);

                        double[] solution = aimCalculator.targetPowersHeading(robotPose, motionState.velocity().toVector(), targetShootPose);
                        double targetVelocity = solution[0];

                        //pass in current voltage as long as value is valid
                        shootingTargetVelocity = targetVelocity; // need to update
                        currentTargetVelocity = useDefaultVelocity ? DEFAULT_VELOCITY : targetVelocity;

                        if(targetHeadingChannel != null) {
                            targetHeadingChannel.send(Angle.fromRadians(solution[1]));//send target heading over through channel
                        }
                        setShooter(currentTargetVelocity);
                        setHood(solution[2]); //set hood servo position

                        if (resultChannel != null && !notified[0]) {//only returns to channel if not targeting default velocity
                            if (isAtVelocity(shootingTargetVelocity, tolerance)) {
                                notified[0] = true;
                                resultChannel.sendIfEmpty(VelocityStatus.AT_TARGET);
                            }
                        }
                    }
                    return self;
                },
                self -> {
                    if(resultChannel != null){
                        resultChannel.sendIfEmpty(VelocityStatus.CANCELLED);
                    }
                }
        );
    }
    public Mode lutSOTMMode(ALLIANCE alliance, SIDE side, double tolerance){
        return lutSOTMMode(alliance, side, null, tolerance);
    }

    public Mode lutSOTMMode(ALLIANCE alliance, SIDE side){
        return lutSOTMMode(alliance, side, null, DEFAULT_TOLERANCE);
    }

    public void lutSOTM(ALLIANCE alliance, SIDE side, double tolerance){
        switchTo(lutSOTMMode(alliance, side, tolerance));
    }
    public void lutSOTM(ALLIANCE alliance, SIDE side){
        switchTo(lutSOTMMode(alliance, side));
    }
    public void setHood(double pos){
        hoodServo.set(pos);
    }

    //two channels used: one for the target heading(to go to turret or save) & one for velocity status
    public Mode lutStaticMode(
              ALLIANCE alliance,
              SIDE side,
              Channel<Shooter.VelocityStatus> resultChannel,
              double tolerance
        ){
        boolean[] notified = new boolean[1];
        return egressMode(self -> {//used so that if it switches mode, it performs second block logic
                stateTelemetry.setValue("LUT Static (" + alliance + " " + side);
                motorGroup.setRunMode(MotorEx.RunMode.VelocityControl);
                MotionState motionState = motionStates.query();//query for now
                if(motionState != null) {
                    Pose robotPose = motionState.pose();
                    Pose targetShootPose = getPose(alliance, side);

                    double distance = MathUtils.clamp(
                            robotPose.distance(targetShootPose),
                            AimCalculator.distances[0],
                            AimCalculator.distances[AimCalculator.distances.length - 1]
                    );

                    double targetVelocity = aimCalculator.distToVel.get(distance);
                    shootingTargetVelocity = targetVelocity; // need to update
                    currentTargetVelocity = useDefaultVelocity ? DEFAULT_VELOCITY : targetVelocity;
                    if(targetHeadingChannel != null) {
                        targetHeadingChannel.send(Angle.fromRadians(aimCalculator.getTargetHeading(robotPose, targetShootPose)));
                    }
                    //pass in current voltage as long as value is valid
                    setShooter(currentTargetVelocity);
                    setHood(aimCalculator.distToHoodPos.get(distance)); //set hood servo position

                    if (resultChannel != null && !notified[0]) {
                        if (isAtVelocity(shootingTargetVelocity, tolerance)) {//only returns to channel if not targeting default velocity
                            notified[0] = true;
                            resultChannel.sendIfEmpty(VelocityStatus.AT_TARGET);
                        }
                    }
                }
                return self;
            },
            self -> {
                if(resultChannel != null){
                    resultChannel.sendIfEmpty(VelocityStatus.CANCELLED);
                }
            }
        );
    }
    public Mode lutStaticMode(ALLIANCE alliance, SIDE side, double tolerance){
        return lutStaticMode(alliance, side, null, tolerance);
    }

    public Mode lutStaticMode(ALLIANCE alliance, SIDE side){
        return lutStaticMode(alliance, side, null, DEFAULT_TOLERANCE);
    }

    public void lutStatic(ALLIANCE alliance, SIDE side, double tolerance){
        switchTo(lutStaticMode(alliance, side, tolerance));
    }

    public void lutStatic(ALLIANCE alliance, SIDE side){
        switchTo(lutStaticMode(alliance, side));
    }

    @NonNull
    @Override
    protected Mode init() {
        return off;
    }

    public void setShooter(double currentTargetVelocity){
        shooterLeft.set(
                currentTargetVelocity,
                currentVoltage > 0 ? currentVoltage : MotorEx.REFERENCE_VOLTAGE
        );
        shooterRight.set(
                currentTargetVelocity,
                currentVoltage > 0 ? currentVoltage : MotorEx.REFERENCE_VOLTAGE
        );

    }


    private Mode fixedVelocityMode(double targetVelocity, double tolerance, Channel<Shooter.VelocityStatus> resultChannel){
        boolean[] notified = new boolean[1];
        currentTargetVelocity = targetVelocity;
        return egressMode(self -> {
            stateTelemetry.setValue("Fixed Velocity (" + targetVelocity + ")");
            motorGroup.setRunMode(MotorEx.RunMode.VelocityControl);
            shootingTargetVelocity = targetVelocity; // need to update
            currentTargetVelocity = useDefaultVelocity ? DEFAULT_VELOCITY : targetVelocity;
            setShooter(currentTargetVelocity);
            setHood(HOOD_DEFAULT);


            if (resultChannel != null && notified[0] == false) {
                    if (isAtVelocity(shootingTargetVelocity, tolerance)) {
                        notified[0] = true;
                        resultChannel.sendIfEmpty(VelocityStatus.AT_TARGET);
                    }
                }
                return self;
            }, self -> {
                if(resultChannel != null){
                    resultChannel.sendIfEmpty(VelocityStatus.CANCELLED);
                }
            });
    }

    //this will be the channel that the target heading gets sent over to
    public void setHeadingChannel(Channel<Angle> channel){
        targetHeadingChannel = channel;
    }


    public void fixedVelocity(double targetVelocity){
        switchTo(fixedVelocityMode(targetVelocity, DEFAULT_TOLERANCE, null));
    }

    public void fixedVelocity(double targetVelocity, double tolerance) {
        switchTo(fixedVelocityMode(targetVelocity, tolerance, null));
    }

    public void fixedVelocity(double targetVelocity, double tolerance, Channel<Shooter.VelocityStatus> resultChannel) {
        switchTo(fixedVelocityMode(targetVelocity, tolerance, resultChannel));//if result channel is null, it will pass in null to third parameter
    }



    private Mode fixedPowerMode(double power){//parametrized so it keeps holding the velocity
        return entranceMode(
                from -> {
                    stateTelemetry.setValue("Fixed Power (" + power + ")");
                    motorGroup.setRunMode(MotorEx.RunMode.RawPower);
                    motorGroup.set(power);
                    setHood(HOOD_DEFAULT);
                }
        );
    }


    public void fixedPower(double power){
        switchTo(fixedPowerMode(power));
    }

    public void off(){
        switchTo(off);
    }

    public Pose getPose(ALLIANCE alliance, SIDE side){
        if(alliance == ALLIANCE.BLUE){
            return side == SIDE.CLOSE ? AIM_BLUE_CLOSE : AIM_BLUE_FAR;
        } else{
            return side == SIDE.CLOSE ? AIM_RED_CLOSE : AIM_RED_FAR;
        }
    }

    public double getVelocity(){//returns velocity in ticks/sec
        return motorGroup.getVelocity();//need to check that this method works correctly
    }

    public double getError(double targetVelocity){//keep sign here
        return getVelocity() - targetVelocity;
    }

    public void updateVoltage(double voltage){
        if(voltage > 0){
            this.currentVoltage = voltage;
        }
    }



    //Conditions to start shooting automatically: 1) atVelocity && inside the zone
    public boolean isAtVelocity(double targetVelocity, double tolerance){//an invalid tolerance assumed to be no tolerance
        if(tolerance != 0) {
            return Math.abs(getVelocity() - targetVelocity) <= Math.abs(tolerance);
        } return true;
    }

    // Checks against currently commanded velocity(including default)
    public boolean isAtCurrentTargetVelocity(double tolerance) {
        return isAtVelocity(currentTargetVelocity, tolerance);
    }

    // Checks against calculated shooting velocity(not includign default)
    public boolean isAtShootingTargetVelocity(double tolerance) {
        return isAtVelocity(shootingTargetVelocity, tolerance);
    }

    //pass in side you're shooting from
    //if close side, it has to be less than both of the lines
    //if far side, it has to be greater than boht of the lines
    public boolean isValidShootPosition(SIDE side){
        MotionState motionState = motionStates.query();
        if(motionState == null) return false;
        Pose robotPose = motionState.pose();
        if(side == SIDE.CLOSE){
            if(robotPose.y() <= (WITHIN_SHOOTING_RANGE[0] * robotPose.x() + WITHIN_SHOOTING_RANGE[1]) && robotPose.y() <= (WITHIN_SHOOTING_RANGE[2] * robotPose.x() + WITHIN_SHOOTING_RANGE[3])){
                return true;
            }
        } else{//SIDE.FAR
            if(robotPose.y() >= (WITHIN_SHOOTING_RANGE[0] * robotPose.x() + WITHIN_SHOOTING_RANGE[1]) && robotPose.y() >= (WITHIN_SHOOTING_RANGE[2] * robotPose.x() + WITHIN_SHOOTING_RANGE[3])){
                return true;
            }
        }
        return false;
    }
    public enum VelocityStatus{
        AT_TARGET,
        CANCELLED
    }

    public void setUseDefaultVelocity(boolean useDefault){
        this.useDefaultVelocity = useDefault;
    }

    public double getCurrentTargetVelocity(){//includes default velocity
        return currentTargetVelocity;
    }

    public double getShootingTargetVelocity(){//does not include default velocity
        return shootingTargetVelocity;
    }



    public void sendMotionState(MotionState motionState){
        motionStates.send(motionState);
    }
}
