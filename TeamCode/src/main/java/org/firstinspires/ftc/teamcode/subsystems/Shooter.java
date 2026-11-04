package org.firstinspires.ftc.teamcode.subsystems;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.config.Config;
import com.pedropathing.localization.MotionState;
import com.pedropathing.math.Pose;
import com.pedropathing.math.Vector;
import com.pedropathing.math.Vector2D;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.seattlesolvers.solverslib.hardware.motors.Motor;
import com.seattlesolvers.solverslib.util.InterpLUT;
import com.seattlesolvers.solverslib.util.MathUtils;


import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.game.SIDE;
import org.firstinspires.ftc.teamcode.hardware.MotorEx;
import org.firstinspires.ftc.teamcode.constants.ConfigNames;
import org.firstinspires.ftc.teamcode.util.MathFunctions;
import dev.frozenmilk.dairy.mercurial.ftc.Context;
import dev.frozenmilk.dairy.mercurial.processes.Channel;
import dev.frozenmilk.dairy.mercurial.processes.SwitchedStateMachine;

import org.firstinspires.ftc.teamcode.game.ALLIANCE;

@Config
//Single Shooter Motor
public class Shooter extends SwitchedStateMachine {
    public MotorEx shooter;
    Channel<MotionState> motionStates = linkedSingleChannel();
    public static double[] pidCoefficients = new double[] {0.003, 0.0, 0.0};
    public static double[] feedforwardCoefficients = new double[]{0.0291871, 0.000355203, 0};
    public static boolean usePIDCoefficients = true;
    public static boolean useFeedforwardCoefficients = true;
    public static boolean DIRECTION_REVERSE = false;

    public static Pose AIM_BLUE_CLOSE = new Pose(61, 55);
    public static Pose AIM_BLUE_FAR = new Pose(61, 67);
    public static Pose AIM_RED_CLOSE = new Pose(141.5 - 61, 55);
    public static Pose AIM_RED_FAR = new Pose(141.5 - 61, 67);
    public double previousVoltage;
    private Telemetry.Item stateTelemetry;
    Mode off;
    AimCalculator aimCalculator;
    public static double DEFAULT_TOLERANCE = 50;
    public Shooter(Context ctx){
        stateTelemetry = ctx.telemetry().addData("Shooter/State", "unknown");
        aimCalculator = new AimCalculator();
        shooter = new MotorEx(ctx.hardwareMap(), ConfigNames.shooter);
        shooter.setRunMode(Motor.RunMode.VelocityControl);
        shooter.setInverted(DIRECTION_REVERSE);
        shooter.motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        if(usePIDCoefficients){
            shooter.setVeloCoefficients(pidCoefficients[0], pidCoefficients[1], pidCoefficients[2]);
        }
        if(useFeedforwardCoefficients){
            shooter.setFeedforwardCoefficients(feedforwardCoefficients[0], feedforwardCoefficients[1], feedforwardCoefficients[2]);
        }


        off = entranceMode(from -> {
            shooter.setRunMode(Motor.RunMode.RawPower);
            shooter.set(0);
            stateTelemetry.setValue("off");
        });
    }


    @Config
    public static class AimCalculator{
        public InterpLUT distToVel;
        public InterpLUT distToKCorrection;
        public static double[] distances =  new double[] {20,   30,   40,   50,   60,   70,  80};
        public static double[] velocities = new double[] {700, 900, 1000, 1050, 1100, 1200, 1250};
        public static double[] velCorrectionFactor = {0.4, 0.5, 0.6,0.54, 0.85,0.9,0.96,1.02,1.07, 1.1, 1.14,1.18,1.25,1.31}; // take time in the air and then subtract a bit
        public static int iterations = 10;
        public static double robotHeadingAimCorrectionFactor = -0;
        public AimCalculator() {
            distToVel = new InterpLUT();
            distToKCorrection = new InterpLUT();
            for (int i = 0; i < distances.length; i++) {
                distToVel.add(distances[i], velocities[i]);
                distToKCorrection.add(distances[i], velCorrectionFactor[i]);
            }
            distToVel.createLUT();
            distToKCorrection.createLUT();
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
         * @return bottom velocity, top velocity, heading
         */
        public double[] targetPowersHeading(Pose pose, Vector velocity, Pose targetPose) {
            Vector2D gap = targetPose.minus(pose).toVector2D();
            Vector2D dirParallel = gap.normalized();
            Vector2D dirPerp = Vector2D.cartesian(gap.x(), gap.y());
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
                double distanceIdeal = (realDist / predictParallel) * predict;
                // d + correction + error = ideal d
                // (d + error) + correction = ideal d, update d += error
                targetDist += distanceIdeal - predict;
            }

            targetDist = MathUtils.clamp(targetDist, distances[0]+0.01, distances[distances.length - 1]-0.01); // find a better way later

            return new double[]{
                    distToVel.get(targetDist),
                    MathFunctions.normalizeAngle(targetPose.minus(pose).heading() + headingCorrection)
            };
        }

    }

    public Mode lutSOTMMode(ALLIANCE alliance, SIDE side, Channel<Shooter.LUTStatus> resultChannel, double tolerance){
        boolean[] notified = new boolean[1];
        return egressMode(self -> {//used so that if it switches mode, it performs second block logic
                    stateTelemetry.setValue("LUT Static (" + alliance + " " + side);
                    shooter.setRunMode(MotorEx.RunMode.VelocityControl);
                    MotionState motionState = motionStates.poll();
                    Pose robotPose = motionState.pose();
                    Pose targetShootPose = getPose(alliance, side);

                    double targetVelocity = aimCalculator.targetPowersHeading(robotPose, motionState.velocity().toVector(), targetShootPose)[0];
                    shooter.set(targetVelocity);

                    if(resultChannel != null && !notified[0]){
                        if(isAtVelocity(targetVelocity, tolerance)){
                            resultChannel.sendIfEmpty(LUTStatus.AtTarget);
                        }
                    }
                    return self;
                },
                self -> {
                    if(resultChannel != null){
                        resultChannel.sendIfEmpty(LUTStatus.Cancelled);
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


    public Mode lutStaticMode(ALLIANCE alliance, SIDE side, Channel<Shooter.LUTStatus> resultChannel, double tolerance){
        boolean[] notified = new boolean[1];
        return egressMode(self -> {//used so that if it switches mode, it performs second block logic
                stateTelemetry.setValue("LUT Static (" + alliance + " " + side);
                shooter.setRunMode(MotorEx.RunMode.VelocityControl);
                MotionState motionState = motionStates.poll();
                Pose robotPose = motionState.pose();
                Pose targetShootPose = getPose(alliance, side);

                double distance = robotPose.distance(targetShootPose);
                double targetVelocity = aimCalculator.distToVel.get(distance);
                shooter.set(targetVelocity);

                if(resultChannel != null && !notified[0]){
                    if(isAtVelocity(targetVelocity, tolerance)){
                        resultChannel.sendIfEmpty(LUTStatus.AtTarget);
                    }
                }
                return self;
            },
            self -> {
                if(resultChannel != null){
                    resultChannel.sendIfEmpty(LUTStatus.Cancelled);
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

    @NonNull
    @Override
    protected Mode init() {
        return off;
    }


    private Mode fixedVelocityMode(double targetVelocity, double tolerance, Channel<Shooter.VelocityStatus> resultChannel){
        boolean[] notified = new boolean[1];
        return entranceMode(from -> {
                    stateTelemetry.setValue("Fixed Velocity (" + targetVelocity + ")");
                    shooter.setRunMode(MotorEx.RunMode.VelocityControl);
                },
                self -> {
                    shooter.set(targetVelocity);
                    if (resultChannel != null && notified[0] == false) {
                        if (isAtVelocity(targetVelocity, tolerance)) {
                            notified[0] = true;
                            resultChannel.sendIfEmpty(Shooter.VelocityStatus.AtTarget);
                        }
                    }
                    return self;
                });
    }


    public void fixedVelocity(double targetVelocity){
        switchTo(fixedVelocityMode(targetVelocity, DEFAULT_TOLERANCE, null));
    }

    public void fixedVelocity(double targetVelocity, double tolerance) {
        switchTo(fixedVelocityMode(targetVelocity, tolerance, null));
    }

    public void fixedVelocity(double targetVelocity, double tolerance, Channel<Shooter.VelocityStatus> resultChannel) {
        if (resultChannel != null) {
            switchTo(fixedVelocityMode(targetVelocity, tolerance, resultChannel));
        } else{
            fixedVelocity(targetVelocity, tolerance, null);
        }
    }

    private Mode fixedPowerMode(double power){//parametrized so it keeps holding the velocity
        return entranceMode(
                from -> {
                    stateTelemetry.setValue("Fixed Power (" + power + ")");
                    shooter.setRunMode(MotorEx.RunMode.RawPower);
                    shooter.set(power);
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
        return shooter.getVelocity();
    }

    public boolean isAtVelocity(double targetVelocity, double tolerance){//an invalid tolerance assumed to be no tolerance
        if(tolerance != 0) {
            return Math.abs(getVelocity() - targetVelocity) <= Math.abs(tolerance);
        } return true;
    }

    public double getError(double targetVelocity){//keep sign here
        return getVelocity() - targetVelocity;
    }

    public enum VelocityStatus{
        AtTarget
    }

    public enum LUTStatus{
        AtTarget,
        Cancelled
    }

}
