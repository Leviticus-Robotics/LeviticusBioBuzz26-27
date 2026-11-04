package org.firstinspires.ftc.teamcode.subsystems;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.constants.ConfigNames;
import org.firstinspires.ftc.teamcode.hardware.MotorEx;

import dev.frozenmilk.dairy.mercurial.ftc.Context;
import dev.frozenmilk.dairy.mercurial.processes.Channel;
import dev.frozenmilk.dairy.mercurial.processes.StateMachine;
import dev.frozenmilk.dairy.mercurial.processes.SwitchedStateMachine;

@Config
public class Transfer extends MotorSubsystem {
    public static double[] pidCoefficients = new double[] {1.0, 0.0, 0.0};
    public static double[] feedforwardCoefficients = new double[]{0, 0, 0};
    public static boolean usePIDCoefficients = false;
    public static boolean useFeedforwardCoefficients = false;

    public Transfer(Context ctx){
        stateTelemetry = ctx.telemetry().addData("Transfer/State", "unknown");
        motor = new MotorEx(ctx.hardwareMap(), ConfigNames.intake);
        motor.motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motor.setInverted(true);
        if(usePIDCoefficients){
            motor.setVeloCoefficients(pidCoefficients[0], pidCoefficients[1], pidCoefficients[2]);
        }
        if(useFeedforwardCoefficients){
            motor.setFeedforwardCoefficients(feedforwardCoefficients[0], feedforwardCoefficients[1], feedforwardCoefficients[2]);
        }
        offMode = entranceMode(from ->{
            motor.setRunMode(MotorEx.RunMode.RawPower);
            motor.set(0);
            stateTelemetry.setValue("Off");
        });

    }

}
