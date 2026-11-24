package org.firstinspires.ftc.teamcode.subsystemTesting.turret;

import static dev.frozenmilk.dairy.mercurial.continuations.Continuations.exec;
import static dev.frozenmilk.dairy.mercurial.continuations.Continuations.loop;
import static dev.frozenmilk.dairy.mercurial.continuations.Continuations.seconds;
import static dev.frozenmilk.dairy.mercurial.continuations.Continuations.waitUntil;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.helper.AngleHelper;
import org.firstinspires.ftc.teamcode.sensor.TurretEncoder;

import java.util.concurrent.TimeUnit;

import dev.frozenmilk.dairy.mercurial.ftc.MercurialFTC;
import dev.frozenmilk.dairy.mercurial.processes.EventManager;


@Config
//seems to be bugging -> noisy readings seems that Mercurial not updating constantly
public class TurretEncoderTest {

    public static final MercurialFTC.RegisterableProgram TurretEncoderTest = teleOp()
            .withName("Turret Encoder Testing")
            .withGroup("Subsystem Testing");

    public static MercurialFTC.RegisterableProgram teleOp() {
        return MercurialFTC.teleop(ctx -> {
            Telemetry dashboardTelem = FtcDashboard.getInstance().getTelemetry();
            TurretEncoder turretEncoder = new TurretEncoder(ctx);
            Telemetry.Item absoluteRawEncoderPos = ctx.telemetry().addData("Absolute Raw: ", "unknown");
            Telemetry.Item incrementalRawEncoderPos = ctx.telemetry().addData("Incremental Raw: ", "unknown");
            Telemetry dashboardTelemetry = FtcDashboard.getInstance().getTelemetry();
            Telemetry.Item absoluteDashboard = dashboardTelemetry.addData("Absolute Raw: ", "unknown");
            Telemetry.Item incrementalDashboard = dashboardTelemetry.addData("Incremental Raw: ", "unknown");
            Telemetry.Item updateRate = ctx.telemetry().addData("Update Rate: ", "unknown");
            ElapsedTime timer = new ElapsedTime();
            double[] previousTime = {0};



            ctx.waitForStart();
            turretEncoder.addHandler(event -> {
                if(event.getAngleType() == AngleHelper.AngleType.Absolute){
                    absoluteRawEncoderPos.setValue(event.getAngle().toDegrees());
//                    absoluteDashboard.setValue(event.getAngle().toDegrees());
                }
                else if(event.getAngleType() == AngleHelper.AngleType.Incremental){
                    incrementalRawEncoderPos.setValue(event.getAngle().toDegrees());
//                    incrementalDashboard.setValue(event.getAngle().toDegrees());
                }
                return EventManager.EventHandled.ok();
            });




            //update  loop times
            loop(
                    exec(() -> {
//                        absoluteDashboard.setValue(turretEncoder.getRecentAbsolutePosition().toDegrees());
//                        incrementalDashboard.setValue(turretEncoder.getRecentIncrementalPosition().toDegrees());
//                        absoluteRawEncoderPos.setValue(turretEncoder.getRecentAbsolutePosition().toDegrees());
//                        incrementalRawEncoderPos.setValue(turretEncoder.getRecentIncrementalPosition().toDegrees());
                        dashboardTelemetry.addData("Difference (I-A): ", turretEncoder.recentIncrementalPosition.toDegrees() - turretEncoder.recentAbsolutePosition.toDegrees());
                        dashboardTelemetry.update();
                        double deltaTime = timer.now(TimeUnit.MILLISECONDS) - previousTime[0];
                        previousTime[0] = timer.now(TimeUnit.MILLISECONDS);
                        updateRate.setValue(1000.0 / deltaTime);
                    })
            ).spawnable().spawnLink();

            ctx.dropToScheduler();
            return null;

        });
    }




}
