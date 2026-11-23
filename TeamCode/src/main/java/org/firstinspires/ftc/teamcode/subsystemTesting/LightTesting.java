package org.firstinspires.ftc.teamcode.subsystemTesting;


import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;

import org.firstinspires.ftc.teamcode.subsystems.GobildaLightOutputs;

import java.util.concurrent.atomic.AtomicInteger;

import dev.frozenmilk.dairy.mercurial.continuations.Continuations;
import dev.frozenmilk.dairy.mercurial.ftc.MercurialFTC;
import dev.frozenmilk.dairy.mercurial.processes.EventManager;

@Config
public class LightTesting {
    public static final MercurialFTC.RegisterableProgram lightsTesting = teleOp().withName("lights testing");
    public static final GobildaLightOutputs.ColorMode[] modes = new GobildaLightOutputs.ColorMode[] {
        GobildaLightOutputs.ColorMode.Black,
        GobildaLightOutputs.ColorMode.Red,
        GobildaLightOutputs.ColorMode.Orange,
        GobildaLightOutputs.ColorMode.Yellow,
        GobildaLightOutputs.ColorMode.Sage,
        GobildaLightOutputs.ColorMode.Green,
        GobildaLightOutputs.ColorMode.Azure,
        GobildaLightOutputs.ColorMode.Blue,
        GobildaLightOutputs.ColorMode.Indigo,
        GobildaLightOutputs.ColorMode.Violet,
        GobildaLightOutputs.ColorMode.White
    };
    public static MercurialFTC.RegisterableProgram teleOp() {
        return MercurialFTC.teleop((ctx) -> {
            GobildaLightOutputs outputs = new GobildaLightOutputs(ctx);
            AtomicInteger index = new AtomicInteger(0);
            Continuations.loop(
                Continuations.sequence(
                    Continuations.exec(() -> {
                        index.set((index.get() + 1) % modes.length);

                        TelemetryPacket packet = (new TelemetryPacket());
                        packet.put("light", modes[index.get()]);
                        FtcDashboard.getInstance().sendTelemetryPacket(packet);
                        outputs.setSolidColor(modes[index.get()]);
                    }),
                    Continuations.waitFor.bind(Continuations.seconds(.1))
                )
            ).spawnable().spawnLink();
            ctx.waitForStart();
            ctx.dropToScheduler();
            return null;
        });
    }
}
