package org.firstinspires.ftc.teamcode.subsystems;

import static dev.frozenmilk.dairy.mercurial.continuations.Continuations.loop;

import com.qualcomm.hardware.lynx.LynxModule;

import java.util.List;

import dev.frozenmilk.dairy.mercurial.ftc.Context;
import dev.frozenmilk.dairy.mercurial.processes.EventManager;

public class BulkCacheDaemon extends EventManager<BulkCacheDaemon.Event> {
    public enum Event {
        Cleared
    }

    private final List<LynxModule> modules;
    private final static long CACHE_DURATION = 10_000_000;
    private long next = System.nanoTime();

    public BulkCacheDaemon(Context ctx) {
        modules = ctx.hardwareMap().getAll(LynxModule.class);
        // enable bulk caching
        modules.forEach(m -> m.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL));
    }

    @Override
    protected void eventSource() {
        if (System.nanoTime() > next) {
            modules.forEach(LynxModule::clearBulkCache);
            next = System.nanoTime() + CACHE_DURATION;
            notify(Event.Cleared);
        }
    }
}