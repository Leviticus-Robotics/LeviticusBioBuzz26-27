package org.firstinspires.ftc.teamcode.localization.mercurial;

import androidx.annotation.NonNull;

import com.pedropathing.localization.Localizer;
import com.pedropathing.localization.MotionState;
import com.pedropathing.math.Pose;
import com.pedropathing.math.Twist;
import com.pedropathing.math.Velocity;


import dev.frozenmilk.dairy.mercurial.processes.EventManager;

import java.util.Map;

public class MercurialLocalizer extends EventManager<MotionState> implements Localizer {
    private final Localizer localizer;
    private static final long CACHE_DURATION_NANOS = 18_000_000L; // 10 milliseconds
    private long nextNanos = System.nanoTime();

    public MercurialLocalizer(@NonNull Localizer localizer) {
        this.localizer = localizer;
    }

    // Matches Kotlin's @get:JvmName("localizer")
    @NonNull
    public Localizer localizer() {
        return localizer;
    }

    @Override
    public void eventSource() {
        long now = System.nanoTime();
        if (now >= nextNanos) {
            localizer.update();
            notify(localizer.state());
            nextNanos = System.nanoTime() + CACHE_DURATION_NANOS;
        }
    }

    @Override
    public void update() {
        localizer.update();
    }

    @Override
    public void reset() {

    }

    @NonNull
    @Override
    public MotionState state() {
        return localizer.state();
    }


    @Override
    public void setPose(@NonNull Pose pose) {
        localizer.setPose(pose);
    }

    @Override
    public void setX(double x) {
        localizer.setX(x);
    }

    @Override
    public void setY(double y) {
        localizer.setY(y);
    }

    @Override
    public void setHeading(double heading) {
        localizer.setHeading(heading);
    }

    @NonNull
    @Override
    public Pose pose() {
        return localizer.pose();
    }

    @NonNull
    @Override
    public Twist twist() {
        return localizer.twist();
    }

    @NonNull
    @Override
    public Velocity velocity() {
        return localizer.velocity();
    }

    @NonNull
    @Override
    public Map<String, Object> debug() {
        return localizer.debug();
    }
}