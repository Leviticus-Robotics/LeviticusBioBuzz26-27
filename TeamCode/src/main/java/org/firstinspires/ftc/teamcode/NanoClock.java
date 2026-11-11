package org.firstinspires.ftc.teamcode;

public interface NanoClock {
    long now();

    NanoClock SYSTEM = System::nanoTime;
}
