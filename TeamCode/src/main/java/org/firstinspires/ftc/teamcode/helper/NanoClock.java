package org.firstinspires.ftc.teamcode.helper;

public interface NanoClock {
    long now();

    NanoClock SYSTEM = System::nanoTime;
}
