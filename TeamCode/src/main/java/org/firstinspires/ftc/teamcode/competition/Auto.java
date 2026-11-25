package org.firstinspires.ftc.teamcode.competition;

import org.firstinspires.ftc.teamcode.robot.FullSystemsTeleOp;

import dev.frozenmilk.dairy.mercurial.ftc.MercurialFTC;

public class Auto {
    public static final MercurialFTC.RegisterableProgram auto = MercurialFTC.autonomous(ctx -> {

        return FullSystemsTeleOp.teleOp(null, null, null); // TODO add coords
    });
}
