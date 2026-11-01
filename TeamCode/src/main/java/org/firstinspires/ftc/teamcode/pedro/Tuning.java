package org.firstinspires.ftc.teamcode.pedro;

import com.pedropathing.tuning.autotune.Procedure;
import com.pedropathing.tuning.autotune.Tuner;

import org.firstinspires.ftc.teamcode.pedro.procedures.MecanumTuner;

public class Tuning {
    //comment to test
    //my limelight
    @Tuner
    public static Procedure mecanumTuner() {
        return new MecanumTuner();
    }
}
