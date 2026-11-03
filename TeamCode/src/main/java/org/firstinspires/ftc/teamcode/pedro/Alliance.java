package org.firstinspires.ftc.teamcode.pedro;

import com.pedropathing.math.Pose;

public enum Alliance {
    RED, BLUE;

    /** Position of this alliance's hive (goal) on the field. */
    public Pose hive() {
        return this == RED ? Constants.redHive : Constants.blueHive;
    }

    public Alliance other() {
        return this == RED ? BLUE : RED;
    }
}
