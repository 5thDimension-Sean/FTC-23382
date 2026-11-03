package org.firstinspires.ftc.teamcode.pedro.teleop;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class turret {
    private DcMotor turret;

    public void init(HardwareMap hardwareMap){
        turret = hardwareMap.get(DcMotor.class, "turrets");
    }
}
