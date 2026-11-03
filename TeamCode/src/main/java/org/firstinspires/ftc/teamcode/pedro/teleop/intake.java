package org.firstinspires.ftc.teamcode.pedro.teleop;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class intake {
    private DcMotor intake;

    public void init(HardwareMap hardwareMap){
        intake = hardwareMap.get(DcMotor.class, "intake");
    }

    public void intakePower(double power){
        intake.setPower(power);
    }


}
