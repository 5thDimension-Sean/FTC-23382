package org.firstinspires.ftc.teamcode.pedro.subsystem;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class intake {
    private DcMotor intakeleft;
    private DcMotor intakeright;

    public void init(HardwareMap hardwareMap){
        intakeleft = hardwareMap.get(DcMotor.class, "intakeleft");
        intakeright = hardwareMap.get(DcMotor.class, "intakeright");
    }

    public void intakePower(double power){
        intakeleft.setPower(power);
        intakeright.setPower(power);
    }


}
