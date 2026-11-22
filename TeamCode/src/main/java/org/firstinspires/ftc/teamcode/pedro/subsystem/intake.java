package org.firstinspires.ftc.teamcode.pedro.subsystem;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class intake {
    private DcMotor intakeLeft;
    private DcMotor intakeRight;

    public void init(HardwareMap hardwareMap){
        intakeLeft = hardwareMap.get(DcMotor.class, "intakeLeft");
        intakeRight = hardwareMap.get(DcMotor.class, "intakeRight");
    }

    public void intakePower(double power){
        intakeLeft.setPower(power);
        intakeRight.setPower(power);
    }


}
