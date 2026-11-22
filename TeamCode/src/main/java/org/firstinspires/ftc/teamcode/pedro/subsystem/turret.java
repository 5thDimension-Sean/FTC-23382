package org.firstinspires.ftc.teamcode.pedro.subsystem;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class turret {
    private DcMotor turret;
    private final int robotX = 166; // in mm
    private final int robotY = 217; //in mm

    public void init(HardwareMap hardwareMap){
        turret = hardwareMap.get(DcMotor.class, "turret");
    }
    //get current position for odom and camera and maybe distance idk but montecarlo it
    //assuming currentPos = currentx, currenty.
    public void setPower(double power){
        turret.setPower(power);
    }

}
