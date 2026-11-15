package org.firstinspires.ftc.teamcode.pedro.teleop;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class turret {
    private DcMotor turret;
    private final int robotX = 166; // in mm
    private final int robotY = 217; //in mm

    public void init(HardwareMap hardwareMap){
        turret = hardwareMap.get(DcMotor.class, "turrets");
    }
    //get current position for odom and camera and maybe distance idk but montecarlo it
    //assuming currentPos = currentx, currenty.
    private int currentX = 2123;
    private int currentY = 12049;

}
