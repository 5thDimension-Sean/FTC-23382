package org.firstinspires.ftc.teamcode.pedro.subsystem;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoImplEx;

public class turret {
    private DcMotor turret;
    private ServoImplEx linkage;
    private final int robotX = 166; // in mm
    private final int robotY = 217; //in mm

    public void init(HardwareMap hardwareMap){
        turret = hardwareMap.get(DcMotor.class, "turret");
        linkage = hardwareMap.get(ServoImplEx.class, "linkage");
    }
    //get current position for odom and camera and maybe distance idk but montecarlo it
    //assuming currentPos = currentx, currenty.
    public void setPower(double power){
        turret.setPower(power);
    }

    public void moveLoader(String position){
        if(position.equalsIgnoreCase("open")){
            linkage.setPosition(1.0);
        } else if(position.equalsIgnoreCase("close")){
            linkage.setPosition(0.0);
        }
    }

    public void moveLoaderManual(int direction){
        if(direction == 0){
            linkage.setPosition(linkage.getPosition()-0.01);
        }else if(direction == 1){
            linkage.setPosition(linkage.getPosition()+0.01);
        }
    }

}
