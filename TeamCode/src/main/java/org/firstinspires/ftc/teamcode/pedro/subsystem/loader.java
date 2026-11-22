package org.firstinspires.ftc.teamcode.pedro.subsystem;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PwmControl;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoImplEx;

public class loader {
    private ServoImplEx loaderLeft;
    private ServoImplEx loaderRight;

    public void init(HardwareMap hardwareMap){
        loaderLeft = hardwareMap.get(ServoImplEx.class, "loaderLeft");
        loaderRight = hardwareMap.get(ServoImplEx.class, "loaderRight");
        loaderLeft.setPwmRange(new PwmControl.PwmRange(500, 2500));
        loaderRight.setPwmRange(new PwmControl.PwmRange(500, 2500));
    }

    public void servo(String position){
        if(position.equalsIgnoreCase("open")){
            loaderLeft.setPosition(0.4);
            loaderLeft.setPosition
        } else if(position.equalsIgnoreCase("close")){

        }
    }


}
