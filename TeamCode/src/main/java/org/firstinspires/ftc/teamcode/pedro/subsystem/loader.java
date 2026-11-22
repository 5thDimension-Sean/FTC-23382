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

    /**
     * Moves both loader servos to a predefined position.
     *
     * @param position "open" (1.0) or "close" (0.0), case-insensitive.
     */
    public void moveLoader(String position){
        if(position.equalsIgnoreCase("open")){
            loaderLeft.setPosition(1.0);
            loaderLeft.setPosition(1.0);
        } else if(position.equalsIgnoreCase("close")){
            loaderLeft.setPosition(0.0);
            loaderLeft.setPosition(0.0);
        }
    }
    /**
     * Incrementally adjusts both loader servos by 0.01 per call.
     *
     * @param direction 0 to decrease position(close), 1 to increase position(open).
     */
    public void moveLoaderManual(int direction){
        if(direction == 0){
            loaderLeft.setPosition(loaderLeft.getPosition()-0.01);
            loaderRight.setPosition(loaderRight.getPosition()-0.01);
        }else if(direction == 1){
            loaderLeft.setPosition(loaderLeft.getPosition()+0.01);
            loaderRight.setPosition(loaderRight.getPosition()+0.01);
        }
    }


}
