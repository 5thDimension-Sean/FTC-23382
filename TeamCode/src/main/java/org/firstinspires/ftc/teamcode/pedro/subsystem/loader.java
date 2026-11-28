package org.firstinspires.ftc.teamcode.pedro.subsystem;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PwmControl;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoImplEx;

import org.firstinspires.ftc.teamcode.pedro.Constants;

public class loader {
    public ServoImplEx loaderLeft;
    public ServoImplEx loaderRight;

    public void init(HardwareMap hardwareMap){
        loaderLeft = hardwareMap.get(ServoImplEx.class, Constants.LOADER_LEFT_SERVO_NAME);
        loaderRight = hardwareMap.get(ServoImplEx.class, Constants.LOADER_RIGHT_SERVO_NAME);
        loaderLeft.setPwmRange(new PwmControl.PwmRange(500, 2500));
        loaderRight.setPwmRange(new PwmControl.PwmRange(500, 2500));
    }

    /**
     * Moves both loader servos to a predefined position.
     *
     * @param position "up" (left 1.0, right 0.0) or "down" (left 0.0, right 1.0), case-insensitive.
     */
    public void position(String position){
        if(position.equalsIgnoreCase("up") || position.equalsIgnoreCase("open")){
            loaderLeft.setPosition(1.0);
            loaderRight.setPosition(0.0);
        } else if(position.equalsIgnoreCase("down") || position.equalsIgnoreCase("close")){
            loaderLeft.setPosition(0.0);
            loaderRight.setPosition(1.0);
        }
    }
    /**
     * Incrementally adjusts both loader servos by 0.01 per call.
     *
     * @param direction 0 to decrease position(close), 1 to increase position(open).
     */
    public void positionManual(int direction){
        if(direction == 0){
            loaderLeft.setPosition(loaderLeft.getPosition()-0.01);
            loaderRight.setPosition(loaderRight.getPosition()+0.01);
        }else if(direction == 1){
            loaderLeft.setPosition(loaderLeft.getPosition()+0.01);
            loaderRight.setPosition(loaderRight.getPosition()-0.01);
        }
    }


}
