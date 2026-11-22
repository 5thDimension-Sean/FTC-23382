package org.firstinspires.ftc.teamcode.pedro.subsystem;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PwmControl;
import com.qualcomm.robotcore.hardware.ServoImplEx;

import org.firstinspires.ftc.teamcode.pedro.Constants;

public class turret {
    private DcMotorEx flywheel;
    private ServoImplEx linkage;

    public void init(HardwareMap hardwareMap){
        flywheel = hardwareMap.get(DcMotorEx.class, Constants.TURRET_FLYWHEEL_MOTOR_NAME);
        linkage = hardwareMap.get(ServoImplEx.class, "linkage");
        linkage.setPwmRange(new PwmControl.PwmRange(500, 2500));

        flywheel.setPower(0);
        flywheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        flywheel.setVelocityPIDFCoefficients(
                Constants.TURRET_FLYWHEEL_KP,
                Constants.TURRET_FLYWHEEL_KI,
                Constants.TURRET_FLYWHEEL_KD,
                Constants.TURRET_FLYWHEEL_KF
        );
    }

    /**
     * Sets the flywheel's target velocity in encoder ticks per second.
     *
     * @param ticksPerSecond Signed target velocity. Use 0 to stop the flywheel.
     */
    public void setFlywheelVelocity(double ticksPerSecond) {
        flywheel.setVelocity(ticksPerSecond);
    }

    /**
     * Moves the linkage servo to a predefined position.
     *
     * @param position "open" (1.0) or "close" (0.0), case-insensitive.
     */
    public void moveLoader(String position){
        if(position.equalsIgnoreCase("open")){
            linkage.setPosition(1.0);
        } else if(position.equalsIgnoreCase("close")){
            linkage.setPosition(0.0);
        }
    }
    /**
     * Incrementally adjusts the linkage servo position by 0.01 per call.
     *
     * @param direction 0 to decrease position, 1 to increase position.
     */
    public void moveLoaderManual(int direction){
        if(direction == 0){
            linkage.setPosition(linkage.getPosition()-0.01);
        }else if(direction == 1){
            linkage.setPosition(linkage.getPosition()+0.01);
        }
    }

}
