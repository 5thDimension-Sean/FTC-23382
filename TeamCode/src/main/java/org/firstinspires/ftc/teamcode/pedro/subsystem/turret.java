package org.firstinspires.ftc.teamcode.pedro.subsystem;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PwmControl;
import com.qualcomm.robotcore.hardware.ServoImplEx;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.LLStatus;
import com.qualcomm.hardware.limelightvision.Limelight3A;


import org.firstinspires.ftc.teamcode.pedro.Constants;

public class turret {
    private DcMotorEx flywheel;
    public ServoImplEx linkage;
    public ServoImplEx turretServo;
    public ServoImplEx blocker;
    Limelight3A limelight;

    public void init(HardwareMap hardwareMap){
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.setPollRateHz(100);
        limelight.start();
        limelight.pipelineSwitch(0);
        flywheel = hardwareMap.get(DcMotorEx.class, Constants.TURRET_FLYWHEEL_MOTOR_NAME);
        linkage = hardwareMap.get(ServoImplEx.class, "linkage");
        linkage.setPwmRange(new PwmControl.PwmRange(500, 2500));
        turretServo = hardwareMap.get(ServoImplEx.class, "turretServo");
        turretServo.setPwmRange(new PwmControl.PwmRange(500, 2500));
        blocker = hardwareMap.get(ServoImplEx.class, "blocker");
        blocker.setPwmRange(new PwmControl.PwmRange(500, 2500));
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
     * @param position "open" (0.61.0) or "close" (0.0), case-insensitive.
     */
    public void moveLink(String position){
        if(position.equalsIgnoreCase("open")){
            linkage.setPosition(0.61);
        } else if(position.equalsIgnoreCase("close")){
            linkage.setPosition(0.0);
        }
    }
    /**
     * Moves the blocker servo to a predefined position.
     *
     * @param position "open" (0.47) or "close" (0.17), case-insensitive.
     */
    public void moveBlocker(String position){
        if(position.equalsIgnoreCase("open")){
            blocker.setPosition(0.17);
        } else if(position.equalsIgnoreCase("close")){
            blocker.setPosition(0.47);
        }
    }
    /**
     * Incrementally adjusts the linkage servo position by 0.01 per call.
     *
     * @param direction 0 to decrease position, 1 to increase position.
     */
    public void moveLinkManual(int direction){
        if(direction == 0){
            linkage.setPosition(linkage.getPosition()-0.005);
        }else if(direction == 1){
            linkage.setPosition(linkage.getPosition()+0.005);
        }
    }
    /**
     * Incrementally adjusts the turret servo position by 0.01 per call.
     *
     * @param direction 0 to go left, 1 to go right.
     */
    public void moveTurretManual(int direction){
        if(direction == 0){
            turretServo.setPosition(turretServo.getPosition()-0.005);
        }else if(direction == 1){
            turretServo.setPosition(turretServo.getPosition()+0.005);
        }
    }
    /**
     * Incrementally adjusts the blocker servo position by 0.01 per call.
     *
     * @param direction 0 to go left, 1 to go right.
     */
    public void moveBlockerManual(int direction){
        if(direction == 0){
            blocker.setPosition(blocker.getPosition()-0.005);
        }else if(direction == 1){
            blocker.setPosition(blocker.getPosition()+0.005);
        }
    }
}
