package org.firstinspires.ftc.teamcode.pedro.subsystem;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.pedro.Constants;

public class intake {
    private DcMotor intakeLeft;
    private DcMotor intakeRight;
    private double currentPower = 0.0;
    private long lastUpdateTime = System.nanoTime();

    // Maximum power change per second
    private static final double RAMP_RATE = 6.5;

    public void init(HardwareMap hardwareMap){
        intakeLeft = hardwareMap.get(DcMotor.class, Constants.INTAKE_LEFT_MOTOR_NAME);
        intakeRight = hardwareMap.get(DcMotor.class, Constants.INTAKE_RIGHT_MOTOR_NAME);
        intakeLeft.setDirection(DcMotor.Direction.REVERSE);
    }

    /**
     * Gradually adjusts both intake motors toward the target power.
     * Must be called repeatedly inside the TeleOp loop.
     *
     * @param targetPower Desired motor power from -1.0 to 1.0.
     */

    public void setPowerGradual(double targetPower) {
        targetPower = Math.max(-1.0, Math.min(1.0, targetPower));

        long now = System.nanoTime();
        double dt = (now - lastUpdateTime) / 1e9;
        lastUpdateTime = now;

        double maxChange = RAMP_RATE * dt;
        double difference = targetPower - currentPower;

        if (Math.abs(difference) <= maxChange) {
            currentPower = targetPower;
        } else {
            currentPower += Math.copySign(maxChange, difference);
        }

        intakeLeft.setPower(currentPower);
        intakeRight.setPower(currentPower);
    }


}
