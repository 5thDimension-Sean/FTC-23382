package org.firstinspires.ftc.teamcode.pedro.teleop;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.pedro.Constants;

/**
 * Turret motor. Angle 0 = pointing at the robot's front, positive = counter-clockwise, in radians.
 * The encoder is never reset here, so the angle carries over from auto to teleop as long as the
 * robot stays powered. Encoder 0 must be "pointing forward" when the robot is switched on.
 */
public class turret {
    private DcMotor turret;

    public void init(HardwareMap hardwareMap){
        turret = hardwareMap.get(DcMotor.class, "turrets");
    }

    private static double ticksPerRadian() {
        return Constants.turretTicksPerRev / (2 * Math.PI);
    }

    /** Current turret angle in radians. */
    public double getAngle() {
        return turret.getCurrentPosition() / ticksPerRadian();
    }

    /** Drives the turret to a robot-relative angle (radians), never past the turret limits. */
    public void aim(double angleRad) {
        double min = Math.toRadians(Constants.turretMinAngleDeg);
        double max = Math.toRadians(Constants.turretMaxAngleDeg);
        double clamped = Math.max(min, Math.min(max, angleRad));
        turret.setTargetPosition((int) Math.round(clamped * ticksPerRadian()));
        turret.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        turret.setPower(Constants.turretPower);
    }

    /** Stops driving the turret. */
    public void release() {
        turret.setPower(0);
        turret.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }
}
