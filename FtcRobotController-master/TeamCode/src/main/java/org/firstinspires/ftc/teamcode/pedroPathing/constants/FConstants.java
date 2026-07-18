package org.firstinspires.ftc.teamcode.pedroPathing.constants;

import com.pedropathing.follower.FollowerConstants;
import com.pedropathing.ftc.drivetrains.MecanumConstants;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

public class FConstants {
    public static final FollowerConstants followerConstants = new FollowerConstants();
    public static final MecanumConstants mecanumConstants = new MecanumConstants();

    static {
        followerConstants.mass = 12.0;
        followerConstants.forwardZeroPowerAcceleration = -30.01;
        followerConstants.lateralZeroPowerAcceleration = -85.11;

        mecanumConstants.leftFrontMotorName = "frontLeftMotor";
        mecanumConstants.leftRearMotorName = "backLeftMotor";
        mecanumConstants.rightFrontMotorName = "frontRightMotor";
        mecanumConstants.rightRearMotorName = "backRightMotor";

        mecanumConstants.leftFrontMotorDirection = DcMotorSimple.Direction.FORWARD;
        mecanumConstants.leftRearMotorDirection = DcMotorSimple.Direction.FORWARD;
        mecanumConstants.rightFrontMotorDirection = DcMotorSimple.Direction.REVERSE;
        mecanumConstants.rightRearMotorDirection = DcMotorSimple.Direction.REVERSE;
    }
}
