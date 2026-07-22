package org.firstinspires.ftc.teamcode.pedroPathing.constants;

import com.pedropathing.ftc.localization.constants.DriveEncoderConstants;

public class LConstants {
    public static final DriveEncoderConstants driveEncoderConstants = new DriveEncoderConstants();

    static {
        driveEncoderConstants.leftFrontMotorName = "frontleft";
        driveEncoderConstants.leftRearMotorName = "backleft";
        driveEncoderConstants.rightFrontMotorName = "frontright";
        driveEncoderConstants.rightRearMotorName = "backright";

        // You may need to tune these for your robot
        driveEncoderConstants.forwardTicksToInches = 0.001989436789;
        driveEncoderConstants.strafeTicksToInches = 0.001989436789;
        driveEncoderConstants.turnTicksToInches = 0.001989436789;

        driveEncoderConstants.robot_Width = 18;
        driveEncoderConstants.robot_Length = 18;
    }
}
