package org.firstinspires.ftc.teamcode.pedroPathing.constants;

import com.pedropathing.ftc.localization.Encoder;
import com.pedropathing.ftc.localization.constants.DriveEncoderConstants;

public class LConstants {
    public static final DriveEncoderConstants driveEncoderConstants = new DriveEncoderConstants();

    static {
        driveEncoderConstants.leftFrontMotorName = "frontleft";
        driveEncoderConstants.leftRearMotorName = "backleft";
        driveEncoderConstants.rightFrontMotorName = "frontright";
        driveEncoderConstants.rightRearMotorName = "backright";

        // You may need to tune these for your robot
        driveEncoderConstants.forwardTicksToInches = 0.0110412;
        driveEncoderConstants.strafeTicksToInches = 0.0110412;
        driveEncoderConstants.turnTicksToInches = 0.0110412;
        driveEncoderConstants.robot_Width = 15.5;
        driveEncoderConstants.robot_Length = 15.5;

        // Encoder directions
        driveEncoderConstants.leftFrontEncoderDirection = Encoder.REVERSE;
        driveEncoderConstants.rightFrontEncoderDirection = Encoder.FORWARD;
        driveEncoderConstants.leftRearEncoderDirection = Encoder.REVERSE;
        driveEncoderConstants.rightRearEncoderDirection = Encoder.FORWARD;
    }
}
