package org.firstinspires.ftc.teamcode.RobotCode.constants;

import com.acmerobotics.dashboard.config.Config;
import com.pedropathing.ftc.localization.Encoder;
import com.pedropathing.ftc.localization.constants.DriveEncoderConstants;

@Config
public class LConstants {
    public static double forwardTicksToInches = 0.0110412;
    public static double strafeTicksToInches  = 0.01656;
    public static double turnTicksToInches    = 0.01243;
    public static double robotWidth           = 15.5;
    public static double robotLength          = 15.5;

    public static final DriveEncoderConstants driveEncoderConstants = new DriveEncoderConstants();

    static {
        driveEncoderConstants.leftFrontMotorName  = "frontleft";
        driveEncoderConstants.leftRearMotorName   = "backleft";
        driveEncoderConstants.rightFrontMotorName = "frontright";
        driveEncoderConstants.rightRearMotorName  = "backright";

        driveEncoderConstants.leftFrontEncoderDirection  = Encoder.FORWARD;
        driveEncoderConstants.rightFrontEncoderDirection = Encoder.FORWARD;
        driveEncoderConstants.leftRearEncoderDirection   = Encoder.FORWARD;
        driveEncoderConstants.rightRearEncoderDirection  = Encoder.FORWARD;
    }

    static DriveEncoderConstants build() {
        driveEncoderConstants.forwardTicksToInches = forwardTicksToInches;
        driveEncoderConstants.strafeTicksToInches  = strafeTicksToInches;
        driveEncoderConstants.turnTicksToInches    = turnTicksToInches;
        driveEncoderConstants.robot_Width          = robotWidth;
        driveEncoderConstants.robot_Length         = robotLength;
        return driveEncoderConstants;
    }
}
