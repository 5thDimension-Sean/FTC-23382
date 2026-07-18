package org.firstinspires.ftc.teamcode.pedroPathing.constants;

import com.pedropathing.ftc.localization.constants.TwoWheelConstants;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;

public class LConstants {
    public static final TwoWheelConstants twoWheelConstants = new TwoWheelConstants();

    static {
        twoWheelConstants.forwardTicksToInches = 0.001989436789;
        twoWheelConstants.strafeTicksToInches = 0.001989436789;
        twoWheelConstants.forwardPodY = 1;
        twoWheelConstants.strafePodX = -2.5;
        twoWheelConstants.IMU_HardwareMapName = "imu";
        twoWheelConstants.forwardEncoder_HardwareMapName = "fl";
        twoWheelConstants.strafeEncoder_HardwareMapName = "fr";
        twoWheelConstants.IMU_Orientation = new RevHubOrientationOnRobot(RevHubOrientationOnRobot.LogoFacingDirection.UP, RevHubOrientationOnRobot.UsbFacingDirection.FORWARD);
    }
}
