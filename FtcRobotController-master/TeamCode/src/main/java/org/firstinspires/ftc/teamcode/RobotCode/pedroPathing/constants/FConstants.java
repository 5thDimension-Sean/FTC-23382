package org.firstinspires.ftc.teamcode.pedroPathing.constants;

import com.pedropathing.follower.Follower;
import com.pedropathing.follower.FollowerConstants;
import com.pedropathing.ftc.FollowerBuilder;
import com.pedropathing.ftc.drivetrains.MecanumConstants;
import com.pedropathing.paths.PathConstraints;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class FConstants {
    public static final FollowerConstants followerConstants = new FollowerConstants();
    public static final MecanumConstants mecanumConstants = new MecanumConstants();
    public static final PathConstraints pathConstraints = new PathConstraints(0.995, 25, 15, 1.5, 500, 0.4, 10, 0.1);

    public static Follower createFollower(HardwareMap hardwareMap) {
        return new FollowerBuilder(followerConstants, hardwareMap)
                .pathConstraints(pathConstraints)
                .mecanumDrivetrain(mecanumConstants)
                .driveEncoderLocalizer(LConstants.driveEncoderConstants)
                .build();
    }

    static {
        followerConstants.mass = 8;
        followerConstants.forwardZeroPowerAcceleration = -30.01;
        followerConstants.lateralZeroPowerAcceleration = -85.11;

        followerConstants.coefficientsTranslationalPIDF.setCoefficients(0.1, 0, 0, 0);
        followerConstants.coefficientsHeadingPIDF.setCoefficients(1.5, 0, 0, 0);
// Drive: Low P and low F to prevent runaway movement
        followerConstants.coefficientsDrivePIDF.setCoefficients(0.02, 0, 0, 0, 0);

        mecanumConstants.leftFrontMotorName = "frontleft";
        mecanumConstants.leftRearMotorName = "backleft";
        mecanumConstants.rightFrontMotorName = "frontright";
        mecanumConstants.rightRearMotorName = "backright";

        mecanumConstants.leftFrontMotorDirection = DcMotorSimple.Direction.REVERSE;
        mecanumConstants.leftRearMotorDirection = DcMotorSimple.Direction.REVERSE;
        mecanumConstants.rightFrontMotorDirection = DcMotorSimple.Direction.FORWARD;
        mecanumConstants.rightRearMotorDirection = DcMotorSimple.Direction.FORWARD;

    }
}
