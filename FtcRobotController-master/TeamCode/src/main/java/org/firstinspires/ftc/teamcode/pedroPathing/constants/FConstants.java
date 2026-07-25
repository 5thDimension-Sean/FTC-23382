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
    public static final PathConstraints pathConstraints = new PathConstraints(0.995, 0.1, 0.1, 0.007, 500, 0.1, 10, 0.1);

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

        // Translational PID settings
        followerConstants.coefficientsTranslationalPIDF.setCoefficients(0.08, 0, 0.02, 0);
        // Heading PID settings
        followerConstants.coefficientsHeadingPIDF.setCoefficients(1.5, 0, 0.2, 0);
        // Drive PID settings
        followerConstants.coefficientsDrivePIDF.setCoefficients(0.05, 0, 0.01, 0.6, 0);

        mecanumConstants.leftFrontMotorName = "frontleft";
        mecanumConstants.leftRearMotorName = "backleft";
        mecanumConstants.rightFrontMotorName = "frontright";
        mecanumConstants.rightRearMotorName = "backright";

        mecanumConstants.leftFrontMotorDirection = DcMotorSimple.Direction.FORWARD;
        mecanumConstants.leftRearMotorDirection = DcMotorSimple.Direction.FORWARD;
        mecanumConstants.rightFrontMotorDirection = DcMotorSimple.Direction.REVERSE;
        mecanumConstants.rightRearMotorDirection = DcMotorSimple.Direction.REVERSE;
    }
}
