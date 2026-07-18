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

    public static FollowerConstants followerWeight = new FollowerConstants()
            .mass(5);
    public static MecanumConstants driveConstants = new MecanumConstants()
            .maxPower(1)
            .rightFrontMotorName("fr")
            .rightRearMotorName("br")
            .leftRearMotorName("bl")
            .leftFrontMotorName("fl")
            .leftFrontMotorDirection(DcMotorSimple.Direction.FORWARD)
            .leftRearMotorDirection(DcMotorSimple.Direction.FORWARD)
            .rightFrontMotorDirection(DcMotorSimple.Direction.REVERSE)
            .rightRearMotorDirection(DcMotorSimple.Direction.REVERSE);

    public static Follower createFollower(HardwareMap hardwareMap) {
        return new FollowerBuilder(followerConstants, hardwareMap)
                .pathConstraints(pathConstraints)
                .mecanumDrivetrain(driveConstants)
                .build();
    }
    static {
        followerConstants.mass = 12.0;
        followerConstants.forwardZeroPowerAcceleration = -30.01;
        followerConstants.lateralZeroPowerAcceleration = -85.11;

        mecanumConstants.leftFrontMotorName = "fl";
        mecanumConstants.leftRearMotorName = "bl";
        mecanumConstants.rightFrontMotorName = "fr";
        mecanumConstants.rightRearMotorName = "br";

        mecanumConstants.leftFrontMotorDirection = DcMotorSimple.Direction.FORWARD;
        mecanumConstants.leftRearMotorDirection = DcMotorSimple.Direction.FORWARD;
        mecanumConstants.rightFrontMotorDirection = DcMotorSimple.Direction.REVERSE;
        mecanumConstants.rightRearMotorDirection = DcMotorSimple.Direction.REVERSE;
    }
}
