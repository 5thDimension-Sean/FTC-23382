package org.firstinspires.ftc.teamcode.RobotCode.constants;

import com.acmerobotics.dashboard.config.Config;
import com.pedropathing.follower.Follower;
import com.pedropathing.follower.FollowerConstants;
import com.pedropathing.ftc.FollowerBuilder;
import com.pedropathing.ftc.drivetrains.MecanumConstants;
import com.pedropathing.paths.PathConstraints;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

@Config
public class FConstants {
    public static double mass                         = 8;
    public static double forwardZeroPowerAcceleration = -30.01;
    public static double lateralZeroPowerAcceleration = -85.11;

    public static double translationalP = 0.1;
    public static double translationalI = 0;
    public static double translationalD = 0;
    public static double translationalF = 0;

    public static double headingP = 1.5;
    public static double headingI = 0;
    public static double headingD = 0;
    public static double headingF = 0;

    public static double driveP = 0.02;
    public static double driveI = 0;
    public static double driveD = 0;
    public static double driveF = 0;

    public static Follower createFollower(HardwareMap hardwareMap) {
        FollowerConstants fc = new FollowerConstants();
        fc.mass = mass;
        fc.forwardZeroPowerAcceleration = forwardZeroPowerAcceleration;
        fc.lateralZeroPowerAcceleration = lateralZeroPowerAcceleration;
        fc.coefficientsTranslationalPIDF.setCoefficients(translationalP, translationalI, translationalD, translationalF);
        fc.coefficientsHeadingPIDF.setCoefficients(headingP, headingI, headingD, headingF);
        fc.coefficientsDrivePIDF.setCoefficients(driveP, driveI, driveD, driveF, 0);

        MecanumConstants mc = new MecanumConstants();
        mc.leftFrontMotorName  = "frontleft";
        mc.leftRearMotorName   = "backleft";
        mc.rightFrontMotorName = "frontright";
        mc.rightRearMotorName  = "backright";
        mc.leftFrontMotorDirection  = DcMotorSimple.Direction.REVERSE;
        mc.leftRearMotorDirection   = DcMotorSimple.Direction.REVERSE;
        mc.rightFrontMotorDirection = DcMotorSimple.Direction.FORWARD;
        mc.rightRearMotorDirection  = DcMotorSimple.Direction.FORWARD;

        return new FollowerBuilder(fc, hardwareMap)
                .pathConstraints(new PathConstraints(0.995, 25, 15, 1.5, 500, 0.4, 10, 0.1))
                .mecanumDrivetrain(mc)
                .driveEncoderLocalizer(LConstants.build())
                .build();
    }
}
