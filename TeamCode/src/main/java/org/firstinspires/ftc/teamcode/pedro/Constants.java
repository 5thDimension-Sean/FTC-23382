package org.firstinspires.ftc.teamcode.pedro;

import com.pedropathing.algorithm.Foresight;
import com.pedropathing.algorithm.ForesightConfig;
import com.pedropathing.controllers.Controller;
import com.pedropathing.follower.Follower;
import com.pedropathing.math.Matrix;
import com.pedropathing.math.Vector2D;
import com.pedropathing.revhub.drivetrains.Mecanum;
import com.pedropathing.revhub.drivetrains.MecanumConfig;
import com.pedropathing.revhub.localizers.PinpointConfig;
import com.pedropathing.revhub.localizers.PinpointLocalizer;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public class Constants {
    public static final double pollenWeight = 24.95; //weight in g
    public static final double pollenDiameter = 2.8; //length in cm
    public static final double nectarWeight = 41.28; //weight in g
    public static final double nectarDiameter = 3.6; //length in cm

    public static final String TURRET_FLYWHEEL_MOTOR_NAME = "turret";
    public static final int TURRET_FLYWHEEL_TICKS_PER_REVOLUTION = 28; // goBILDA 5203-2402-0001
    public static final double TURRET_FLYWHEEL_MAX_TEST_RPM = 5000.0;

    public static final double TURRET_FLYWHEEL_KP = 11.0;
    public static final double TURRET_FLYWHEEL_KI = 1;
    public static final double TURRET_FLYWHEEL_KD = 1.5;
    public static final double TURRET_FLYWHEEL_KF = 11.0;


    public static Follower create(HardwareMap h) {
        return new Follower(
                new PinpointLocalizer(h, localizerConfig),
                new Mecanum(h, drivetrainConfig),
                new Foresight(foresightConfig)
        );
    }

    public static MecanumConfig drivetrainConfig = new MecanumConfig(c -> {
        c.frontLeftName.set("frontleft");
        c.frontRightName.set("frontright");
        c.backLeftName.set("backleft");
        c.backRightName.set("backright");
        c.frontLeftDirection.set(DcMotorSimple.Direction.REVERSE);
        c.frontRightDirection.set(DcMotorSimple.Direction.FORWARD);
        c.backLeftDirection.set(DcMotorSimple.Direction.REVERSE);
        c.backRightDirection.set(DcMotorSimple.Direction.FORWARD);
    });
    //fix
    public static PinpointConfig localizerConfig = new PinpointConfig(c -> {
        c.name.set("pinpoint");
        c.podType.set(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        c.xPodOffset.set(-3.25492126);
        c.yPodOffset.set(-4.293885);
        c.xPodDirection.set(GoBildaPinpointDriver.EncoderDirection.FORWARD);
        c.yPodDirection.set(GoBildaPinpointDriver.EncoderDirection.REVERSED);
        c.globalDistanceUnit.set(DistanceUnit.INCH);
        c.offsetUnits.set(DistanceUnit.INCH);
    });
    //fix
    public static ForesightConfig foresightConfig = new ForesightConfig(
            c -> {
                Controller primaryTranslationalForward = Controller.proportional(0.26430523490123387);
                Controller secondaryTranslationalForward = Controller.proportional(0.0976537219300599);
                Controller primaryTranslationalLateral = Controller.proportional(0.3925327717327282);
                Controller secondaryTranslationalLateral = Controller.proportional(0.14503037048640982);

                c.forwardTranslational.set(Controller.piecewise(secondaryTranslationalForward).put(2.5, primaryTranslationalForward));
                c.strafeTranslational.set(Controller.piecewise(secondaryTranslationalLateral).put(2.5, primaryTranslationalLateral));

                c.coast.set(Controller.proportionalFeedforward(0.01363369546157871));
                c.brake.set(Controller.proportionalFeedforward(0.011588641142341904));

                c.headingFeedback.set(Controller.proportional(4.22288438940617));
                c.headingBrakeCoefficients.set(Vector2D.cartesian(0.05623572299045145, 0.003062585087449841));

                c.linearBrakeCoefficients.set(Matrix.diag(0.02777853780019659, 0.029531157290356956));
                c.quadraticBrakeCoefficients.set(Matrix.diag(-2.5435190970420963E-4, -1.527781802567794E-4));

                c.maxAchievableForwardVelocity.set(77.0247814319019);
                c.maxAchievableStrafeVelocity.set(63.726675934398486);
                c.naturalForwardDeceleration.set(34.56646864170023);
                c.naturalStrafeDeceleration.set(57.37080165515346);
            }
    );
}
