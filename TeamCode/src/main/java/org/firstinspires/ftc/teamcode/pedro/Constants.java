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
    //the x pod might be opposite(5.25, -7.5)
    public static PinpointConfig localizerConfig = new PinpointConfig(c -> {
        c.name.set("pinpoint");
        c.podType.set(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        c.xPodOffset.set(1.6529030687227024);
        c.yPodOffset.set(1.7600048996332123);
        c.xPodDirection.set(GoBildaPinpointDriver.EncoderDirection.REVERSED);
        c.yPodDirection.set(GoBildaPinpointDriver.EncoderDirection.REVERSED);
        c.globalDistanceUnit.set(DistanceUnit.INCH);
        c.offsetUnits.set(DistanceUnit.INCH);
    });

    public static ForesightConfig foresightConfig = new ForesightConfig(
            c -> {
                Controller primaryTranslationalForward = Controller.proportional(0.15771804606990475);
                Controller secondaryTranslationalForward = Controller.proportional(0.05827260371902287);
                Controller primaryTranslationalLateral = Controller.proportional(0.22916847431114615);
                Controller secondaryTranslationalLateral = Controller.proportional(0.0846716277635569);

                c.forwardTranslational.set(Controller.piecewise(secondaryTranslationalForward).put(2.5, primaryTranslationalForward));
                c.strafeTranslational.set(Controller.piecewise(secondaryTranslationalLateral).put(2.5, primaryTranslationalLateral));

                c.coast.set(Controller.proportionalFeedforward(0.01836056929454132));
                c.brake.set(Controller.proportionalFeedforward(0.01560648390036012));

                c.headingFeedback.set(Controller.proportional(3.7869369438988136));
                c.headingBrakeCoefficients.set(Vector2D.cartesian(0.03927541753292378, 0.005324075370555076));

                c.linearBrakeCoefficients.set(Matrix.diag(0.04015432840316273, 0.046300909542618895));
                c.quadraticBrakeCoefficients.set(Matrix.diag(0.002125396116917167, 0.001804057208990252));

                c.maxAchievableForwardVelocity.set(57.0670999910702);
                c.maxAchievableStrafeVelocity.set(47.55022328872176);
                c.naturalForwardDeceleration.set(35.323920889028045);
                c.naturalStrafeDeceleration.set(57.24624165886486);
            }
    );
}