package org.firstinspires.ftc.teamcode.pedro;

import com.pedropathing.algorithm.Foresight;
import com.pedropathing.algorithm.ForesightConfig;
import com.pedropathing.controllers.Controller;
import com.pedropathing.api.PoseFactory;
import com.pedropathing.follower.Follower;
import com.pedropathing.math.Matrix;
import com.pedropathing.math.Pose;
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
    public static final String colorLeft = "BLUE";
    public static final String colorRight = "RED";
    public static String selectedTeam = "UNSELECTED";

    /** Blue side tracks the blue AprilTags (IDs 38-45); red side tracks the red tags (IDs 30-37). */
    public static boolean isBlue() {
        return colorLeft.equals(selectedTeam);
    }

    // ---- Turret aiming (field frame: Pedro, inches, origin at a field corner, 144 x 144) ----
    // goBILDA 5-turn servo = 1800 deg for 0..1 at 500-2500 us; it drives the SMALL gear of the
    // 2.75:1 turret, so the turret turns 1/2.75 as far as the servo.
    public static final double TURRET_SERVO_DEG_PER_UNIT = 1800.0;
    public static final double TURRET_GEAR_RATIO = 2.75;
    public static final double TURRET_DEG_PER_UNIT = TURRET_SERVO_DEG_PER_UNIT / TURRET_GEAR_RATIO; // ~654.5
    public static final double TURRET_CENTER = 0.5;       // measured: turret points straight ahead
    public static final double TURRET_MIN = 0.386;        // measured safe limits
    public static final double TURRET_MAX = 0.624;
    /** +1 if a larger servo command swings the turret counter-clockwise (left); flip to -1 if not. */
    public static final double TURRET_SIGN = 1.0;

    // HIVE per manual section 9.6 (TU11): centered on the field, red HIVE at x = 72 - 12.75 and
    // blue at x = 72 + 12.75 (25.5 in center to center); each HIVE has a CELL 9.42 in either side
    // of field center along y. Audience is at low y.
    public static final double HIVE_RED_X = 59.25;
    public static final double HIVE_BLUE_X = 84.75;
    public static final double HIVE_AUDIENCE_CELL_Y = 62.58;
    public static final double HIVE_SCORING_CELL_Y = 81.42;
    public static final double GOAL_HEIGHT_IN = 59.5;     // CELL opening is 53.5-65.6 in above the TILES

    // TODO measure/choose: where the robot sits at PLAY so odometry is in the field frame.
    private static final PoseFactory degrees = PoseFactory.degrees();
    public static Pose startPose() {
        return isBlue() ? degrees.of(86, 8, 90) : degrees.of(58, 8, 90);
    }

    public static final double pollenWeight = 24.95; //weight in g
    public static final double pollenDiameter = 2.8; //length in cm
    public static final double nectarWeight = 41.28; //weight in g
    public static final double nectarDiameter = 3.6; //length in cm

    public static final String FRONT_LEFT_DRIVE_MOTOR_NAME = "frontleft";
    public static final String FRONT_RIGHT_DRIVE_MOTOR_NAME = "frontright";
    public static final String BACK_LEFT_DRIVE_MOTOR_NAME = "backleft";
    public static final String BACK_RIGHT_DRIVE_MOTOR_NAME = "backright";
    public static final String PINPOINT_NAME = "pinpoint";
    public static final String LIMELIGHT_NAME = "limelight";

    public static final String INTAKE_LEFT_MOTOR_NAME = "intakeLeft";
    public static final String INTAKE_RIGHT_MOTOR_NAME = "intakeRight";
    public static final String LOADER_LEFT_SERVO_NAME = "loaderLeft";
    public static final String LOADER_RIGHT_SERVO_NAME = "loaderRight";

    public static final String TURRET_FLYWHEEL_MOTOR_NAME = "turret";
    public static final String TURRET_LINKAGE_SERVO_NAME = "linkage";
    public static final String TURRET_ROTATION_SERVO_NAME = "turretServo";
    public static final String TURRET_BLOCKER_SERVO_NAME = "blocker";
    public static final String INTAKE_KICKERRIGHT_SERVO_NAME = "kickerRight";
    public static final String INTAKE_KICKERLEFT_SERVO_NAME = "kickerLeft";

    public static final int TURRET_FLYWHEEL_TICKS_PER_REVOLUTION = 28; // goBILDA 5203-2402-0001
    public static final double TURRET_FLYWHEEL_MAX_TEST_RPM = 5000.0;

    public static final double TURRET_FLYWHEEL_KP = 12.5;
    public static final double TURRET_FLYWHEEL_KI = 1;
    public static final double TURRET_FLYWHEEL_KD = 0.0;
    public static final double TURRET_FLYWHEEL_KF = 14.5;


    public static Follower create(HardwareMap h) {
        return new Follower(
                new PinpointLocalizer(h, localizerConfig),
                new Mecanum(h, drivetrainConfig),
                new Foresight(foresightConfig)
        );
    }

    public static MecanumConfig drivetrainConfig = new MecanumConfig(c -> {
        c.frontLeftName.set(Constants.FRONT_LEFT_DRIVE_MOTOR_NAME);
        c.frontRightName.set(Constants.FRONT_RIGHT_DRIVE_MOTOR_NAME);
        c.backLeftName.set(Constants.BACK_LEFT_DRIVE_MOTOR_NAME);
        c.backRightName.set(Constants.BACK_RIGHT_DRIVE_MOTOR_NAME);
        c.frontLeftDirection.set(DcMotorSimple.Direction.REVERSE);
        c.frontRightDirection.set(DcMotorSimple.Direction.FORWARD);
        c.backLeftDirection.set(DcMotorSimple.Direction.REVERSE);
        c.backRightDirection.set(DcMotorSimple.Direction.FORWARD);
    });
    //fix
    public static PinpointConfig localizerConfig = new PinpointConfig(c -> {
        c.name.set(Constants.PINPOINT_NAME);
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
