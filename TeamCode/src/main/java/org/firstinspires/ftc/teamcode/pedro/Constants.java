package org.firstinspires.ftc.teamcode.pedro;

import com.pedropathing.algorithm.Foresight;
import com.pedropathing.algorithm.ForesightConfig;
import com.pedropathing.controllers.Controller;
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
    public static final double pollenWeight = 24.95; //weight in g
    public static final double pollenDiameter = 2.8; //length in cm
    public static final double nectarWeight = 41.28; //weight in g
    public static final double nectarDiameter = 3.6; //length in cm

    // ------------------------------------------------------------------------------------------
    // Robot footprint (inches). Used by FieldDrawer to draw the robot rectangle in Panels, so
    // change it here and the drawing follows.
    // ------------------------------------------------------------------------------------------
    public static final double robotWidth = 16;  // left <-> right
    public static final double robotLength = 16; // front <-> back

    // ------------------------------------------------------------------------------------------
    // Field + auto -> teleop pose handoff
    // ------------------------------------------------------------------------------------------
    public static final double fieldSize = 144; // Pedro field is 144 x 144 in, origin bottom-left

    /** Pose saved by auto is only trusted by teleop if it was updated this recently. */
    public static final long poseHandoffMaxAgeMs = 60_000;

    /** Teleop start pose when there is no fresh pose from auto (same spot as soloRed's start). */
    public static final Pose teleopFallbackStart = new Pose(58.75, 9.5, Math.toRadians(180));

    // ------------------------------------------------------------------------------------------
    // Limelight 3A AprilTag relocalization
    // Camera mounting offset must be entered in the Limelight web UI (robot-space camera pose), so
    // the botpose it returns is already for the robot centre.
    // ------------------------------------------------------------------------------------------
    public static final String limelightName = "limelight";  // name in the robot configuration
    public static final int limelightAprilTagPipeline = 0;    // pipeline that runs AprilTags
    public static final int limelightMinTags = 1;             // tags needed before we trust a reset
    public static final long limelightMaxStalenessMs = 100;   // reject frames older than this
    public static final double limelightMaxAvgTagDistIn = 120; // reject when tags are further than this
    /**
     * Rotation between the FTC field frame (what the Limelight reports, origin = field centre) and
     * the Pedro frame (origin = bottom-left corner), in degrees: one of 0, 90, 180, -90.
     * Run the "LimelightCheck" TeleOp with the robot on a known pose; it prints the best value.
     */
    public static final double limelightFieldRotationDeg = 0;

    // ------------------------------------------------------------------------------------------
    // Turret / shooter / goal ("hive") math, used by ShotCalculator and turret.
    //
    // !! PLACEHOLDERS !! Every value in this block is a starting guess. Measure them on the robot
    // and check them against the Biobuzz game manual before trusting the output.
    // ------------------------------------------------------------------------------------------
    // Goal (hive) position on the field, Pedro inches.
    public static final Pose redHive = new Pose(131, 131);
    public static final Pose blueHive = new Pose(13, 131);

    // Target height (inches above the floor) that the ball has to reach.
    public static final double hiveHeightUpright = 30;
    public static final double hiveHeightTipped = 22;
    // Aim point shift when the hive is tipped (inches, field frame, added to the hive position).
    public static final double hiveTippedOffsetX = 0;
    public static final double hiveTippedOffsetY = 0;
    // The ball must be falling at least this steeply (deg below horizontal) when it reaches the hive.
    public static final double minEntryAngleDegUpright = 25;
    public static final double minEntryAngleDegTipped = 10;
    /** If false, ShotCalculator refuses every shot while the hive is tipped. */
    public static final boolean allowShotWhenTipped = true;

    // Where the ball leaves the robot.
    public static final double launchHeight = 14;           // inches above the floor
    public static final double shooterLaunchAngleDeg = 55;  // fixed hood angle above horizontal
    public static final double turretOffsetForward = 0;     // turret pivot vs robot centre (+ = front)
    public static final double turretOffsetLeft = 0;        // turret pivot vs robot centre (+ = left)

    // Flywheel: converts the launch speed the physics needs into a wheel RPM.
    public static final double flywheelRadiusIn = 1.89;     // 96 mm wheel
    public static final double flywheelTransferEfficiency = 0.6; // ball speed / wheel surface speed
    public static final double flywheelMaxRpm = 6000;

    // Take-the-shot limits.
    public static final double minShotDistanceIn = 24;
    public static final double maxShotDistanceIn = 130;
    public static final double maxShotRobotSpeedIn = 6;     // in/s, robot must be nearly stopped

    // Turret mechanism. Angle 0 = turret pointing at the robot's front, positive = counter-clockwise.
    // Encoder 0 must be that position when the robot is powered on.
    public static final double turretTicksPerRev = 2000;    // encoder ticks for one full turret turn
    public static final double turretMinAngleDeg = -180;
    public static final double turretMaxAngleDeg = 180;
    public static final double turretPower = 0.8;


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