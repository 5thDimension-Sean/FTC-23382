package org.firstinspires.ftc.teamcode.pedro.teleop;

import com.pedropathing.follower.Follower;
import com.pedropathing.math.Pose;
import com.pedropathing.math.Velocity;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.pedro.Alliance;
import org.firstinspires.ftc.teamcode.pedro.Constants;
import org.firstinspires.ftc.teamcode.pedro.FieldDrawer;
import org.firstinspires.ftc.teamcode.pedro.LimelightRelocalizer;
import org.firstinspires.ftc.teamcode.pedro.RobotPose;
import org.firstinspires.ftc.teamcode.pedro.ShotCalculator;

/**
 * Controls (gamepad1):
 *   sticks          drive
 *   y               toggle "hive is tipped"
 *   a               toggle turret auto-aim (off at start so the turret never moves until you ask)
 *   left bumper     reset odometry from the AprilTags, position only (MegaTag2)
 *   right bumper    reset odometry from the AprilTags, position + heading (MegaTag1)
 *   x / b (in init) pick BLUE / RED alliance (otherwise the alliance auto set)
 */
@TeleOp(name = "23382 Teleop")
public class teleop extends OpMode {

    private Follower follower;
    intake i = new intake();
    turret t = new turret();
    private final LimelightRelocalizer limelight = new LimelightRelocalizer();

    private RobotPose.Source poseSource;
    private boolean hiveTipped = false;
    private boolean autoAim = false;
    private boolean lastY, lastA, lastLeftBumper, lastRightBumper;
    private String resetMessage = "-";

    @Override
    public void init() {
        follower = Constants.create(hardwareMap);
        // Continue from where auto ended (or the fallback start if auto was a while ago).
        poseSource = RobotPose.startTeleop(follower);
        i.init(hardwareMap);
        t.init(hardwareMap);
        limelight.init(hardwareMap);
    }

    @Override
    public void init_loop() {
        if (gamepad1.x) RobotPose.setAlliance(Alliance.BLUE);
        if (gamepad1.b) RobotPose.setAlliance(Alliance.RED);

        Pose p = follower.pose();
        telemetry.addData("Alliance (x=blue, b=red)", RobotPose.alliance());
        telemetry.addData("Start pose from", poseSource);
        telemetry.addData("Start pose", "x %.1f  y %.1f  h %.1f deg", p.x(), p.y(), Math.toDegrees(p.heading()));
        telemetry.addData("Limelight", limelight.status());
        telemetry.update();
    }

    @Override
    public void start() {
        limelight.start();
    }

    @Override
    public void loop() {
        follower.manual(
                -gamepad1.left_stick_y,
                gamepad1.left_stick_x,
                gamepad1.right_stick_x
        );

        follower.update();
        Pose pose = follower.pose();

        // --- Limelight: keep it informed of our heading, and reset on request ------------------
        limelight.feedHeading(pose);
        if (gamepad1.left_bumper && !lastLeftBumper) {
            resetMessage = limelight.relocalize(follower, false) ? "position reset" : "failed: " + limelight.status();
        }
        if (gamepad1.right_bumper && !lastRightBumper) {
            resetMessage = limelight.relocalize(follower, true) ? "position + heading reset" : "failed: " + limelight.status();
        }
        lastLeftBumper = gamepad1.left_bumper;
        lastRightBumper = gamepad1.right_bumper;
        pose = follower.pose(); // may have just been corrected

        // Shared pose for whatever OpMode runs next.
        RobotPose.update(follower);

        // --- driver toggles ------------------------------------------------------------------
        if (gamepad1.y && !lastY) hiveTipped = !hiveTipped;
        if (gamepad1.a && !lastA) {
            autoAim = !autoAim;
            if (!autoAim) t.release();
        }
        lastY = gamepad1.y;
        lastA = gamepad1.a;

        // --- turret + shot decision ----------------------------------------------------------
        Velocity v = follower.velocity();
        double speed = Math.hypot(v.vx, v.vy);
        ShotCalculator.Solution shot = ShotCalculator.solve(
                pose, RobotPose.alliance(), hiveTipped, speed, t.getAngle());
        if (autoAim) t.aim(shot.turretAngle);

        // --- Panels field view ---------------------------------------------------------------
        FieldDrawer.drawRobot(pose);
        FieldDrawer.drawTarget(shot.targetX, shot.targetY);
        FieldDrawer.drawAimLine(pose, shot.targetX, shot.targetY, shot.canShoot);
        FieldDrawer.update();

        // --- telemetry -----------------------------------------------------------------------
        telemetry.addData("Pose", "x %.1f  y %.1f  h %.1f deg", pose.x(), pose.y(), Math.toDegrees(pose.heading()));
        telemetry.addData("Alliance / hive", "%s / %s", RobotPose.alliance(), hiveTipped ? "TIPPED" : "upright");
        telemetry.addData("Flat dist / height diff / line of sight", "%.1f / %.1f / %.1f in",
                shot.horizontalDistance, shot.heightDiff, shot.lineOfSightDistance);
        telemetry.addData("Turret", "%s  target %.1f deg  now %.1f deg%s", autoAim ? "AUTO-AIM" : "off",
                Math.toDegrees(shot.turretAngle), Math.toDegrees(t.getAngle()),
                shot.turretInRange ? "" : "  (out of range)");
        telemetry.addData("Launch", "%.0f in/s  %.0f rpm  entry %.0f deg",
                shot.launchSpeed, shot.flywheelRpm, shot.entryAngleDeg);
        telemetry.addData("TAKE SHOT?", shot.canShoot ? "YES" : "NO - " + shot.reason);

        LimelightRelocalizer.Reading seen = limelight.read(pose, false);
        telemetry.addData("Limelight", seen != null
                ? String.format("%d tag(s), odometry is %.1f in off", seen.tags, seen.driftIn)
                : limelight.status());
        telemetry.addData("Last reset (LB pos, RB pos+heading)", resetMessage);
        telemetry.update();
    }

    @Override
    public void stop() {
        RobotPose.update(follower);
        t.release();
        limelight.stop();
    }
}
