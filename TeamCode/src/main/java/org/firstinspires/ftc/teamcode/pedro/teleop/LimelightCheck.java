package org.firstinspires.ftc.teamcode.pedro.teleop;

import com.pedropathing.follower.Follower;
import com.pedropathing.math.Pose;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.pedro.Constants;
import org.firstinspires.ftc.teamcode.pedro.FieldGeometry;
import org.firstinspires.ftc.teamcode.pedro.LimelightRelocalizer;

/**
 * One-time setup helper: finds the right value for Constants.limelightFieldRotationDeg.
 *
 * 1. Put the robot on a spot you know (e.g. the start pose in Constants.teleopFallbackStart) with
 *    an AprilTag in view of the Limelight.
 * 2. Run this OpMode. It starts odometry at that known pose, then shows what the Limelight pose
 *    would convert to for each of the four possible frame rotations.
 * 3. The rotation with the smallest error is the one to put in Constants.
 */
@TeleOp(name = "LimelightCheck", group = "Setup")
public class LimelightCheck extends OpMode {
    private static final double[] ROTATIONS_DEG = {0, 90, 180, -90};

    private Follower follower;
    private final LimelightRelocalizer limelight = new LimelightRelocalizer();

    @Override
    public void init() {
        follower = Constants.create(hardwareMap);
        follower.setPose(Constants.teleopFallbackStart);
        follower.update();
        limelight.init(hardwareMap);
    }

    @Override
    public void start() {
        limelight.start();
    }

    @Override
    public void loop() {
        follower.update();
        Pose known = follower.pose();
        telemetry.addData("Odometry (known pose)", "x %.1f  y %.1f  h %.1f deg",
                known.x(), known.y(), Math.toDegrees(known.heading()));

        double[] raw = limelight.rawFtcPose();
        if (raw == null) {
            telemetry.addData("Limelight", limelight.isAvailable() ? "no AprilTag in view" : limelight.status());
            telemetry.update();
            return;
        }
        telemetry.addData("Limelight raw (FTC frame)", "x %.1f  y %.1f  yaw %.1f deg  (%d tags)",
                raw[0], raw[1], Math.toDegrees(raw[2]), (int) raw[3]);

        double bestRotation = ROTATIONS_DEG[0];
        double bestScore = Double.MAX_VALUE;
        for (double rotDeg : ROTATIONS_DEG) {
            Pose converted = FieldGeometry.ftcToPedro(raw[0], raw[1], raw[2], Math.toRadians(rotDeg));
            double posError = converted.distance(known);
            double headingError = Math.abs(FieldGeometry.wrap(converted.heading() - known.heading()));
            telemetry.addData("Rotation " + (int) rotDeg, "x %.1f  y %.1f  h %.1f deg   error %.1f in, %.1f deg",
                    converted.x(), converted.y(), Math.toDegrees(converted.heading()),
                    posError, Math.toDegrees(headingError));
            double score = posError + 12 * headingError;
            if (score < bestScore) {
                bestScore = score;
                bestRotation = rotDeg;
            }
        }
        telemetry.addData("BEST limelightFieldRotationDeg", (int) bestRotation);
        telemetry.addData("Currently set", Constants.limelightFieldRotationDeg);
        telemetry.update();
    }

    @Override
    public void stop() {
        limelight.stop();
    }
}
