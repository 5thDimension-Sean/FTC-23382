package org.firstinspires.ftc.teamcode.pedro.teleop;

import com.pedropathing.follower.Follower;
import com.pedropathing.math.Pose;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.pedro.Constants;

@TeleOp(name = "Hive Odometry Preview", group = "Calibration")
public class HiveOdometryPreview extends LimelightHiveOpeningPreview {
    private static final double DRIVE_SCALE = 0.30;
    private static final double MIN_BASELINE_INCHES = 12;
    private static final double MIN_INTERSECTION_SINE = Math.sin(Math.toRadians(15));

    private Follower follower;
    private Pose first;
    private int cell;
    private double hiveX = Double.NaN;
    private double hiveY = Double.NaN;
    private boolean previousA;
    private boolean previousY;
    private String status = "Aim the straight-ahead shooter at the opening. Press A.";

    @Override
    public void init() {
        super.init();
        follower = Constants.create(hardwareMap);
        follower.setPose(Pose.zero());
        hardwareMap.get(DcMotorEx.class, Constants.TURRET_FLYWHEEL_MOTOR_NAME).setPower(0);
    }

    @Override
    public void start() {
        super.start();
        previousA = gamepad1.a;
        previousY = gamepad1.y;
    }

    @Override
    protected void handleOpening(LLResult frame, HiveOpeningVision.Opening opening) {
        follower.manual(-DRIVE_SCALE * gamepad1.left_stick_y,
                -DRIVE_SCALE * gamepad1.left_stick_x, -DRIVE_SCALE * gamepad1.right_stick_x);
        follower.update();
        Pose pose = follower.pose();
        if (gamepad1.a && !previousA) {
            captureFirst(pose, opening);
        } else if (gamepad1.y && !previousY) {
            captureSecond(pose, opening);
        }
        previousA = gamepad1.a;
        previousY = gamepad1.y;

        telemetry.addData("Odometry setup", status);
        telemetry.addData("Pinpoint position (in)", "X %.1f / Y %.1f", pose.x(), pose.y());
        telemetry.addData("Pinpoint heading (deg)", "%.1f", Math.toDegrees(pose.heading()));
        if (Double.isFinite(hiveX) && (opening == null || opening.cell == cell)) {
            double dx = hiveX - pose.x();
            double dy = hiveY - pose.y();
            double relative = Math.atan2(dy, dx) - pose.heading();
            double normalized = Math.atan2(Math.sin(relative), Math.cos(relative));
            telemetry.addData("Predicted Hive bearing (deg, left positive)", "%+.1f",
                    Math.toDegrees(normalized));
            telemetry.addData("Predicted Hive distance (in)", "%.1f", Math.hypot(dx, dy));
        } else if (Double.isFinite(hiveX)) {
            telemetry.addData("Odometry prediction", "Cell changed. Capture again.");
        }
    }

    private void captureFirst(Pose pose, HiveOpeningVision.Opening opening) {
        if (opening == null) {
            status = "Show the upright opening before pressing A.";
            return;
        }
        first = pose;
        cell = opening.cell;
        hiveX = Double.NaN;
        hiveY = Double.NaN;
        status = "A saved. Move sideways, turn to face the same opening, stop and press Y.";
    }

    private void captureSecond(Pose pose, HiveOpeningVision.Opening opening) {
        if (first == null || opening == null || opening.cell != cell) {
            status = "Press A first and keep the same upright cell for Y.";
            return;
        }

        double dx = pose.x() - first.x();
        double dy = pose.y() - first.y();
        double ux = Math.cos(first.heading()), uy = Math.sin(first.heading());
        double vx = Math.cos(pose.heading()), vy = Math.sin(pose.heading());
        double intersection = ux * vy - uy * vx;
        if (Math.hypot(dx, dy) < MIN_BASELINE_INCHES
                || Math.abs(intersection) < MIN_INTERSECTION_SINE) {
            status = "Move farther sideways and face the opening again before pressing Y.";
            return;
        }

        // Intersect the forward-facing sight lines from both odometry poses.
        double firstDistance = (dx * vy - dy * vx) / intersection;
        double secondDistance = (dx * uy - dy * ux) / intersection;
        if (!(firstDistance > 0 && secondDistance > 0)) {
            status = "The sight lines do not meet ahead of both poses. Recheck alignment.";
            return;
        }
        hiveX = first.x() + ux * firstDistance;
        hiveY = first.y() + uy * firstDistance;
        status = "Hive point saved. You can now check the prediction out of camera view.";
    }
}
