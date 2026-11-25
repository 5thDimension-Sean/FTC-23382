package org.firstinspires.ftc.teamcode.pedro.teleop;

import com.pedropathing.follower.Follower;
import com.pedropathing.math.Pose;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.PwmControl;
import com.qualcomm.robotcore.hardware.ServoImplEx;

import org.firstinspires.ftc.teamcode.pedro.Constants;

@TeleOp(name = "Hive Odometry Preview", group = "Calibration")
public class HiveOdometryPreview extends LimelightHiveOpeningPreview {
    private static final double DRIVE_SCALE = 0.30;
    private static final double MIN_BASELINE_INCHES = 12;
    private static final double MIN_INTERSECTION_SINE = Math.sin(Math.toRadians(15));

    private static final long MAX_VISION_GAP_NANOS = 4_000_000_000L;
    // Measured from 90 degrees right to 90 degrees left, every 45 degrees.
    private static final double[] TURRET_COMMANDS = {0.3655, 0.4325,
            HiveTurretAim.STRAIGHT_AHEAD_POSITION, 0.5600, 0.6285};

    private Follower follower;
    private ServoImplEx servo;
    private HiveTurretAim aim;
    private long lastMatchingVisionNanos;
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
        servo = hardwareMap.get(ServoImplEx.class, Constants.TURRET_ROTATION_SERVO_NAME);
        servo.setPwmRange(new PwmControl.PwmRange(500, 2500));
        aim = new HiveTurretAim(servo, hardwareMap.appContext);
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
        boolean captureA = gamepad1.a && !previousA;
        boolean captureY = gamepad1.y && !previousY;
        if (captureA || captureY) {
            if (gamepad1.left_bumper) {
                status = "Release LB before recording the Hive point.";
            } else if (!(Math.abs(servo.getPosition() - HiveTurretAim.STRAIGHT_AHEAD_POSITION) <= 0.003)) {
                status = "Set the turret straight ahead at 0.5000 in Turret Travel Calibration.";
            } else if (captureA) {
                captureFirst(pose, opening);
            } else {
                captureSecond(pose, opening);
            }
        }
        previousA = gamepad1.a;
        previousY = gamepad1.y;

        telemetry.addData("Odometry setup", status);
        telemetry.addData("Pinpoint position (in)", "X %.1f / Y %.1f", pose.x(), pose.y());
        telemetry.addData("Pinpoint heading (deg)", "%.1f", Math.toDegrees(pose.heading()));
        double predictedBearing = Double.NaN;
        if (Double.isFinite(hiveX) && (opening == null || opening.cell == cell)) {
            double dx = hiveX - pose.x();
            double dy = hiveY - pose.y();
            double relative = Math.atan2(dy, dx) - pose.heading();
            double normalized = Math.atan2(Math.sin(relative), Math.cos(relative));
            predictedBearing = Math.toDegrees(normalized);
            telemetry.addData("Predicted Hive bearing (deg, left positive)", "%+.1f", predictedBearing);
            telemetry.addData("Predicted Hive distance (in)", "%.1f", Math.hypot(dx, dy));
        } else if (Double.isFinite(hiveX)) {
            telemetry.addData("Odometry prediction", "Cell changed. Capture again.");
        }
        telemetry.addData("Servo command", "%.4f", servo.getPosition());
        telemetry.addData("Reacquisition test",
                updateAim(frame, opening, predictedBearing, !captureA && !captureY));
        telemetry.addLine("After both points are saved, hold LB to test. Release LB to hold.");
    }

    private String updateAim(LLResult frame, HiveOpeningVision.Opening opening,
            double bearing, boolean allowMovement) {
        long now = System.nanoTime();
        boolean hasMap = Double.isFinite(hiveX);
        boolean matching = opening != null && hasMap && opening.cell == cell;
        if (matching) {
            lastMatchingVisionNanos = now;
        } else if (opening != null || !hasMap) {
            lastMatchingVisionNanos = 0;
        }
        boolean enabled = allowMovement && gamepad1.left_bumper && hasMap;
        String visionStatus = aim.update(frame, opening, enabled && matching, false, false, telemetry);
        if (!enabled || opening != null) {
            return enabled && !matching ? "Holding: cell changed" : visionStatus;
        }
        if (!aim.isCalibrated()) {
            return "Holding: calibrate in Hive Tracking Test";
        }
        if (lastMatchingVisionNanos == 0 || now - lastMatchingVisionNanos > MAX_VISION_GAP_NANOS) {
            return "Holding: odometry timeout";
        }
        if (!Double.isFinite(bearing) || Math.abs(bearing) > 90) {
            return "Holding: Hive beyond the turret range";
        }
        double segment = (bearing + 90) / 45;
        int lower = Math.min((int) segment, TURRET_COMMANDS.length - 2);
        double target = TURRET_COMMANDS[lower]
                + (segment - lower) * (TURRET_COMMANDS[lower + 1] - TURRET_COMMANDS[lower]);
        return aim.nudgeTowardOdometry(target);
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
