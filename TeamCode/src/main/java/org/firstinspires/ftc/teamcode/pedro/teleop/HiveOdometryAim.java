package org.firstinspires.ftc.teamcode.pedro.teleop;

import com.pedropathing.math.Pose;

import org.firstinspires.ftc.robotcore.external.Telemetry;

final class HiveOdometryAim {
    private static final double MIN_BASELINE_INCHES = 12;
    private static final double MIN_INTERSECTION_SINE = Math.sin(Math.toRadians(15));
    private static final long MAX_VISION_GAP_NANOS = 4_000_000_000L;
    private static final double[] TURRET_COMMANDS = {0.3655, 0.4325,
            HiveTurretAim.STRAIGHT_AHEAD_POSITION, 0.5600, 0.6285};

    private final FieldPoint[] points = new FieldPoint[4];
    private Pose first;
    private int pendingCell = -1;
    private int lastSeenCell = -1;
    private long lastSeenNanos;
    private String status = "No Hive point saved.";

    String status() {
        return status;
    }

    boolean isMapped(HiveOpeningVision.Opening opening) {
        return opening != null && points[opening.cell] != null;
    }

    void clearLastSeen() {
        lastSeenCell = -1;
    }

    void observe(HiveOpeningVision.Opening opening) {
        if (opening != null) {
            lastSeenCell = opening.cell;
            lastSeenNanos = System.nanoTime();
        }
    }

    void captureNext(Pose pose, HiveOpeningVision.Opening opening, double command) {
        if (opening != null && first != null && pendingCell == opening.cell) {
            captureSecond(pose, opening, command);
        } else {
            captureFirst(pose, opening, command);
        }
    }

    void captureFirst(Pose pose, HiveOpeningVision.Opening opening, double command) {
        if (!canCapture(pose, opening, command)) {
            return;
        }
        first = pose;
        pendingCell = opening.cell;
        status = "First point saved. Move sideways and line up the same opening.";
    }

    void captureSecond(Pose pose, HiveOpeningVision.Opening opening, double command) {
        if (!canCapture(pose, opening, command)) {
            return;
        }
        if (first == null || opening.cell != pendingCell) {
            status = "Record the first sighting of this cell.";
            return;
        }

        double dx = pose.x() - first.x();
        double dy = pose.y() - first.y();
        double ux = Math.cos(first.heading()), uy = Math.sin(first.heading());
        double vx = Math.cos(pose.heading()), vy = Math.sin(pose.heading());
        double intersection = ux * vy - uy * vx;
        if (Math.hypot(dx, dy) < MIN_BASELINE_INCHES
                || Math.abs(intersection) < MIN_INTERSECTION_SINE) {
            status = "Move farther sideways and face the same opening again.";
            return;
        }

        double firstDistance = (dx * vy - dy * vx) / intersection;
        double secondDistance = (dx * uy - dy * ux) / intersection;
        if (!(firstDistance > 0 && secondDistance > 0)) {
            status = "Sight lines do not meet ahead of both poses. Recheck alignment.";
            return;
        }
        points[opening.cell] = new FieldPoint(first.x() + ux * firstDistance,
                first.y() + uy * firstDistance);
        first = null;
        pendingCell = -1;
        status = "Hive point saved for this cell.";
    }

    private boolean canCapture(Pose pose, HiveOpeningVision.Opening opening, double command) {
        if (opening == null) {
            status = "Show the upright opening before recording a sighting.";
            return false;
        }
        if (!validPose(pose)) {
            status = "Pinpoint pose unavailable.";
            return false;
        }
        if (!(Math.abs(command - HiveTurretAim.STRAIGHT_AHEAD_POSITION) <= 0.0035)) {
            status = "Set turret straight at 0.5000 in Turret Travel Calibration.";
            return false;
        }
        return true;
    }

    void showPrediction(Pose pose, Telemetry telemetry) {
        FieldPoint point = lastPoint();
        if (point != null && validPose(pose)) {
            telemetry.addData("Predicted Hive bearing (deg, left positive)", "%+.1f",
                    bearingDegrees(pose, point));
            telemetry.addData("Predicted Hive distance (in)", "%.1f",
                    Math.hypot(point.x - pose.x(), point.y - pose.y()));
        }
    }

    String reacquire(Pose pose, HiveTurretAim aim) {
        FieldPoint point = lastPoint();
        if (point == null) {
            return "Holding: target lost, cell not mapped";
        }
        if (System.nanoTime() - lastSeenNanos > MAX_VISION_GAP_NANOS) {
            return "Holding: odometry timeout";
        }
        if (!validPose(pose)) {
            return "Holding: Pinpoint pose unavailable";
        }
        double bearing = bearingDegrees(pose, point);
        if (!Double.isFinite(bearing) || Math.abs(bearing) > 90) {
            return "Holding: Hive beyond the turret range";
        }
        double segment = (bearing + 90) / 45;
        int lower = Math.min((int) segment, TURRET_COMMANDS.length - 2);
        double target = TURRET_COMMANDS[lower]
                + (segment - lower) * (TURRET_COMMANDS[lower + 1] - TURRET_COMMANDS[lower]);
        return aim.nudgeTowardOdometry(target);
    }

    private FieldPoint lastPoint() {
        return lastSeenCell < 0 ? null : points[lastSeenCell];
    }

    private static double bearingDegrees(Pose pose, FieldPoint point) {
        double relative = Math.atan2(point.y - pose.y(), point.x - pose.x()) - pose.heading();
        return Math.toDegrees(Math.atan2(Math.sin(relative), Math.cos(relative)));
    }

    private static boolean validPose(Pose pose) {
        return pose != null && Double.isFinite(pose.x()) && Double.isFinite(pose.y())
                && Double.isFinite(pose.heading());
    }

    private static final class FieldPoint {
        final double x;
        final double y;

        FieldPoint(double x, double y) {
            this.x = x;
            this.y = y;
        }
    }
}
