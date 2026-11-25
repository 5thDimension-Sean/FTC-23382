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
    private final Sighting[] automaticFirst = new Sighting[4];
    private Pose stablePose;
    private int stableCell = -1;
    private double stableCommand;
    private long stableSince;
    private double lastAutomaticFrame = Double.NaN;
    private Sighting first;
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
        first = new Sighting(pose, pose.heading(), Double.NaN);
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

        FieldPoint point = intersect(first, new Sighting(pose, pose.heading(), Double.NaN));
        if (point == null) {
            status = "Move farther sideways and recheck alignment with the same opening.";
            return;
        }
        points[opening.cell] = point;
        first = null;
        pendingCell = -1;
        status = "Hive point saved for this cell.";
    }

    void mapAutomatically(Pose pose, HiveOpeningVision.Opening opening, double command,
            double frameTimestamp, boolean moving, HiveTurretAim aim) {
        if (moving || opening == null || !validPose(pose) || !aim.isCalibrated()
                || command < TURRET_COMMANDS[0] || command > TURRET_COMMANDS[TURRET_COMMANDS.length - 1]
                || !Double.isFinite(command)) {
            stablePose = null;
            return;
        }
        int cell = opening.cell;
        if (points[cell] != null || pendingCell == cell) {
            stablePose = null;
            return;
        }
        double error = aim.openingErrorDegrees(opening);
        if (!Double.isFinite(error) || Math.abs(error) > 5.0) {
            stablePose = null;
            return;
        }

        long now = System.nanoTime();
        if (stablePose == null || stableCell != cell || Math.abs(command - stableCommand) > 0.0015
                || Math.hypot(pose.x() - stablePose.x(), pose.y() - stablePose.y()) > 1.0
                || Math.abs(normalize(pose.heading() - stablePose.heading())) > Math.toRadians(2)) {
            stablePose = pose;
            stableCell = cell;
            stableCommand = command;
            stableSince = now;
            lastAutomaticFrame = frameTimestamp;
            return;
        }
        if (now - stableSince < 250_000_000L || frameTimestamp == lastAutomaticFrame) {
            return;
        }
        lastAutomaticFrame = frameTimestamp;
        double heading = pose.heading() + Math.toRadians(angleForCommand(command) - error);
        Sighting sample = new Sighting(pose, heading, Math.hypot(opening.x, opening.z));
        Sighting previous = automaticFirst[cell];
        if (previous == null) {
            automaticFirst[cell] = sample;
            status = "Auto: first sighting saved. Move sideways and pause with Hive in view.";
            return;
        }
        FieldPoint point = intersect(previous, sample);
        if (point != null) {
            points[cell] = point;
            automaticFirst[cell] = null;
            status = "Hive point mapped automatically.";
        }
    }

    private static double angleForCommand(double command) {
        int lower = 0;
        while (lower < TURRET_COMMANDS.length - 2 && command > TURRET_COMMANDS[lower + 1]) {
            lower++;
        }
        double fraction = (command - TURRET_COMMANDS[lower])
                / (TURRET_COMMANDS[lower + 1] - TURRET_COMMANDS[lower]);
        return -90 + (lower + fraction) * 45;
    }

    private static FieldPoint intersect(Sighting first, Sighting second) {
        double dx = second.pose.x() - first.pose.x();
        double dy = second.pose.y() - first.pose.y();
        double ux = Math.cos(first.heading), uy = Math.sin(first.heading);
        double vx = Math.cos(second.heading), vy = Math.sin(second.heading);
        double intersection = ux * vy - uy * vx;
        if (Math.hypot(dx, dy) < MIN_BASELINE_INCHES
                || Math.abs(intersection) < MIN_INTERSECTION_SINE) {
            return null;
        }
        double firstDistance = (dx * vy - dy * vx) / intersection;
        double secondDistance = (dx * uy - dy * ux) / intersection;
        if (!validDistance(firstDistance, first.cameraDepth)
                || !validDistance(secondDistance, second.cameraDepth)) {
            return null;
        }
        return new FieldPoint(first.pose.x() + ux * firstDistance,
                first.pose.y() + uy * firstDistance);
    }

    private static boolean validDistance(double distance, double cameraDepth) {
        return Double.isFinite(distance) && distance > 0
                && (Double.isNaN(cameraDepth)
                || (distance >= 0.6 * cameraDepth && distance <= 1.6 * cameraDepth));
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
        return Math.toDegrees(normalize(relative));
    }

    private static double normalize(double radians) {
        return Math.atan2(Math.sin(radians), Math.cos(radians));
    }

    private static boolean validPose(Pose pose) {
        return pose != null && Double.isFinite(pose.x()) && Double.isFinite(pose.y())
                && Double.isFinite(pose.heading());
    }

    private static final class Sighting {
        final Pose pose;
        final double heading;
        final double cameraDepth;

        Sighting(Pose pose, double heading, double cameraDepth) {
            this.pose = pose;
            this.heading = heading;
            this.cameraDepth = cameraDepth;
        }
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
