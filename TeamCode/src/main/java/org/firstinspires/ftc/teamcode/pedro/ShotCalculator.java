package org.firstinspires.ftc.teamcode.pedro;

import com.pedropathing.math.Pose;

/**
 * Turret / shooter math. Pure Java (no hardware) so it can be unit tested off the robot.
 *
 * Given where the robot is, it works out:
 *   - the flat distance to the hive (hypotenuse of the x/y difference),
 *   - the height difference between the launcher and the hive opening, and from both of those the
 *     straight line-of-sight distance (second hypotenuse),
 *   - the turret angle that points at the hive (relative to the robot, within the turret's limits),
 *   - the launch speed (and flywheel RPM) a fixed-angle shooter needs to put the ball into the hive,
 *   - and finally whether the shot should be taken, and if not, why.
 *
 * A tipped hive uses a different target height, aim offset and minimum entry angle.
 * All tuning values live in {@link Constants}. Angles in the Solution are radians unless named Deg.
 */
public final class ShotCalculator {
    /** Gravity in inches per second squared. */
    public static final double GRAVITY = 386.088;

    private ShotCalculator() {}

    public static final class Solution {
        /** Hive point being aimed at (field inches). */
        public double targetX, targetY;
        /** Target height above the floor, inches. */
        public double targetHeight;
        /** Turret pivot position on the field, inches. */
        public double turretX, turretY;

        /** Flat distance turret -> hive. */
        public double horizontalDistance;
        /** Hive opening height minus launch height (negative = shooting downhill). */
        public double heightDiff;
        /** Straight-line distance turret -> hive opening: hypot(horizontalDistance, heightDiff). */
        public double lineOfSightDistance;

        /** Direction to the hive in the field frame. */
        public double fieldBearing;
        /** Turret angle to command, relative to the robot front, clamped to the turret limits. */
        public double turretAngle;
        /** False when the hive is behind a turret limit, so turretAngle is only the closest reachable. */
        public boolean turretInRange;

        /** Ball speed needed at the launcher, in/s. NaN when the hive cannot be reached at the fixed angle. */
        public double launchSpeed;
        public double flywheelRpm;
        /** How steeply the ball is falling when it reaches the hive, degrees below horizontal. */
        public double entryAngleDeg;

        public boolean hiveTipped;
        public boolean canShoot;
        /** "OK" when canShoot, otherwise the first reason the shot is refused. */
        public String reason;
    }

    /**
     * @param robot              current odometry pose
     * @param alliance           which hive to shoot at
     * @param hiveTipped         true if the hive is currently tipped
     * @param robotSpeed         robot translational speed, in/s (shots are refused while moving)
     * @param currentTurretAngle current turret angle in radians, used to choose between equivalent
     *                           angles when the turret can travel more than one turn
     */
    public static Solution solve(Pose robot, Alliance alliance, boolean hiveTipped,
                                 double robotSpeed, double currentTurretAngle) {
        Solution s = new Solution();
        s.hiveTipped = hiveTipped;

        // --- where the ball leaves from -------------------------------------------------------
        double[] pivot = FieldGeometry.robotToField(robot,
                Constants.turretOffsetForward, Constants.turretOffsetLeft);
        s.turretX = pivot[0];
        s.turretY = pivot[1];

        // --- where we are aiming (depends on tipped / upright) --------------------------------
        Pose hive = alliance.hive();
        s.targetX = hive.x() + (hiveTipped ? Constants.hiveTippedOffsetX : 0);
        s.targetY = hive.y() + (hiveTipped ? Constants.hiveTippedOffsetY : 0);
        s.targetHeight = hiveTipped ? Constants.hiveHeightTipped : Constants.hiveHeightUpright;
        double minEntryDeg = hiveTipped ? Constants.minEntryAngleDegTipped : Constants.minEntryAngleDegUpright;

        // --- distances: flat hypotenuse, height difference, line-of-sight hypotenuse ----------
        double dx = s.targetX - s.turretX;
        double dy = s.targetY - s.turretY;
        s.horizontalDistance = Math.hypot(dx, dy);
        s.heightDiff = s.targetHeight - Constants.launchHeight;
        s.lineOfSightDistance = Math.hypot(s.horizontalDistance, s.heightDiff);

        // --- turret angle ---------------------------------------------------------------------
        s.fieldBearing = Math.atan2(dy, dx);
        double wanted = FieldGeometry.wrap(s.fieldBearing - robot.heading());
        double min = Math.toRadians(Constants.turretMinAngleDeg);
        double max = Math.toRadians(Constants.turretMaxAngleDeg);
        s.turretInRange = false;
        double best = Double.NaN;
        for (int k = -2; k <= 2; k++) { // equivalent angles, in case the turret can turn > 360 deg
            double candidate = wanted + k * 2 * Math.PI;
            if (candidate < min || candidate > max) continue;
            if (Double.isNaN(best) || Math.abs(candidate - currentTurretAngle) < Math.abs(best - currentTurretAngle)) {
                best = candidate;
            }
        }
        if (Double.isNaN(best)) {
            best = Math.abs(wanted - min) < Math.abs(wanted - max) ? min : max; // closest limit
        } else {
            s.turretInRange = true;
        }
        s.turretAngle = best;

        // --- launch speed for a fixed hood angle ----------------------------------------------
        // Projectile: y = d tan(a) - g d^2 / (2 v^2 cos^2(a)), solved for v at y = heightDiff.
        double a = Math.toRadians(Constants.shooterLaunchAngleDeg);
        double d = s.horizontalDistance;
        double denominator = 2 * Math.pow(Math.cos(a), 2) * (d * Math.tan(a) - s.heightDiff);
        if (denominator > 1e-9 && d > 1e-9) {
            s.launchSpeed = Math.sqrt(GRAVITY * d * d / denominator);
            double vx = s.launchSpeed * Math.cos(a);
            double vyAtTarget = s.launchSpeed * Math.sin(a) - GRAVITY * (d / vx);
            s.entryAngleDeg = Math.toDegrees(Math.atan2(-vyAtTarget, vx));
            double wheelRadPerSec = s.launchSpeed / (Constants.flywheelTransferEfficiency * Constants.flywheelRadiusIn);
            s.flywheelRpm = wheelRadPerSec * 60.0 / (2 * Math.PI);
        } else {
            s.launchSpeed = Double.NaN;
            s.entryAngleDeg = Double.NaN;
            s.flywheelRpm = Double.NaN;
        }

        // --- take the shot? (first failing check wins) ----------------------------------------
        if (hiveTipped && !Constants.allowShotWhenTipped) {
            s.reason = "hive tipped";
        } else if (s.horizontalDistance < Constants.minShotDistanceIn) {
            s.reason = "too close";
        } else if (s.horizontalDistance > Constants.maxShotDistanceIn) {
            s.reason = "too far";
        } else if (!s.turretInRange) {
            s.reason = "hive outside turret range";
        } else if (Double.isNaN(s.launchSpeed)) {
            s.reason = "hive unreachable at hood angle";
        } else if (s.entryAngleDeg < minEntryDeg) {
            s.reason = "entry angle too shallow";
        } else if (s.flywheelRpm > Constants.flywheelMaxRpm) {
            s.reason = "needs more flywheel speed than available";
        } else if (robotSpeed > Constants.maxShotRobotSpeedIn) {
            s.reason = "robot moving";
        } else {
            s.reason = "OK";
        }
        s.canShoot = s.reason.equals("OK");
        return s;
    }
}
