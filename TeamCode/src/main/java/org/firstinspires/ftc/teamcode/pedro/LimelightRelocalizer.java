package org.firstinspires.ftc.teamcode.pedro;

import com.pedropathing.follower.Follower;
import com.pedropathing.math.Pose;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.robotcore.external.navigation.Position;

/**
 * Fixes odometry drift using the Limelight 3A's AprilTag pipeline.
 *
 * The Limelight already knows where every tag is (the season field map loaded on the camera), so it
 * hands back the robot's field position. This class validates that reading, converts it from the
 * FTC field frame to the Pedro frame, and writes it into the follower (and {@link RobotPose}).
 *
 * Two modes:
 *   - position only (MegaTag2): uses our own heading from odometry, so only x/y are corrected.
 *     Most accurate; use it by default.
 *   - position + heading (MegaTag1): also corrects heading. Needs a clear view of the tags.
 *
 * The Limelight camera offset on the robot is set in the Limelight web UI, not here.
 */
public class LimelightRelocalizer {
    private static final double METERS_TO_INCHES = 39.3701;
    /** How far outside the field a reading may land before it is thrown away. */
    private static final double FIELD_MARGIN_IN = 6;

    public static final class Reading {
        public Pose pose;          // Pedro frame
        public int tags;           // tags used for this pose
        public double avgTagDistIn;
        public long stalenessMs;
        public double driftIn;     // distance from the odometry pose passed to read()
        public boolean usedHeading;
    }

    private Limelight3A limelight;
    private String status = "not initialised";

    /** @return false if no Limelight is in the robot configuration (everything else then no-ops). */
    public boolean init(HardwareMap hardwareMap) {
        try {
            limelight = hardwareMap.get(Limelight3A.class, Constants.limelightName);
            limelight.pipelineSwitch(Constants.limelightAprilTagPipeline);
            status = "ready";
            return true;
        } catch (Exception e) {
            limelight = null;
            status = "no Limelight named '" + Constants.limelightName + "' in the config";
            return false;
        }
    }

    public boolean isAvailable() {
        return limelight != null;
    }

    /** Call in start(): begins polling. getLatestResult() returns null before this. */
    public void start() {
        if (limelight != null) limelight.start();
    }

    public void stop() {
        if (limelight != null) limelight.stop();
    }

    public String status() {
        return status;
    }

    /** Call every loop: MegaTag2 needs the robot's heading (in the FTC field frame) each frame. */
    public void feedHeading(Pose odometry) {
        if (limelight == null) return;
        double rotation = Math.toRadians(Constants.limelightFieldRotationDeg);
        double yawFtc = FieldGeometry.pedroHeadingToFtc(odometry.heading(), rotation);
        limelight.updateRobotOrientation(Math.toDegrees(yawFtc));
    }

    /**
     * Latest trustworthy AprilTag pose in the Pedro frame, or null (see {@link #status()} for why).
     *
     * @param odometry   current odometry pose, used for the drift number and (in position-only
     *                   mode) for the heading
     * @param useHeading true = MegaTag1, also correct heading; false = MegaTag2, position only
     */
    public Reading read(Pose odometry, boolean useHeading) {
        if (limelight == null) return null;
        LLResult r = limelight.getLatestResult();
        if (r == null || !r.isValid()) {
            status = "no AprilTag in view";
            return null;
        }
        if (r.getStaleness() > Constants.limelightMaxStalenessMs) {
            status = "frame is " + r.getStaleness() + " ms old";
            return null;
        }
        int tags = r.getBotposeTagCount();
        if (tags < Constants.limelightMinTags) {
            status = "only " + tags + " tag(s), need " + Constants.limelightMinTags;
            return null;
        }
        double avgDist = r.getBotposeAvgDist() * METERS_TO_INCHES;
        if (avgDist > Constants.limelightMaxAvgTagDistIn) {
            status = String.format("tags too far (%.0f in)", avgDist);
            return null;
        }
        Pose3D raw = useHeading ? r.getBotpose() : r.getBotpose_MT2();
        if (raw == null) {
            status = "no botpose in result";
            return null;
        }
        Position p = raw.getPosition();
        double xIn = p.unit.toInches(p.x);
        double yIn = p.unit.toInches(p.y);
        if (xIn == 0 && yIn == 0) { // Limelight reports 0,0 when it has no solution
            status = "botpose is empty";
            return null;
        }
        double yaw = raw.getOrientation().getYaw(AngleUnit.RADIANS);
        double rotation = Math.toRadians(Constants.limelightFieldRotationDeg);
        Pose pedro = FieldGeometry.ftcToPedro(xIn, yIn, yaw, rotation);
        if (!useHeading) pedro = pedro.withHeading(odometry.heading());

        double limit = Constants.fieldSize + FIELD_MARGIN_IN;
        if (pedro.x() < -FIELD_MARGIN_IN || pedro.x() > limit || pedro.y() < -FIELD_MARGIN_IN || pedro.y() > limit) {
            status = String.format("pose off the field (%.0f, %.0f) - check limelightFieldRotationDeg", pedro.x(), pedro.y());
            return null;
        }

        Reading out = new Reading();
        out.pose = pedro;
        out.tags = tags;
        out.avgTagDistIn = avgDist;
        out.stalenessMs = r.getStaleness();
        out.driftIn = pedro.distance(odometry);
        out.usedHeading = useHeading;
        status = "tags: " + tags;
        return out;
    }

    /**
     * Snaps the follower (and the shared {@link RobotPose}) to the AprilTag pose.
     *
     * @return true if a valid reading was applied; otherwise nothing changes
     */
    public boolean relocalize(Follower follower, boolean useHeading) {
        Reading reading = read(follower.pose(), useHeading);
        if (reading == null) return false;
        follower.setPose(reading.pose);
        RobotPose.save(reading.pose);
        status = String.format("reset! moved %.1f in (%d tags)", reading.driftIn, reading.tags);
        return true;
    }

    /**
     * Raw MegaTag1 pose in the FTC field frame {x in, y in, yaw rad, tags}, or null. Only for
     * working out {@link Constants#limelightFieldRotationDeg} (see the LimelightCheck TeleOp).
     */
    public double[] rawFtcPose() {
        if (limelight == null) return null;
        LLResult r = limelight.getLatestResult();
        if (r == null || !r.isValid()) return null;
        Pose3D raw = r.getBotpose();
        if (raw == null) return null;
        Position p = raw.getPosition();
        return new double[]{
                p.unit.toInches(p.x), p.unit.toInches(p.y),
                raw.getOrientation().getYaw(AngleUnit.RADIANS), r.getBotposeTagCount()};
    }
}
