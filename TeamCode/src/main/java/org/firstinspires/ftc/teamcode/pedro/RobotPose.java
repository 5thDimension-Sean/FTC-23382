package org.firstinspires.ftc.teamcode.pedro;

import com.pedropathing.follower.Follower;
import com.pedropathing.math.Pose;

/**
 * The one odometry pose shared by every OpMode.
 *
 * Static fields survive an OpMode ending, so whatever auto last saw is still here when teleop
 * starts. They are lost only when the Robot Controller app restarts, which is why teleop also
 * checks the pose is recent and falls back to a default start pose otherwise.
 *
 * Usage:
 *   auto  init():  RobotPose.startAuto(follower, start, alliance)   // start = first pose of the auto
 *   auto  loop():  RobotPose.update(follower)
 *   auto  stop():  RobotPose.update(follower)
 *   teleop init(): RobotPose.startTeleop(follower)
 *   teleop loop(): RobotPose.update(follower)
 */
public final class RobotPose {
    public enum Source {
        /** Teleop resumed from the pose auto left behind. */
        AUTO_HANDOFF,
        /** No recent pose was saved, so the default start pose was used. */
        FALLBACK
    }

    private static Pose pose = null;
    private static long updatedAtMs = 0;
    private static Alliance alliance = Alliance.RED;

    private RobotPose() {}

    /** Call in auto init: puts the follower at the auto's first pose and records it. */
    public static void startAuto(Follower follower, Pose start, Alliance a) {
        alliance = a;
        follower.setPose(start);
        follower.update();
        save(start);
    }

    /**
     * Call in teleop init: continues from where auto ended if that is recent, otherwise from
     * {@link Constants#teleopFallbackStart}.
     */
    public static Source startTeleop(Follower follower) {
        Source source;
        Pose startPose;
        if (hasFreshPose()) {
            startPose = pose;
            source = Source.AUTO_HANDOFF;
        } else {
            startPose = Constants.teleopFallbackStart;
            source = Source.FALLBACK;
        }
        follower.setPose(startPose);
        follower.update();
        save(startPose);
        return source;
    }

    /** Call every loop (and in stop()) in auto and teleop. */
    public static void update(Follower follower) {
        save(follower.pose());
    }

    /** Overwrites the stored pose, e.g. after the Limelight corrected the follower. */
    public static void save(Pose p) {
        pose = p;
        updatedAtMs = System.currentTimeMillis();
    }

    public static boolean hasFreshPose() {
        return pose != null && System.currentTimeMillis() - updatedAtMs <= Constants.poseHandoffMaxAgeMs;
    }

    /** Last saved pose, or null if nothing has been saved since the app started. */
    public static Pose get() {
        return pose;
    }

    public static Alliance alliance() {
        return alliance;
    }

    public static void setAlliance(Alliance a) {
        alliance = a;
    }

    /** Forget the saved pose (next teleop will use the fallback start). */
    public static void clear() {
        pose = null;
    }
}
