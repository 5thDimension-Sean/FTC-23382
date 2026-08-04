package org.firstinspires.ftc.teamcode.RobotCode.tuning;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;

import org.firstinspires.ftc.teamcode.RobotCode.Tuning;
import org.firstinspires.ftc.teamcode.RobotCode.constants.FConstants;

/**
 * Drives a curved path DISTANCE inches forward-left then back.
 * Used to test curved path following tuning (centripetal, heading, drive vectors).
 */
@Config
@Autonomous(name = "Curved Back And Forth", group = "Pedro Tuning")
public class CurvedBackAndForth extends OpMode {
    public static double DISTANCE = 20;

    private Follower follower;
    private PathChain forwards, backwards;
    private boolean forward = true;

    @Override
    public void init() {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        Tuning.init();

        follower = FConstants.createFollower(hardwareMap);

        double d = Math.abs(DISTANCE);
        Pose p0  = new Pose(0, 0, 0);
        Pose p1  = new Pose(d, 0, 0);
        Pose p2  = new Pose(d, DISTANCE, 0);

        forwards = follower.pathBuilder()
                .addPath(new BezierCurve(p0, p1, p2))
                .build();

        backwards = follower.pathBuilder()
                .addPath(new BezierCurve(p2, p1, p0))
                .build();

        follower.followPath(forwards, true);

        telemetry.addLine("Running curved path over " + DISTANCE + " inches.");
        telemetry.update();
    }

    @Override
    public void loop() {
        follower.update();

        if (!follower.isBusy()) {
            if (forward) {
                forward = false;
                follower.followPath(backwards, true);
            } else {
                forward = true;
                follower.followPath(forwards, true);
            }
        }

        Tuning.drawDebug(follower);
        telemetry.addData("going forward", forward);
        follower.telemetryDebug(telemetry);
    }
}
