package org.firstinspires.ftc.teamcode.RobotCode.tuning;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;

import org.firstinspires.ftc.teamcode.RobotCode.Tuning;
import org.firstinspires.ftc.teamcode.RobotCode.constants.FConstants;

/**
 * Drives the robot DISTANCE inches forward then backward continuously.
 * Used to test straight-line path following tuning.
 */
@Config
@Autonomous(name = "Straight Back And Forth", group = "Pedro Tuning")
public class StraightBackAndForth extends OpMode {
    public static double DISTANCE = 40;

    private Follower follower;
    private PathChain forwards, backwards;
    private boolean forward = true;

    @Override
    public void init() {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        Tuning.init();

        follower = FConstants.createFollower(hardwareMap);

        Pose start = new Pose(0, 0, 0);
        Pose end   = new Pose(DISTANCE, 0, 0);

        forwards = follower.pathBuilder()
                .addPath(new BezierLine(start, end))
                .setLinearHeadingInterpolation(0, 0)
                .build();

        backwards = follower.pathBuilder()
                .addPath(new BezierLine(end, start))
                .setLinearHeadingInterpolation(0, 0)
                .build();

        follower.followPath(forwards, true);

        telemetry.addLine("Running straight back and forth over " + DISTANCE + " inches.");
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
