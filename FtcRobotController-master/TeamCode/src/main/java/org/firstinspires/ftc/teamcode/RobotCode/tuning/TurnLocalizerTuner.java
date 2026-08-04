package org.firstinspires.ftc.teamcode.RobotCode.tuning;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import com.pedropathing.follower.Follower;

import org.firstinspires.ftc.teamcode.RobotCode.constants.FConstants;
import org.firstinspires.ftc.teamcode.RobotCode.constants.LConstants;

/**
 * Spin the robot TURNS full rotations (default 1).
 * Read the displayed multiplier and put it into LConstants.turnTicksToInches.
 */
@Config
@Autonomous(name = "Turn Localizer Tuner", group = "Pedro Tuning")
public class TurnLocalizerTuner extends OpMode {
    public static double TURNS = 1;

    private Follower follower;
    private double totalHeading;
    private double lastHeading;

    @Override
    public void init() {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        follower = FConstants.createFollower(hardwareMap);
        totalHeading = 0;
        lastHeading  = 0;
        telemetry.addLine("Spin the robot " + TURNS + " full turn(s), then read the multiplier.");
        telemetry.update();
    }

    @Override
    public void loop() {
        follower.update();

        double heading = follower.getPose().getHeading();
        double delta = heading - lastHeading;
        if (delta > Math.PI)  delta -= 2 * Math.PI;
        if (delta < -Math.PI) delta += 2 * Math.PI;
        totalHeading += delta;
        lastHeading = heading;

        double target = TURNS * 2 * Math.PI;
        double multiplier = (totalHeading == 0) ? 0
                : LConstants.turnTicksToInches * target / totalHeading;

        telemetry.addData("total heading (rad)", totalHeading);
        telemetry.addData("turnTicksToInches should be", multiplier);
        telemetry.update();
    }
}
