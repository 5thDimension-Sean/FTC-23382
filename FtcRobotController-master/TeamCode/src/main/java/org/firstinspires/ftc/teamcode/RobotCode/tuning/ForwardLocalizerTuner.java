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
 * Push the robot straight forward DISTANCE inches using a ruler.
 * Read the displayed multiplier and put it into LConstants.forwardTicksToInches.
 */
@Config
@Autonomous(name = "Forward Localizer Tuner", group = "Pedro Tuning")
public class ForwardLocalizerTuner extends OpMode {
    public static double DISTANCE = 48;

    private Follower follower;

    @Override
    public void init() {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        follower = FConstants.createFollower(hardwareMap);
        telemetry.addLine("Push the robot " + DISTANCE + " inches forward, then read the multiplier.");
        telemetry.update();
    }

    @Override
    public void loop() {
        follower.update();
        double measured = follower.getPose().getX();
        double multiplier = (measured == 0) ? 0
                : LConstants.forwardTicksToInches * DISTANCE / measured;

        telemetry.addData("distance moved (in)", measured);
        telemetry.addData("forwardTicksToInches should be", multiplier);
        telemetry.update();
    }
}
