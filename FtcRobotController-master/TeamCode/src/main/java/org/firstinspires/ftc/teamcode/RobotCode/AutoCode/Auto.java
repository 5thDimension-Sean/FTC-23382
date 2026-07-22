package org.firstinspires.ftc.teamcode.RobotCode.AutoCode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;

import org.firstinspires.ftc.teamcode.HardwareDefinitions;
import org.firstinspires.ftc.teamcode.RobotCode.Tuning;
import org.firstinspires.ftc.teamcode.pedroPathing.constants.FConstants;

@Autonomous(name="23382_auto", group="Robot")
public class Auto extends LinearOpMode {
    HardwareDefinitions robot = new HardwareDefinitions();
    //add routins as needed
    public enum routine {
        BLUE_LEFT,
        BLUE_RIGHT,
        RED_LEFT,
        RED_RIGHT
    }
    private routine selectedRoutine = routine.BLUE_LEFT;
    private double currentIncrement = 1.0;
    private int delaySteps = 0;
    private final double MAX_DELAY_SECONDS = 15.0; // max delay

    private Follower follower;
    private Timer pathTimer, opModeTimer;

    // Define your state enum
    public enum PathState {
        STATE_1,
        STATE_2,
        IDLE
    }
    private PathState pathState;

    // Define starting and target poses
    private final Pose startPose = new Pose(0, 0, Math.toRadians(0));
    private final Pose endPose = new Pose(24, 0, Math.toRadians(0));

    private PathChain drivePath;

    public void buildPaths() {
        drivePath = follower.pathBuilder()
                .addPath(new BezierLine(startPose, endPose))
                .setLinearHeadingInterpolation(startPose.getHeading(), endPose.getHeading())
                .build();
    }

    public void autonomousPathUpdate() {
        switch (pathState) {
            case STATE_1:
                // Start following the path, true = hold end heading
                follower.followPath(drivePath, true);
                setPathState(PathState.STATE_2);
                break;

            case STATE_2:
                // Wait until the robot finishes the path
                if (!follower.isBusy()) {
                    // Add your next action or transition here
                    setPathState(PathState.IDLE);
                }
                break;

            case IDLE:
                // Do nothing once finished
                break;
        }
    }


    public void setPathState(PathState newPathState) {
        pathState = newPathState;
        pathTimer.resetTimer();
    }


    @Override
    public void runOpMode() {
        Tuning.init(); // Initialize FTControl Panels with Pedro offsets

        boolean dpadLeftPressed = false;
        boolean dpadRightPressed = false;
        boolean dpadUpPressed = false;
        boolean dpadDownPressed = false;
        boolean bButtonPressed = false;

        // Initialize follower using your auto constants
        follower = FConstants.createFollower(hardwareMap);
        follower.setPose(startPose);
        pathTimer = new Timer();
        opModeTimer = new Timer();

        //loop to allow for routine switching
        while (!isStarted() && !isStopRequested()) {
            double totalDelaySeconds = delaySteps * currentIncrement;
            if (gamepad1.dpad_right && !dpadRightPressed) {
                selectedRoutine = getNextRoutine(selectedRoutine);
                dpadRightPressed = true;
            } else if (!gamepad1.dpad_right) {
                dpadRightPressed = false;
            }
            if (gamepad1.dpad_left && !dpadLeftPressed) {
                selectedRoutine = getPreviousRoutine(selectedRoutine);
                dpadLeftPressed = true;
            } else if (!gamepad1.dpad_left) {
                dpadLeftPressed = false;
            }
            if (gamepad1.b && !bButtonPressed) {
                cycleIncrement();
                delaySteps = 0;
                bButtonPressed = true;
            } else if (!gamepad1.b) {
                bButtonPressed = false;
            }
            if (gamepad1.dpad_up && !dpadUpPressed) {
                if ((totalDelaySeconds + currentIncrement) <= MAX_DELAY_SECONDS) {
                    delaySteps++;
                }
                dpadUpPressed = true;
            } else if (!gamepad1.dpad_up) {
                dpadUpPressed = false;
            }
            if (gamepad1.dpad_down && !dpadDownPressed) {
                if (delaySteps > 0) {
                    delaySteps--;
                }
                dpadDownPressed = true;
            } else if (!gamepad1.dpad_down) {
                dpadDownPressed = false;
            }

            //tele
            telemetry.addData("Selected Routine", ":  " + selectedRoutine);
            telemetry.addData("Step Size (B) ", ":  " + currentIncrement + " Seconds");
            telemetry.addData("Total Start Delay", "->  " + totalDelaySeconds + " Seconds");
            telemetry.addLine("\n-------------------------------------");
            telemetry.addLine("D-pad LEFT/RIGHT to change Routine.");
            telemetry.addLine("D-pad UP/DOWN to change Delay.");
            telemetry.addLine("Press (B) to swap step size (1.0, 0.75, 0.5, 0.25).");
            telemetry.update();

            sleep(10);
        }


        waitForStart();

        // Build paths after start if they depend on selection
        buildPaths();
        setPathState(PathState.STATE_1);
        opModeTimer.resetTimer();

        // Recalculate final active delay time
        double finalDelaySeconds = delaySteps * currentIncrement;


        if (finalDelaySeconds > 0) {
            telemetry.addData("Status", "Waiting out delay: " + finalDelaySeconds + "s");
            telemetry.update();
            sleep((long)(finalDelaySeconds * 1000)); // Convert double seconds to long milliseconds
        }

        while (opModeIsActive()) {
            switch (selectedRoutine) {
                case BLUE_LEFT:
                    runBlueLeft();
                    break;
                case BLUE_RIGHT:
                    runBlueRight();
                    break;
                case RED_LEFT:
                    runRedLeft();
                    break;
                case RED_RIGHT:
                    runRedRight();
                    break;
            }
        }
    }

    private routine getNextRoutine(routine current) {
        int nextOrdinal = current.ordinal() + 1;
        if (nextOrdinal >= routine.values().length) {
            return routine.values()[0]; // Loop back to start
        }
        return routine.values()[nextOrdinal];
    }

    private routine getPreviousRoutine(routine current) {
        int prevOrdinal = current.ordinal() - 1;
        if (prevOrdinal < 0) {
            return routine.values()[routine.values().length - 1]; // Loop to end
        }
        return routine.values()[prevOrdinal];
    }

    // Cycles through increment options when B button is tapped
    private void cycleIncrement() {
        if (currentIncrement == 1.0) {
            currentIncrement = 0.75;
        } else if (currentIncrement == 0.75) {
            currentIncrement = 0.50;
        } else if (currentIncrement == 0.50) {
            currentIncrement = 0.25;
        } else {
            currentIncrement = 1.0;
        }
    }

    public void updateRobotState(){
        follower.update();

        // Run state machine logic
        autonomousPathUpdate();

        // Telemetry data output
        Tuning.drawDebug(follower); // Draw to FTControl Panels
        telemetry.addData("Path State", pathState);
        telemetry.addData("X", follower.getPose().getX());
        telemetry.addData("Y", follower.getPose().getY());
        telemetry.addData("Heading", Math.toDegrees(follower.getPose().getHeading()));
        telemetry.addData("Path Timer", pathTimer.getElapsedTimeSeconds());
        telemetry.update();
    }
    private void runBlueLeft() {
        updateRobotState();
    }

    private void runBlueRight() {
        updateRobotState();
    }

    private void runRedLeft() {
        updateRobotState();
    }

    private void runRedRight() {
        updateRobotState();
    }
}
