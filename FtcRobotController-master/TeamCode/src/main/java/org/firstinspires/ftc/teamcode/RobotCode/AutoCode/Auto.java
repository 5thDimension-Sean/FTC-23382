package org.firstinspires.ftc.teamcode.RobotCode.AutoCode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.HardwareDefinitions;


@Autonomous(name="Auto_Selector", group="Robot")
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



    @Override
    public void runOpMode() {

        boolean dpadLeftPressed = false;
        boolean dpadRightPressed = false;
        boolean dpadUpPressed = false;
        boolean dpadDownPressed = false;
        boolean bButtonPressed = false;

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
            telemetry.addData("Selected Routine", ":  " + selectedRoutine + "  <-");
            telemetry.addData("Step Size (B) ", ":  " + currentIncrement + " Seconds");
            telemetry.addData("Total Start Delay", "->  " + totalDelaySeconds + " Seconds  <-");
            telemetry.addLine("\n-------------------------------------");
            telemetry.addLine("Use D-pad LEFT/RIGHT to change Routine.");
            telemetry.addLine("Use D-pad UP/DOWN to change Delay.");
            telemetry.addLine("Press (B) to swap step size (1.0, 0.75, 0.5, 0.25).");
            telemetry.update();

            sleep(10);
        }

        // --- START PRESSED (Match Begins) ---
        waitForStart();

        // Recalculate final active delay time
        double finalDelaySeconds = delaySteps * currentIncrement;

        // Step 1: Execute the user-selected match delay
        if (finalDelaySeconds > 0) {
            telemetry.addData("Status", "Waiting out delay: " + finalDelaySeconds + "s");
            telemetry.update();
            sleep((long)(finalDelaySeconds * 1000)); // Convert double seconds to long milliseconds
        }

        // Step 2: Run the chosen field trajectory
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

    // --- Path Routines ---
    private void runBlueLeft() {
        telemetry.addData("Status", "Executing Blue Left");
        telemetry.update();
    }

    private void runBlueRight() {
        telemetry.addData("Status", "Executing Blue Right");
        telemetry.update();
    }

    private void runRedLeft() {
        telemetry.addData("Status", "Executing Red Left");
        telemetry.update();
    }

    private void runRedRight() {
        telemetry.addData("Status", "Executing Red Right");
        telemetry.update();
    }
}
