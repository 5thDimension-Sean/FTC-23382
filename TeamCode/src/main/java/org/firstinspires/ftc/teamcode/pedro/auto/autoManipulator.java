package org.firstinspires.ftc.teamcode.pedro.auto;

import com.pedropathing.algorithm.ForesightConfig;
import com.pedropathing.follower.Follower;
import com.pedropathing.ivy.Command;
import com.pedropathing.ivy.commands.Commands;
import com.pedropathing.ivy.groups.Groups;
import com.pedropathing.math.Pose;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.pedro.Constants;
import org.firstinspires.ftc.teamcode.pedro.FieldDrawer;
import org.firstinspires.ftc.teamcode.pedro.RobotPose;

import java.util.ArrayList;
import java.util.List;

/**
 * Lets the driver tune every auto from the Driver Station during init, with the gamepad.
 *
 * The settings are static, so they are shared by every auto and survive between runs (until the
 * Robot Controller app restarts). Adding the three calls below to an auto is all it takes:
 *
 *   init_loop():  autoManipulator.initLoop(gamepad1, telemetry, start);
 *   start():      schedule(autoManipulator.start(follower, autoRoutine(), parkRoutine()));
 *   stop():       autoManipulator.stop(follower);
 *
 * Controls (gamepad1, during init):
 *   dpad up / down     pick a setting
 *   dpad left / right  change it by one step (speed 5 %, wait 0.5 s, toggles flip)
 *   bumpers            change it by 5 steps (speed 25 %, wait 2.5 s)
 *
 * Settings:
 *   Max speed          how fast the robot may drive the paths, 10-100 % of its top speed. Lower it
 *                      when the field is slippery or you want a smoother, more accurate run.
 *   Wait before start  how long to sit still after Play. Use it to let an alliance partner go
 *                      first, or to avoid crossing their path.
 *   Park at end        turn off if the partner is parking in the same zone.
 *
 * To add another setting: add an {@link Option}, add it to OPTIONS, and read it in start().
 */
public final class autoManipulator {

    /** One adjustable setting. The value is stored as a whole number of steps so it never drifts. */
    private static final class Option {
        final String name;
        final double unitPerStep;
        final String format; // how to print amount(), e.g. "%.0f%%"
        final int min, max;
        final boolean toggle;
        int steps;

        Option(String name, double unitPerStep, String format, int min, int max, int defaultSteps, boolean toggle) {
            this.name = name;
            this.unitPerStep = unitPerStep;
            this.format = format;
            this.min = min;
            this.max = max;
            this.toggle = toggle;
            this.steps = clamp(defaultSteps);
        }

        int clamp(int v) {
            return Math.max(min, Math.min(max, v));
        }

        void change(int deltaSteps) {
            if (toggle) {
                steps = deltaSteps > 0 ? 1 : 0; // right = ON, left = OFF
            } else {
                steps = clamp(steps + deltaSteps);
            }
        }

        double amount() {
            return steps * unitPerStep;
        }

        String display() {
            return toggle ? (steps == 1 ? "ON" : "OFF") : String.format(format, amount());
        }
    }

    // Speed: 10 % .. 100 % in 5 % steps.   Wait: 0 .. 25 s in 0.5 s steps.
    private static final Option MAX_SPEED = new Option("Max speed", 5, "%.0f%%", 2, 20,
            (int) Math.round(Constants.autoDefaultMaxSpeedPercent / 5.0), false);
    private static final Option WAIT = new Option("Wait before start", 0.5, "%.1f s", 0, 50,
            (int) Math.round(Constants.autoDefaultWaitSeconds / 0.5), false);
    private static final Option PARK = new Option("Park at end", 1, "", 0, 1,
            Constants.autoDefaultPark ? 1 : 0, true);
    private static final Option[] OPTIONS = {MAX_SPEED, WAIT, PARK};

    private static int selected = 0;
    private static boolean lastUp, lastDown, lastLeft, lastRight, lastLeftBumper, lastRightBumper;

    private autoManipulator() {}

    // --- values the autos read --------------------------------------------------------------

    /** Max drive speed as a fraction of top speed, 0.1 .. 1.0. */
    public static double maxSpeedFraction() {
        return MAX_SPEED.amount() / 100.0;
    }

    public static double waitSeconds() {
        return WAIT.amount();
    }

    public static boolean park() {
        return PARK.steps == 1;
    }

    // --- the three calls an auto makes --------------------------------------------------------

    /**
     * Call every init_loop(). Reads the gamepad, applies the changes and prints the menu. Also
     * draws the robot at the auto's start pose on the Panels field so you can check the setup.
     */
    public static void initLoop(Gamepad gamepad, Telemetry telemetry, Pose startPose) {
        boolean up = gamepad.dpad_up, down = gamepad.dpad_down;
        boolean left = gamepad.dpad_left, right = gamepad.dpad_right;
        boolean leftBumper = gamepad.left_bumper, rightBumper = gamepad.right_bumper;

        if (up && !lastUp) selected = (selected + OPTIONS.length - 1) % OPTIONS.length;
        if (down && !lastDown) selected = (selected + 1) % OPTIONS.length;
        if (left && !lastLeft) OPTIONS[selected].change(-1);
        if (right && !lastRight) OPTIONS[selected].change(+1);
        if (leftBumper && !lastLeftBumper) OPTIONS[selected].change(-5);
        if (rightBumper && !lastRightBumper) OPTIONS[selected].change(+5);
        lastUp = up;
        lastDown = down;
        lastLeft = left;
        lastRight = right;
        lastLeftBumper = leftBumper;
        lastRightBumper = rightBumper;

        telemetry.addLine("AUTO SETTINGS   dpad up/down = pick, left/right = change, bumpers = x5");
        for (int i = 0; i < OPTIONS.length; i++) {
            telemetry.addData((i == selected ? "> " : "   ") + OPTIONS[i].name, OPTIONS[i].display());
        }
        telemetry.addData("Alliance", RobotPose.alliance());
        telemetry.addData("Start pose", "x %.1f  y %.1f  h %.1f deg",
                startPose.x(), startPose.y(), Math.toDegrees(startPose.heading()));

        FieldDrawer.drawRobot(startPose);
        FieldDrawer.update();
    }

    /**
     * Call in start(). Applies the speed limit and returns the command to schedule: the wait,
     * then the routine.
     */
    public static Command start(Follower follower, Command routine) {
        return start(follower, routine, null);
    }

    /**
     * Same, with a park routine that is only run when "Park at end" is ON. Put the park moves
     * last in the auto, so skipping them leaves the robot where the routine ended.
     */
    public static Command start(Follower follower, Command routine, Command parkRoutine) {
        double speed = maxSpeedFraction();
        // 100 % = no limit, i.e. exactly how the auto behaved before the manipulator existed.
        Constants.foresightConfig.maxPathSpeed.set(speed >= 1.0 ? ForesightConfig.Constraint.NONE : speed);

        List<Command> steps = new ArrayList<>();
        if (waitSeconds() > 0) steps.add(Commands.waitMs(waitSeconds() * 1000.0));
        steps.add(routine);
        if (parkRoutine != null && park()) steps.add(parkRoutine);
        return Groups.sequential(steps.toArray(new Command[0]));
    }

    /** Call in stop(): hands the speed limit back and saves the final pose for teleop. */
    public static void stop(Follower follower) {
        Constants.foresightConfig.maxPathSpeed.set(ForesightConfig.Constraint.NONE);
        RobotPose.update(follower);
    }
}
