package org.firstinspires.ftc.teamcode.pedro.teleop;

import com.pedropathing.follower.Follower;
import com.pedropathing.math.Pose;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.pedro.Constants;
import org.firstinspires.ftc.teamcode.pedro.subsystem.intake;
import org.firstinspires.ftc.teamcode.pedro.subsystem.loader;
import org.firstinspires.ftc.teamcode.pedro.subsystem.nectarSensors;
import org.firstinspires.ftc.teamcode.pedro.subsystem.turret;

@TeleOp(name = Constants.DRIVER_TELEOP_NAME)
public class teleop extends OpMode {

    private Follower follower;
    private Pose fieldToPlayFrame;
    intake intake = new intake();
    turret turret = new turret();
    loader loader = new loader();
    nectarSensors nectar = new nectarSensors();
    private int loaderPresses;
    private int intakePresses;
    private int flywheelPresses;

    @Override
    public void init() {
        follower = Constants.createForTeleOp(hardwareMap);
        intake.init(hardwareMap);
        turret.init(hardwareMap);
        loader.init(hardwareMap);
        nectar.init(hardwareMap);
        loader.position("up");
    }

    @Override
    public void init_loop() {
        if (gamepad1.x) {
            Constants.selectedTeam = Constants.colorLeft;
        } else if (gamepad1.b) {
            Constants.selectedTeam = Constants.colorRight;
        }

        telemetry.addData("Team selection", Constants.selectedTeam);
        follower.localizer.update();
        Pose pose = follower.pose();
        telemetry.addData("Pinpoint field pose (in, deg)", "%.1f, %.1f, %.1f",
                pose.x(), pose.y(), Math.toDegrees(pose.heading()));
        telemetry.addData("Select team", "X = %s, B = %s", Constants.colorLeft, Constants.colorRight);
        telemetry.update();
    }

    @Override
    public void start() {
        gamepad1.resetEdgeDetection();
        follower.localizer.update();
        fieldToPlayFrame = follower.pose().invert();
    }

    @Override
    public void loop() {
        follower.manual(
                -gamepad1.left_stick_y,
                -gamepad1.left_stick_x,
                -gamepad1.right_stick_x
        );

        if (gamepad1.leftBumperWasPressed()) {
            loaderPresses++;
            if (loaderPresses % 2 == 0) {
                loader.position("up");
            } else {
                loader.position("down");
            }
        }

        if (gamepad1.rightTriggerWasPressed()) {
            intakePresses++;
            if (intakePresses % 2 == 0) {
                flywheelPresses = 0;
            }
        }
        if (gamepad1.rightBumperWasPressed()) {
            flywheelPresses++;
        }
        boolean intakeRunning = intakePresses % 2 != 0;
        boolean flywheelRunning = flywheelPresses % 2 != 0;
        // Hold the left trigger to outtake; releasing it resumes the intake toggle state.
        boolean outtakeRunning = gamepad1.left_trigger > 0.5;
        if (outtakeRunning) {
            intake.setPowerGradual(-1);
        } else if (intakeRunning) {
            intake.setPowerGradual(1);
        } else {
            intake.setPowerGradual(0);
        }

        follower.update();
        turret.trackHive(follower.pose(), telemetry);
        // The turret always aims at the HIVE; the flywheel only spins while the turret is activated.
        // Existing goal offsets are expressed relative to PLAY. Keep Pinpoint in field coordinates.
        turret.aimAtGoal(fieldToPlayFrame.compose(follower.pose()), telemetry);
        if (flywheelRunning) {
            turret.setFlywheelVelocity(Constants.TURRET_FLYWHEEL_SHOT_TICKS_PER_SEC);
        } else {
            turret.setFlywheelVelocity(0);
        }

        if (gamepad2.b) {
            turret.moveTurretManual(1);
        } else if (gamepad2.x) {
            turret.moveTurretManual(0);
        }
        // Color sort: NECTAR (red/blue) seen by either sensor closes the linkage, otherwise it opens.
        // Gamepad 2 Y / A still override while held.
        if (nectar.nectarPresent(telemetry)) {
            turret.moveLink("close");
        } else {
            turret.moveLink("open");
        }
        if (gamepad2.y) {
            turret.moveLink("open");
        } else if (gamepad2.a) {
            turret.moveLink("close");
        }
        if (gamepad2.left_trigger_pressed) {
            turret.moveBlocker("open");
        } else if (gamepad2.left_bumper) {
            turret.moveBlocker("close");
        }

        // Keep the blocker open while the turret is moving, otherwise close it during intake.
        if (outtakeRunning || flywheelRunning || gamepad2.b || gamepad2.x) {
            turret.moveBlocker("open");
        } else if (intakeRunning) {
            turret.moveBlocker("close");
        }

        telemetry.addData("Target Position(block)", turret.blocker.getPosition());
        telemetry.addData("Target Position(linkage)", turret.linkage.getPosition());
        telemetry.addData("Target Position(turret)", turret.turretServo.getPosition());
        telemetry.addData("Target Position(loaderLeft)", loader.loaderLeft.getPosition());
        telemetry.addData("Target Position(loaderRight)", loader.loaderRight.getPosition());
        telemetry.addData("Intake running", intakeRunning);
        telemetry.addData("Turret flywheel on", flywheelRunning);
        telemetry.update();
    }

    @Override
    public void stop() {
        turret.stopTracking();
    }
}
