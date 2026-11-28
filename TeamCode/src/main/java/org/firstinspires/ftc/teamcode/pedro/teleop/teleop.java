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

@TeleOp(name = "23382 Teleop")
public class teleop extends OpMode {

    private Follower follower;
    intake intake = new intake();
    turret turret = new turret();
    loader loader = new loader();
    nectarSensors nectar = new nectarSensors();
    private boolean intakeRunning = false;
    private boolean turretAutoTracking = false;
    private boolean rightBumperWasPressed = false;
    private boolean loaderDown = false;
    private boolean loaderButtonWasPressed = false;

    @Override
    public void init() {
        follower = Constants.create(hardwareMap);
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
        telemetry.addData("Select team", "X = %s, B = %s", Constants.colorLeft, Constants.colorRight);
        telemetry.update();
    }

    @Override
    public void start() {
        // PLAY pose is the origin: (0, 0) with heading 0 = robot front.
        follower.setPose(Pose.zero());
    }

    @Override
    public void loop() {
        follower.manual(
                -gamepad1.left_stick_y,
                -gamepad1.left_stick_x,
                -gamepad1.right_stick_x
        );

        boolean loaderButtonPressed = gamepad1.left_bumper;
        if (loaderButtonPressed && !loaderButtonWasPressed) {
            if (loaderDown) {
                loaderDown = false;
                loader.position("up");
            } else {
                loaderDown = true;
                loader.position("down");
            }
        }
        loaderButtonWasPressed = loaderButtonPressed;

        // Press the right trigger once to start the intake; press it again to stop.
        if (gamepad1.right_trigger_pressed) {
            if (intakeRunning) {
                intakeRunning = false;
                turretAutoTracking = false;
            } else {
                intakeRunning = true;
            }
        }

        // While the intake is running, the right bumper toggles turret auto tracking.
        boolean rightBumperPressed = gamepad1.right_bumper;
        if (rightBumperPressed && !rightBumperWasPressed && intakeRunning) {
            turretAutoTracking = !turretAutoTracking;
        }
        rightBumperWasPressed = rightBumperPressed;
        // Hold the left trigger to outtake; releasing it resumes the intake toggle state.
        boolean outtakeRunning = gamepad1.left_trigger > 0.5;
        intake.setPowerGradual(outtakeRunning ? -1 : (intakeRunning ? 1 : 0));
        follower.update();
        turret.trackHive(follower.pose(), telemetry);
        if (turretAutoTracking) {
            turret.aimAtGoal(follower.pose(), telemetry);
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
        turret.moveLink(nectar.nectarPresent(telemetry) ? "close" : "open");
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
        if (outtakeRunning || turretAutoTracking || gamepad2.b || gamepad2.x) {
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
        telemetry.addData("Turret auto aim", turretAutoTracking);
        telemetry.update();
    }

    @Override
    public void stop() {
        turret.stopTracking();
    }
}
