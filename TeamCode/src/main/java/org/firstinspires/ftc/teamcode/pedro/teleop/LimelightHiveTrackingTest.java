package org.firstinspires.ftc.teamcode.pedro.teleop;

import com.pedropathing.follower.Follower;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.PwmControl;
import com.qualcomm.robotcore.hardware.ServoImplEx;

import org.firstinspires.ftc.teamcode.pedro.Constants;

@TeleOp(name = "Limelight Hive Tracking Test", group = "Calibration")
public class LimelightHiveTrackingTest extends LimelightHiveOpeningPreview {
    private Follower follower;
    private ServoImplEx servo;
    private DcMotorEx flywheel;
    private HiveTurretAim aim;
    private HiveOpeningVision.Opening first;
    private boolean previousA, previousY;
    private String calibration;

    @Override
    public void init() {
        super.init();
        follower = Constants.create(hardwareMap);
        flywheel = hardwareMap.get(DcMotorEx.class, Constants.TURRET_FLYWHEEL_MOTOR_NAME);
        flywheel.setPower(0);
        servo = hardwareMap.get(ServoImplEx.class, Constants.TURRET_ROTATION_SERVO_NAME);
        servo.setPwmRange(new PwmControl.PwmRange(500, 2500));
        aim = new HiveTurretAim(servo, hardwareMap.appContext);
        calibration = aim.isCalibrated() ? "Saved calibration loaded. A starts recalibration."
                : "Aim at center from two distances. A saves first, Y saves second.";
    }

    @Override
    public void start() {
        super.start();
        previousA = gamepad1.a;
        previousY = gamepad1.y;
    }

    @Override
    protected void handleOpening(LLResult frame, HiveOpeningVision.Opening opening) {
        follower.manual(-0.35 * gamepad1.left_stick_y, -0.35 * gamepad1.left_stick_x,
                -0.35 * gamepad1.right_stick_x);
        follower.update();

        boolean captureFirst = gamepad1.a && !previousA;
        boolean captureSecond = gamepad1.y && !previousY;
        previousA = gamepad1.a;
        previousY = gamepad1.y;
        if (captureFirst || captureSecond) {
            capture(captureFirst, opening);
        }

        boolean manual = gamepad1.b || gamepad1.x;
        if (gamepad1.b != gamepad1.x) {
            aim.manualStep(gamepad1.b);
        }
        boolean track = gamepad1.left_bumper && first == null && !captureFirst && !captureSecond;
        telemetry.addData("Calibration", calibration);
        telemetry.addData("Servo command", "%.4f", servo.getPosition());
        telemetry.addData("Turret", aim.update(frame, opening, track, manual, false, telemetry));
        telemetry.addLine("X/B manual. Hold LB to track. Flywheel off.");
    }

    private void capture(boolean captureFirst, HiveOpeningVision.Opening opening) {
        if (opening == null) {
            calibration = "Show the upright opening before capturing.";
        } else if (captureFirst) {
            first = opening;
            calibration = "First saved. Re-aim at a depth at least 12 inches different, then press Y.";
        } else if (first == null || Math.abs(opening.z - first.z)
                < HiveTurretAim.MIN_CALIBRATION_DEPTH_DIFFERENCE) {
            calibration = "Save A first. A and Y need at least 12 inches of depth difference.";
        } else {
            aim.saveCalibration(first, opening);
            first = null;
            calibration = "Two points saved on the Control Hub.";
        }
    }

    @Override
    public void stop() {
        super.stop();
        if (flywheel != null) {
            flywheel.setPower(0);
        }
    }
}
