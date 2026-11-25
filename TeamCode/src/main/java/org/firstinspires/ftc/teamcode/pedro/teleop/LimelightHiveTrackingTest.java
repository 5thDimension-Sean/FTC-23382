package org.firstinspires.ftc.teamcode.pedro.teleop;

import com.pedropathing.follower.Follower;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.PwmControl;
import com.qualcomm.robotcore.hardware.ServoImplEx;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.pedro.Constants;

@TeleOp(name = "Limelight Hive Tracking Test", group = "Calibration")
public class LimelightHiveTrackingTest extends LimelightHiveOpeningPreview {
    private static final double MIN_POSITION = 0.32;
    private static final double MAX_POSITION = 0.38;
    private static final double MAX_STEP = 0.001;
    private static final double DEADBAND_DEGREES = 1.0;
    private static final long STEP_INTERVAL_NANOS = 60_000_000L;

    private Follower follower;
    private ServoImplEx servo;
    private DcMotorEx flywheel;
    private double firstX, secondX;
    private double firstZ = Double.NaN, secondZ = Double.NaN;
    private double lastFrame = Double.NaN;
    private long nextStep;
    private boolean previousA, previousY;
    private String calibration = "Aim at center from two distances. A saves first, Y saves second.";

    @Override
    public void init() {
        super.init();
        follower = Constants.create(hardwareMap);
        flywheel = hardwareMap.get(DcMotorEx.class, Constants.TURRET_FLYWHEEL_MOTOR_NAME);
        flywheel.setPower(0);
        servo = hardwareMap.get(ServoImplEx.class, Constants.TURRET_ROTATION_SERVO_NAME);
        servo.setPwmRange(new PwmControl.PwmRange(500, 2500));
    }

    @Override
    public void start() {
        super.start();
        previousA = gamepad1.a;
        previousY = gamepad1.y;
    }

    @Override
    protected void handleOpening(LLResult frame, double x, double z) {
        follower.manual(-0.35 * gamepad1.left_stick_y, -0.35 * gamepad1.left_stick_x,
                -0.35 * gamepad1.right_stick_x);
        follower.update();

        boolean captureFirst = gamepad1.a && !previousA;
        boolean captureSecond = gamepad1.y && !previousY;
        previousA = gamepad1.a;
        previousY = gamepad1.y;
        boolean targetVisible = Double.isFinite(z);
        if (captureFirst || captureSecond) {
            capture(captureFirst, x, z);
        }

        boolean calibrated = Double.isFinite(secondZ);
        double position = servo.getPosition();
        telemetry.addData("Calibration", calibration);
        telemetry.addData("Servo command", "%.4f", position);
        if (targetVisible) {
            telemetry.addData("Opening depth (in)", "%.1f", z);
        }
        double error = Double.NaN;
        if (targetVisible && calibrated) {
            double slope = (secondX - firstX) / (secondZ - firstZ);
            double alignedX = firstX + slope * (z - firstZ);
            error = Math.toDegrees(Math.atan2(x, z) - Math.atan2(alignedX, z));
            telemetry.addData("Tracking error (deg)", "%+.2f", error);
        }
        String status = "Holding";
        boolean manual = gamepad1.b || gamepad1.x;
        if (!(position >= MIN_POSITION && position <= MAX_POSITION)) {
            status = "Holding: servo command outside 0.32 to 0.38";
        } else if (manual) {
            status = "Manual";
            if (gamepad1.b != gamepad1.x) {
                step(position, gamepad1.b ? MAX_STEP : -MAX_STEP);
            }
        } else if (gamepad1.left_bumper && calibrated && !captureFirst && !captureSecond) {
            if (!targetVisible) {
                status = "Holding: target lost";
            } else if (Math.abs(error) <= DEADBAND_DEGREES) {
                status = "Holding: within 1 degree";
            } else {
                status = "Tracking";
                if (frame.getTimestamp() != lastFrame
                        && step(position, -Math.copySign(MAX_STEP, error))) {
                    lastFrame = frame.getTimestamp();
                }
            }
        }
        telemetry.addData("Turret", status);
        telemetry.addLine("X/B manual. Hold LB to track. Flywheel off.");
    }

    private void capture(boolean first, double x, double z) {
        if (!Double.isFinite(z)) {
            calibration = "Show the upright opening before capturing.";
        } else if (first) {
            firstX = x;
            firstZ = z;
            secondZ = Double.NaN;
            calibration = "First saved. Re-aim at a distance at least 12 inches different, then press Y.";
        } else if (!Double.isFinite(firstZ) || Math.abs(z - firstZ) < 12) {
            calibration = "Save A first. A and Y need at least 12 inches of depth difference.";
        } else {
            secondX = x;
            secondZ = z;
            calibration = "Two points saved. Release LB to hold; X/B overrides tracking.";
        }
    }

    private boolean step(double current, double delta) {
        long now = System.nanoTime();
        if (now < nextStep) {
            return false;
        }
        nextStep = now + STEP_INTERVAL_NANOS;
        double next = Range.clip(current + delta, MIN_POSITION, MAX_POSITION);
        if (next != current) {
            servo.setPosition(next);
        }
        return true;
    }

    @Override
    public void stop() {
        super.stop();
        if (flywheel != null) {
            flywheel.setPower(0);
        }
    }
}
