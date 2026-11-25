package org.firstinspires.ftc.teamcode.pedro.teleop;

import android.content.Context;
import android.content.SharedPreferences;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.robotcore.hardware.ServoImplEx;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;

final class HiveTurretAim {
    static final double STRAIGHT_AHEAD_POSITION = 0.5000;
    private static final double MIN_POSITION = 0.386;
    private static final double MAX_POSITION = 0.624;
    private static final double POSITION_TOLERANCE = 0.00005;
    private static final double FINE_STEP = 0.001;
    private static final double MAX_TRACKING_STEP = 0.0035;
    private static final double TRACKING_GAIN = 0.00055;
    private static final double STOP_ERROR_DEGREES = 0.8;
    private static final double START_ERROR_DEGREES = 1.6;
    private static final double NEAR_STEP = 0.0005;
    private static final long STEP_INTERVAL_NANOS = 60_000_000L;
    private static final long FAST_STEP_INTERVAL_NANOS = 35_000_000L;
    private static final double DRIVING_STOP_ERROR_DEGREES = 0.6;
    private static final double DRIVING_START_ERROR_DEGREES = 1.1;
    private static final long DRIVING_STEP_INTERVAL_NANOS = 24_000_000L;
    private static final double NEAR_GAIN = 0.0002;
    private static final long DRIVING_FINE_SETTLE_NANOS = 60_000_000L;
    private static final long STATIONARY_FINE_SETTLE_NANOS = 100_000_000L;
    private static final double DRIVING_MAX_STEP = 0.005;
    private static final double DRIVING_LARGE_ERROR_GAIN = 0.00080;
    private static final long DRIVE_RELEASE_DELAY_NANOS = 150_000_000L;
    static final double MIN_CALIBRATION_DEPTH_DIFFERENCE = 12;

    private final ServoImplEx servo;
    private final SharedPreferences preferences;
    private double slope;
    private double intercept;
    private double lastFrame = Double.NaN;
    private long nextStep;
    private boolean correcting;
    private int lastDrivingDirection;
    private int lastCorrectionDirection;
    private double firstReversalFrame = Double.NaN;
    private long lastDriveInputNanos;
    private int filteredCell = -1;
    private double lastEstimateFrame = Double.NaN;
    private double previousError = Double.NaN;
    private double olderError = Double.NaN;
    private double filteredError;
    private int restartFrameCount;
    private int restartDirection;
    private double lastRestartFrame;

    HiveTurretAim(ServoImplEx servo, Context context) {
        this.servo = servo;
        preferences = context.getSharedPreferences("hive_turret_aim_v1", Context.MODE_PRIVATE);
        slope = read("slope");
        intercept = read("intercept");
    }

    private double read(String key) {
        return Double.longBitsToDouble(preferences.getLong(key,
                Double.doubleToLongBits(Double.NaN)));
    }

    boolean isCalibrated() {
        return Double.isFinite(slope) && Double.isFinite(intercept);
    }

    void saveCalibration(HiveOpeningVision.Opening first, HiveOpeningVision.Opening second) {
        slope = (second.x - first.x) / (second.z - first.z);
        intercept = first.x - slope * first.z;
        preferences.edit()
                .putLong("slope", Double.doubleToLongBits(slope))
                .putLong("intercept", Double.doubleToLongBits(intercept))
                .apply();
    }

    String update(LLResult frame, HiveOpeningVision.Opening opening,
            boolean requested, boolean manual, boolean driveInput, Telemetry telemetry) {
        long now = System.nanoTime();
        if (driveInput) {
            lastDriveInputNanos = now;
        }
        boolean driving = driveInput || (lastDriveInputNanos != 0
                && now - lastDriveInputNanos < DRIVE_RELEASE_DELAY_NANOS);
        if (!requested || manual || opening == null) {
            filteredCell = -1;
        }
        double error = Double.NaN;
        if (opening != null) {
            telemetry.addData("Opening depth (in)", "%.1f", opening.z);
            if (isCalibrated()) {
                double alignedX = slope * opening.z + intercept;
                error = Math.toDegrees(Math.atan2(opening.x, opening.z)
                        - Math.atan2(alignedX, opening.z));
                if (requested && !manual) {
                    error = filteredError(opening.cell, frame.getTimestamp(), error);
                }
                telemetry.addData("Tracking error (deg)", "%+.2f", error);
            }
        }
        if (!driving || manual || !requested || opening == null) {
            lastDrivingDirection = 0;
            firstReversalFrame = Double.NaN;
        }
        if (manual || !requested || opening == null) {
            correcting = false;
            lastCorrectionDirection = 0;
            restartFrameCount = 0;
        }
        if (manual) {
            return "Manual";
        }
        if (!requested) {
            return "Holding";
        }
        if (!isCalibrated()) {
            return "Holding: calibrate in Hive Tracking Test";
        }
        if (opening == null) {
            return "Holding: target lost";
        }
        double position = servo.getPosition();
        if (!withinTravel(position)) {
            return String.format(java.util.Locale.US, "Holding: servo command outside %.4f to %.4f",
                    MIN_POSITION, MAX_POSITION);
        }
        double magnitude = Math.abs(error);
        double stopError = driving ? DRIVING_STOP_ERROR_DEGREES : STOP_ERROR_DEGREES;
        double startError = driving ? DRIVING_START_ERROR_DEGREES : START_ERROR_DEGREES;
        int direction = error > 0 ? -1 : 1;
        // After crossing center, hold a small error instead of immediately undoing the last step.
        boolean crossedCenter = lastCorrectionDirection != 0 && direction != lastCorrectionDirection;
        double threshold = correcting && !crossedCenter ? stopError : startError;
        if (magnitude <= threshold) {
            correcting = false;
            firstReversalFrame = Double.NaN;
            restartFrameCount = 0;
            return "Holding: aim settled";
        }
        boolean coarse = magnitude >= 4.0;
        double timestamp = frame.getTimestamp();
        if (!correcting && !driving && !coarse && !confirmSettledRestart(direction, timestamp)) {
            return "Holding: aim settled";
        }
        correcting = true;
        restartFrameCount = 0;
        double step = coarse
                ? Range.clip(magnitude * (driving ? DRIVING_LARGE_ERROR_GAIN : TRACKING_GAIN),
                        FINE_STEP, driving ? DRIVING_MAX_STEP : MAX_TRACKING_STEP)
                : Range.clip(magnitude * NEAR_GAIN, NEAR_STEP, FINE_STEP);
        long interval = coarse
                ? (driving ? DRIVING_STEP_INTERVAL_NANOS : FAST_STEP_INTERVAL_NANOS)
                : (driving ? DRIVING_FINE_SETTLE_NANOS : STATIONARY_FINE_SETTLE_NANOS);
        double next = Range.clip(position + direction * step, MIN_POSITION, MAX_POSITION);
        if (Math.abs(next - position) <= POSITION_TOLERANCE) {
            return "At travel limit";
        }
        if (driving && !confirmDrivingDirection(direction, timestamp)) {
            return "Tracking: confirming direction";
        }
        if (timestamp != lastFrame && sendStep(next, interval)) {
            lastFrame = timestamp;
            lastCorrectionDirection = direction;
            if (driving) {
                lastDrivingDirection = direction;
            }
        }
        return "Tracking";
    }

    private double filteredError(int cell, double timestamp, double measured) {
        if (filteredCell != cell) {
            filteredCell = cell;
            restartFrameCount = 0;
            lastEstimateFrame = Double.NaN;
            previousError = Double.NaN;
            olderError = Double.NaN;
            correcting = false;
            lastCorrectionDirection = 0;
            lastDrivingDirection = 0;
            firstReversalFrame = Double.NaN;
        }
        if (timestamp != lastEstimateFrame) {
            // At most one frame of lag, with isolated jumps rejected after the first two frames.
            double median = Double.isNaN(olderError) ? measured
                    : Math.max(Math.min(measured, previousError),
                            Math.min(Math.max(measured, previousError), olderError));
            boolean consistentlyOffCenter = Math.abs(measured) >= 4.0 && Math.abs(median) >= 4.0
                    && Math.signum(measured) == Math.signum(median);
            filteredError = consistentlyOffCenter ? measured : median;
            olderError = previousError;
            previousError = measured;
            lastEstimateFrame = timestamp;
        }
        return filteredError;
    }

    private boolean confirmSettledRestart(int direction, double timestamp) {
        if (restartFrameCount == 0 || direction != restartDirection) {
            restartDirection = direction;
            restartFrameCount = 1;
            lastRestartFrame = timestamp;
        } else if (timestamp != lastRestartFrame) {
            restartFrameCount++;
            lastRestartFrame = timestamp;
        }
        return restartFrameCount >= 3;
    }

    private boolean confirmDrivingDirection(int direction, double timestamp) {
        if (lastDrivingDirection == 0 || direction == lastDrivingDirection) {
            firstReversalFrame = Double.NaN;
            return true;
        }
        if (Double.isNaN(firstReversalFrame)) {
            firstReversalFrame = timestamp;
            return false;
        }
        return timestamp > firstReversalFrame;
    }

    String nudgeTowardOdometry(double target) {
        double position = servo.getPosition();
        if (!withinTravel(position)) {
            return "Holding: servo command outside travel";
        }
        double boundedTarget = Range.clip(target, MIN_POSITION, MAX_POSITION);
        double delta = boundedTarget - position;
        if (Math.abs(delta) <= POSITION_TOLERANCE) {
            return target == boundedTarget ? "Holding: odometry bearing" : "At travel limit";
        }
        double step = Range.clip(delta, -0.003, 0.003);
        sendStep(Range.clip(position + step, MIN_POSITION, MAX_POSITION), STEP_INTERVAL_NANOS);
        return "Odometry: pointing toward Hive";
    }

    boolean restoreCenterIfNeeded() {
        if (withinTravel(servo.getPosition())) {
            return false;
        }
        servo.setPosition(STRAIGHT_AHEAD_POSITION);
        return true;
    }

    void manualStep(boolean increase) {
        double position = servo.getPosition();
        if (withinTravel(position)) {
            double delta = increase ? FINE_STEP : -FINE_STEP;
            double next = Range.clip(position + delta, MIN_POSITION, MAX_POSITION);
            if (Math.abs(next - position) > POSITION_TOLERANCE) {
                sendStep(next, STEP_INTERVAL_NANOS);
            }
        }
    }

    private static boolean withinTravel(double position) {
        return position >= MIN_POSITION - POSITION_TOLERANCE
                && position <= MAX_POSITION + POSITION_TOLERANCE;
    }

    private boolean sendStep(double next, long interval) {
        long now = System.nanoTime();
        if (now < nextStep) {
            return false;
        }
        nextStep = now + interval;
        servo.setPosition(next);
        return true;
    }
}
