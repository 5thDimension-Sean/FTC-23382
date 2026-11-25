package org.firstinspires.ftc.teamcode.pedro.teleop;

import android.content.Context;
import android.content.SharedPreferences;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.robotcore.hardware.ServoImplEx;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;

final class HiveTurretAim {
    private static final double MIN_POSITION = 0.386;
    private static final double MAX_POSITION = 0.624;
    private static final double POSITION_TOLERANCE = 0.00005;
    private static final double FINE_STEP = 0.001;
    private static final double MAX_TRACKING_STEP = 0.003;
    private static final double TRACKING_GAIN = 0.00045;
    private static final double DEADBAND_DEGREES = 1.0;
    private static final long STEP_INTERVAL_NANOS = 60_000_000L;
    private static final long FAST_STEP_INTERVAL_NANOS = 45_000_000L;
    static final double MIN_CALIBRATION_DEPTH_DIFFERENCE = 12;

    private final ServoImplEx servo;
    private final SharedPreferences preferences;
    private double slope;
    private double intercept;
    private double lastFrame = Double.NaN;
    private long nextStep;

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
            boolean requested, boolean manual, Telemetry telemetry) {
        double error = Double.NaN;
        if (opening != null) {
            telemetry.addData("Opening depth (in)", "%.1f", opening.z);
            if (isCalibrated()) {
                double alignedX = slope * opening.z + intercept;
                error = Math.toDegrees(Math.atan2(opening.x, opening.z)
                        - Math.atan2(alignedX, opening.z));
                telemetry.addData("Tracking error (deg)", "%+.2f", error);
            }
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
        if (Math.abs(error) <= DEADBAND_DEGREES) {
            return "Holding: within 1 degree";
        }
        double magnitude = Math.abs(error);
        boolean coarse = magnitude >= 4.0;
        double step = coarse ? Range.clip(magnitude * TRACKING_GAIN, FINE_STEP, MAX_TRACKING_STEP)
                : Range.clip(magnitude * 0.00035, FINE_STEP, MAX_TRACKING_STEP);
        long interval = coarse ? FAST_STEP_INTERVAL_NANOS : STEP_INTERVAL_NANOS;
        double next = Range.clip(position - Math.copySign(step, error), MIN_POSITION, MAX_POSITION);
        if (Math.abs(next - position) <= POSITION_TOLERANCE) {
            return "At travel limit";
        }
        if (frame.getTimestamp() != lastFrame && sendStep(next, interval)) {
            lastFrame = frame.getTimestamp();
        }
        return "Tracking";
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
