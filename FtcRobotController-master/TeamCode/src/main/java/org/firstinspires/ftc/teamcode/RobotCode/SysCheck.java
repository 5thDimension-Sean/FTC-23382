package org.firstinspires.ftc.teamcode.RobotCode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.hardware.VoltageSensor;

import java.util.ArrayList;
import java.util.List;


@TeleOp(name = "check", group = "5th Dimension")
public class SysCheck extends LinearOpMode {

    // Spinner frames cycled while a check is running: \ | / —
    private static final String[] SPINNER = {"\\", "|", "/", "—"};

    // Voltage thresholds (volts)
    private static final double VOLTAGE_OK = 13.0;
    private static final double VOLTAGE_REMINDER = 12.5;
    private static final double VOLTAGE_ERROR = 12.0;

    // Collected problems. Empty == everything passed.
    private final List<String> errors = new ArrayList<>();
    private final List<String> reminders = new ArrayList<>();

    @Override
    public void runOpMode() {
        telemetry.addLine("5th Dimension Pre-Match System Check");
        telemetry.addLine("Press PLAY to run the checklist.");
        telemetry.update();

        waitForStart();
        if (isStopRequested()) return;

        checkMotor("frontleft");
        checkMotor("frontright");
        checkMotor("backleft");
        checkMotor("backright");

        checkHardware("imu", IMU.class, "IMU");
        checkOptionalHardware("frDist", DistanceSensor.class, "Front Distance Sensor");
        checkOptionalHardware("color_sensor", ColorSensor.class, "Color Sensor");
        checkOptionalHardware("limit_switch", TouchSensor.class, "Limit Switch");

        checkVoltage();

        report();
    }

    /**
     * Verifies a drive motor is present and, as a live sanity check, that its
     * encoder can be read. Animates the spinner while doing so.
     */
    private void checkMotor(String name) {
        animate("Checking motor: " + name);
        try {
            DcMotor motor = hardwareMap.get(DcMotor.class, name);

            motor.getCurrentPosition();
        } catch (Exception e) {
            errors.add("Motor '" + name + "' not found");
        }
    }


    private void checkHardware(String configName, Class<?> type, String label) {
        animate("Checking " + label);
        try {
            hardwareMap.get(type, configName);
        } catch (Exception e) {
            errors.add(label + " not found");
        }
    }

    /**
     * Verifies an optional piece of hardware. If it is not in the config we skip
     * it silently (it may simply not be installed on the robot right now).
     */
    private void checkOptionalHardware(String configName, Class<?> type, String label) {
        animate("Checking " + label);
        try {
            hardwareMap.get(type, configName);
        } catch (Exception e) {
            // Optional device — not configured, so nothing to report.
        }
    }

    /**
     * Reads the lowest battery voltage across all voltage sensors and applies
     * the reminder / error thresholds.
     */
    private void checkVoltage() {
        animate("Checking battery voltage");

        double voltage = Double.MAX_VALUE;
        for (VoltageSensor sensor : hardwareMap.voltageSensor) {
            double v = sensor.getVoltage();
            if (v > 0) {
                voltage = Math.min(voltage, v);
            }
        }

        if (voltage == Double.MAX_VALUE) {
            errors.add("Could not read battery voltage");
            return;
        }

        if (voltage < VOLTAGE_ERROR) {
            errors.add(String.format("Battery critically low (%.2f V)", voltage));
        } else if (voltage <= VOLTAGE_REMINDER) {
            reminders.add(String.format("Battery getting low (%.2f V) - swap soon", voltage));
        }
        // voltage >= VOLTAGE_OK (or between reminder and ok)  no message
    }

    /**
     * Spins the loading animation for a short, visible beat while a check runs.
     */
    private void animate(String label) {
        for (int i = 0; i < SPINNER.length && opModeIsActive(); i++) {
            telemetry.addData("Status", "%s %s", SPINNER[i], label);
            telemetry.update();
            sleep(120);
        }
    }

    /**
     * Prints the final verdict and holds it on screen until the OpMode stops.
     */
    private void report() {
        telemetry.clearAll();

        if (errors.isEmpty()) {
            telemetry.addLine("Go 5th Dimension");
            for (String reminder : reminders) {
                telemetry.addData("Reminder", reminder);
            }
        } else {
            for (String error : errors) {
                telemetry.addData("Error", error);
            }
            for (String reminder : reminders) {
                telemetry.addData("Reminder", reminder);
            }
        }
        telemetry.update();

        // Keep the result visible until the driver stops the OpMode.
        while (opModeIsActive()) {
            sleep(100);
        }
    }
}
