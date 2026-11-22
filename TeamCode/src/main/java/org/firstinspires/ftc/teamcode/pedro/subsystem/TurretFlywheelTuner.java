package org.firstinspires.ftc.teamcode.pedro.subsystem;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

import org.firstinspires.ftc.teamcode.pedro.Constants;

@TeleOp(name = "Turret Flywheel PIDF Tuner", group = "Calibration")
public class TurretFlywheelTuner extends OpMode {
    private static final double MAX_TEST_TICKS_PER_SECOND =
            Constants.TURRET_FLYWHEEL_MAX_TEST_RPM * Constants.TURRET_FLYWHEEL_TICKS_PER_REVOLUTION / 60.0;

    private final String[] gainNames = {"P", "I", "D", "F"};
    private final double[] gains = {
            Constants.TURRET_FLYWHEEL_KP,
            Constants.TURRET_FLYWHEEL_KI,
            Constants.TURRET_FLYWHEEL_KD,
            Constants.TURRET_FLYWHEEL_KF
    };
    private DcMotorEx flywheel;
    private DcMotor.RunMode previousMode;
    private PIDFCoefficients previousGains;
    private int selectedGain;
    private boolean previousRight;
    private boolean previousLeft;
    private boolean previousUp;
    private boolean previousDown;

    @Override
    public void init() {
        flywheel = hardwareMap.get(DcMotorEx.class, Constants.TURRET_FLYWHEEL_MOTOR_NAME);
        previousMode = flywheel.getMode();
        previousGains = flywheel.getPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER);
        flywheel.setPower(0);
        flywheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        applyGains();
        telemetry.addData("Flywheel motor", Constants.TURRET_FLYWHEEL_MOTOR_NAME);
        telemetry.update();
    }

    @Override
    public void loop() {
        if (gamepad1.dpad_right && !previousRight) {
            selectedGain = (selectedGain + 1) % gains.length;
        }
        if (gamepad1.dpad_left && !previousLeft) {
            selectedGain = (selectedGain + gains.length - 1) % gains.length;
        }
        boolean increase = gamepad1.dpad_up && !previousUp;
        boolean decrease = gamepad1.dpad_down && !previousDown;
        if (increase != decrease) {
            double step = gamepad1.left_bumper ? 1.0 : 0.1;
            gains[selectedGain] = Math.max(0, gains[selectedGain] + (increase ? step : -step));
            applyGains();
        }
        previousRight = gamepad1.dpad_right;
        previousLeft = gamepad1.dpad_left;
        previousUp = gamepad1.dpad_up;
        previousDown = gamepad1.dpad_down;

        double target = gamepad1.right_bumper
                ? MAX_TEST_TICKS_PER_SECOND * gamepad1.right_trigger : 0;
        if (target > 0) {
            flywheel.setVelocity(target);
        } else {
            flywheel.setPower(0);
        }
        double measured = flywheel.getVelocity();
        telemetry.addLine("Hold RB and squeeze RT to run. Release either to stop.");
        telemetry.addLine("D-pad left/right selects gain, up/down adjusts. Hold LB for larger steps.");
        telemetry.addData("Selected gain", gainNames[selectedGain]);
        telemetry.addData("TURRET_FLYWHEEL_KP", gains[0]);
        telemetry.addData("TURRET_FLYWHEEL_KI", gains[1]);
        telemetry.addData("TURRET_FLYWHEEL_KD", gains[2]);
        telemetry.addData("TURRET_FLYWHEEL_KF", gains[3]);
        telemetry.addData("Target (ticks/s)", "%.1f", target);
        telemetry.addData("Measured (ticks/s)", "%.1f", measured);
        telemetry.addData("Error (ticks/s)", "%.1f", target - measured);
        telemetry.update();
    }

    private void applyGains() {
        flywheel.setVelocityPIDFCoefficients(gains[0], gains[1], gains[2], gains[3]);
    }

    @Override
    public void stop() {
        if (flywheel != null) {
            flywheel.setPower(0);
            if (previousGains != null) {
                flywheel.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, previousGains);
            }
            if (previousMode != null) {
                flywheel.setMode(previousMode);
            }
        }
    }
}
