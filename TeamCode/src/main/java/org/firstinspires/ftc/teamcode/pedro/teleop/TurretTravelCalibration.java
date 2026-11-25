package org.firstinspires.ftc.teamcode.pedro.teleop;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.PwmControl;
import com.qualcomm.robotcore.hardware.ServoImplEx;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.pedro.Constants;

@TeleOp(name = "Turret Travel Calibration", group = "Calibration")
public class TurretTravelCalibration extends OpMode {
    private static final double MEASURED_CENTER = 0.4333;
    private static final double FINE_STEP = 0.001;
    private static final double COARSE_STEP = 0.005;

    private ServoImplEx servo;
    private boolean previousDecrease, previousIncrease, previousCenter, previousLeft, previousRight;
    private boolean previousRecovery;
    private double center = Double.NaN;
    private double left = Double.NaN;
    private double right = Double.NaN;
    private String status = "Check the current position before jogging.";

    @Override
    public void init() {
        servo = hardwareMap.get(ServoImplEx.class, Constants.TURRET_ROTATION_SERVO_NAME);
        DcMotorEx flywheel = hardwareMap.tryGet(DcMotorEx.class, Constants.TURRET_FLYWHEEL_MOTOR_NAME);
        if (flywheel != null) {
            flywheel.setPower(0);
        }
        telemetry.addLine("PLAY displays the position. Motion requires LB and a fresh D-pad press.");
    }

    @Override
    public void start() {
        rememberButtons();
    }

    @Override
    public void loop() {
        boolean decrease = gamepad1.dpad_left && !previousDecrease;
        boolean increase = gamepad1.dpad_right && !previousIncrease;
        boolean recover = gamepad1.left_bumper && gamepad1.right_bumper && gamepad1.y;
        if (recover && !previousRecovery && !gamepad1.dpad_left && !gamepad1.dpad_right) {
            commandPosition(MEASURED_CENTER);
            status = "Measured center commanded.";
        } else if (!recover && gamepad1.left_bumper && decrease != increase
                && gamepad1.dpad_left != gamepad1.dpad_right) {
            jog(increase);
        }

        double command = servo.getPosition();
        if (!gamepad1.left_bumper && validCommand(command)) {
            if (gamepad1.a && !previousCenter) {
                center = command;
                left = right = Double.NaN;
                status = "Straight ahead marked.";
            } else if (gamepad1.x && !previousLeft) {
                left = command;
                status = "Physical left endpoint marked.";
            } else if (gamepad1.b && !previousRight) {
                right = command;
                status = "Physical right endpoint marked.";
            }
        }
        rememberButtons();

        telemetry.addData("Status", status);
        telemetry.addData("Servo command", "%.4f", command);
        telemetry.addData("Straight ahead", formatCommand(center));
        telemetry.addData("Physical left / right", "%s / %s", formatCommand(left), formatCommand(right));
        if (Double.isFinite(left) && Double.isFinite(right)) {
            telemetry.addData("Measured command limits", "%.4f to %.4f",
                    Math.min(left, right), Math.max(left, right));
        }
        telemetry.addLine("Hold LB. Tap D-pad left/right for one decrease/increase.");
        telemetry.addLine("Hold RB too for 0.005 steps. Fine steps are 0.001.");
        telemetry.addLine("Release LB. A marks straight ahead, X marks left, B marks right.");
        telemetry.addLine("Only if the shooter is already straight: hold LB+RB and tap Y to restore center.");
        telemetry.update();
    }

    private void jog(boolean increase) {
        double current = servo.getPosition();
        if (!validCommand(current)) {
            status = "Servo command unavailable.";
            return;
        }
        double step = gamepad1.right_bumper ? COARSE_STEP : FINE_STEP;
        double next = Range.clip(current + (increase ? step : -step), 0.0, 1.0);
        if (next == current) {
            status = "At the servo command boundary.";
            return;
        }
        commandPosition(next);
        status = "One step commanded. Check clearance before the next step.";
    }

    private void commandPosition(double position) {
        servo.setPwmRange(new PwmControl.PwmRange(500, 2500));
        servo.setPosition(position);
    }

    private static boolean validCommand(double command) {
        return command >= 0.0 && command <= 1.0;
    }

    private static String formatCommand(double command) {
        return Double.isFinite(command) ? String.format(java.util.Locale.US, "%.4f", command)
                : "Unmarked";
    }

    private void rememberButtons() {
        previousDecrease = gamepad1.dpad_left;
        previousIncrease = gamepad1.dpad_right;
        previousCenter = gamepad1.a;
        previousLeft = gamepad1.x;
        previousRight = gamepad1.b;
        previousRecovery = gamepad1.left_bumper && gamepad1.right_bumper && gamepad1.y;
    }
}
