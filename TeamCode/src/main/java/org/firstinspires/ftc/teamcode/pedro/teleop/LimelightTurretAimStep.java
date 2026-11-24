package org.firstinspires.ftc.teamcode.pedro.teleop;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.PwmControl;
import com.qualcomm.robotcore.hardware.ServoImplEx;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.pedro.Constants;

@TeleOp(name = "Limelight Turret Aim Step", group = "Calibration")
public class LimelightTurretAimStep extends OpMode {
    private static final int TAG_ID = 42;
    private static final double ALIGNED_TX = -5.23;
    private static final double DEADBAND_DEGREES = 0.5;
    private static final long MAX_AGE_MS = 100;
    // A larger servo command is expected to increase tx, per the Square/Circle report.
    private static final int SERVO_DIRECTION = -1;
    private static final double GAIN = 0.001;
    private static final double MAX_STEP = 0.005;
    // Verified clearance for the one-step test around the shooting position.
    private static final double MIN_SERVO = 0.32;
    private static final double MAX_SERVO = 0.38;

    private Limelight3A limelight;
    private ServoImplEx turretServo;
    private boolean sampled;

    @Override
    public void init() {
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.pipelineSwitch(0);
        turretServo = hardwareMap.tryGet(ServoImplEx.class, Constants.TURRET_ROTATION_SERVO_NAME);
        if (turretServo != null) {
            // Use the same pulse range as TeleOp, where the limits were measured.
            turretServo.setPwmRange(new PwmControl.PwmRange(500, 2500));
            telemetry.addData("Servo command at INIT", "%.5f", turretServo.getPosition())
                    .setRetained(true);
        }
        telemetry.addLine("PLAY captures one correction. Live tx continues updating.");
        telemetry.update();
    }

    @Override
    public void start() {
        limelight.start();
    }

    @Override
    public void loop() {
        telemetry.addData("Reference tx", "%.2f degrees", ALIGNED_TX);
        LLResult result = limelight.getLatestResult();
        if (result != null) {
            telemetry.addData("Result age (ms)", result.getStaleness());
        }
        if (result == null || !result.isValid() || result.getStaleness() > MAX_AGE_MS) {
            telemetry.addLine("Waiting for a fresh detection");
        } else {
            showTag(result);
        }
        telemetry.update();
    }

    private void showTag(LLResult result) {
        for (LLResultTypes.FiducialResult tag : result.getFiducialResults()) {
            if (tag.getFiducialId() == TAG_ID && Double.isFinite(tag.getTargetXDegrees())) {
                double tx = tag.getTargetXDegrees();
                telemetry.addData("Tag 42 live tx", "%.2f degrees", tx);
                telemetry.addData("Live error", "%.2f degrees", tx - ALIGNED_TX);
                if (!sampled) {
                    sampled = true;
                    telemetry.addData("Starting tx", "%.2f degrees", tx).setRetained(true);
                    telemetry.addData("Aim step", proposeStep(tx)).setRetained(true);
                }
                return;
            }
        }
        telemetry.addLine("Tag 42 not visible");
    }

    private String proposeStep(double tx) {
        double error = tx - ALIGNED_TX;
        if (Math.abs(error) <= DEADBAND_DEGREES) {
            return "Already within deadband";
        }
        double step = Range.clip(SERVO_DIRECTION * GAIN * error, -MAX_STEP, MAX_STEP);
        if (!(0.0 <= MIN_SERVO && MIN_SERVO < MAX_SERVO && MAX_SERVO <= 1.0)) {
            return String.format(java.util.Locale.US,
                    "Preview: %+.4f servo units. Set measured limits to enable motion.", step);
        }
        if (turretServo == null) {
            return "Holding: turretServo is missing from the robot configuration";
        }
        double current = turretServo.getPosition();
        if (!(current >= MIN_SERVO && current <= MAX_SERVO)) {
            return String.format(java.util.Locale.US,
                    "Holding: servo %.5f is outside %.4f to %.4f",
                    current, MIN_SERVO, MAX_SERVO);
        }
        double next = Range.clip(current + step, MIN_SERVO, MAX_SERVO);
        if (next == current) {
            return "Holding: turret is at its configured limit";
        }
        turretServo.setPosition(next);
        return String.format(java.util.Locale.US, "Commanded %.4f to %.4f", current, next);
    }

    @Override
    public void stop() {
        if (limelight != null) {
            limelight.stop();
        }
    }
}
