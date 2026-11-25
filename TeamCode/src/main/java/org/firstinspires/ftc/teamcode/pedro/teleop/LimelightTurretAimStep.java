package org.firstinspires.ftc.teamcode.pedro.teleop;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes.FiducialResult;
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
    private static final double DEADBAND_DEGREES = 0.5;
    private static final double GAIN = 0.001;
    private static final double MAX_STEP = 0.005;
    private static final double MIN_SERVO = 0.32;
    private static final double MAX_SERVO = 0.38;
    private static final long SETTLE_NANOS = 400_000_000L;

    private final LimelightFrames frames = new LimelightFrames();
    private Limelight3A limelight;
    private ServoImplEx turretServo;
    private double referenceTx = Double.NaN;
    private double startingTx = Double.NaN;
    private double settledTx = Double.NaN;
    private double fromPosition;
    private double toPosition;
    private double commandFrame;
    private long commandNanos;
    private int increasingCommandTurns = -1;
    private boolean previousA;
    private boolean previousB;
    private boolean moved;
    private String status = "Line up the shooter, then press A.";

    @Override
    public void init() {
        limelight = hardwareMap.get(Limelight3A.class, Constants.LIMELIGHT_NAME);
        limelight.pipelineSwitch(LimelightFrames.PIPELINE);
        turretServo = hardwareMap.tryGet(ServoImplEx.class, Constants.TURRET_ROTATION_SERVO_NAME);
        telemetry.addLine("Keep the flywheel off and verify clearance and cable slack.");
        telemetry.addLine("PLAY starts telemetry. The turret holds until you request a step.");
    }

    @Override
    public void start() {
        limelight.start();
        previousA = gamepad1.a;
        previousB = gamepad1.b;
    }

    @Override
    public void loop() {
        LLResult result = frames.freshResult(limelight);
        double tx = tagTx(result);
        if (!moved) {
            if (gamepad1.dpad_left && !gamepad1.dpad_right) {
                increasingCommandTurns = -1;
            } else if (gamepad1.dpad_right && !gamepad1.dpad_left) {
                increasingCommandTurns = 1;
            }
            if (gamepad1.a && !previousA) {
                if (Double.isFinite(tx)) {
                    referenceTx = tx;
                    status = "Reference captured. Turn the chassis slightly and hold it still.";
                } else {
                    status = "Show a fresh tag 42 detection before capturing a reference.";
                }
            } else if (gamepad1.b && !previousB && gamepad1.right_bumper) {
                requestStep(result, tx);
            }
        } else if (!Double.isFinite(settledTx) && Double.isFinite(tx)
                && result.getTimestamp() != commandFrame
                && System.nanoTime() - commandNanos >= SETTLE_NANOS) {
            settledTx = tx;
            status = "Settled sample captured. Compare it with the starting tx.";
        }
        previousA = gamepad1.a;
        previousB = gamepad1.b;

        telemetry.addData("Status", status);
        telemetry.addLine("A captures reference. Hold RB and press B for one step.");
        telemetry.addData("Increasing servo command turns", increasingCommandTurns < 0 ? "Left" : "Right");
        telemetry.addLine("D-pad left/right changes the assumed turn direction.");
        telemetry.addData("Fresh tag 42 tx (deg)", formatAngle(tx));
        if (result != null) {
            telemetry.addData("Result age (ms)", result.getStaleness());
        }
        telemetry.addData("Reference tx (deg)", formatAngle(referenceTx));
        if (turretServo != null) {
            telemetry.addData("Current servo command", "%.4f", turretServo.getPosition());
        }
        if (moved) {
            telemetry.addData("Starting tx (deg)", formatAngle(startingTx));
            telemetry.addData("Servo command", "%.4f to %.4f (%+.4f)",
                    fromPosition, toPosition, toPosition - fromPosition);
            telemetry.addData("Settled tx (deg)", formatAngle(settledTx));
            if (Double.isFinite(settledTx)) {
                telemetry.addData("Absolute error before and after (deg)", "%.2f to %.2f",
                        Math.abs(startingTx - referenceTx), Math.abs(settledTx - referenceTx));
            }
        }
        telemetry.update();
    }

    private void requestStep(LLResult result, double tx) {
        if (!Double.isFinite(referenceTx) || !Double.isFinite(tx)) {
            status = "A captured reference and fresh tag 42 are required.";
            return;
        }
        double error = tx - referenceTx;
        if (Math.abs(error) <= DEADBAND_DEGREES) {
            status = "Already within the 0.5 degree deadband.";
            return;
        }
        if (turretServo == null) {
            status = "turretServo is missing from the robot configuration.";
            return;
        }
        fromPosition = turretServo.getPosition();
        if (!(fromPosition >= MIN_SERVO && fromPosition <= MAX_SERVO)) {
            status = "Current servo command is outside 0.32 to 0.38.";
            return;
        }
        double step = Range.clip(increasingCommandTurns * GAIN * error, -MAX_STEP, MAX_STEP);
        toPosition = Range.clip(fromPosition + step, MIN_SERVO, MAX_SERVO);
        if (toPosition == fromPosition) {
            status = "Already at the travel limit in that direction.";
            return;
        }

        turretServo.setPwmRange(new PwmControl.PwmRange(500, 2500));
        turretServo.setPosition(toPosition);
        startingTx = tx;
        commandFrame = result.getTimestamp();
        commandNanos = System.nanoTime();
        moved = true;
        status = "Step commanded. Hold the chassis still for the settled reading.";
    }

    private static double tagTx(LLResult result) {
        if (result != null) {
            for (FiducialResult tag : result.getFiducialResults()) {
                if (tag.getFiducialId() == TAG_ID) {
                    return tag.getTargetXDegrees();
                }
            }
        }
        return Double.NaN;
    }

    private static String formatAngle(double angle) {
        return Double.isFinite(angle) ? String.format(java.util.Locale.US, "%+.2f", angle) : "—";
    }

    @Override
    public void stop() {
        if (limelight != null) {
            limelight.stop();
        }
    }
}
