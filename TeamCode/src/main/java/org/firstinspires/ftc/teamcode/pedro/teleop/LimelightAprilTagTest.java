package org.firstinspires.ftc.teamcode.pedro.teleop;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.pedro.Constants;

@TeleOp(name = "Limelight AprilTag Test", group = "Calibration")
public class LimelightAprilTagTest extends OpMode {
    private static final int PIPELINE = 0;
    private static final long MAX_RESULT_AGE_MS = 250;

    private Limelight3A limelight;

    @Override
    public void init() {
        limelight = hardwareMap.get(Limelight3A.class, Constants.LIMELIGHT_NAME);
        limelight.pipelineSwitch(PIPELINE);
        telemetry.addData("Pipeline", PIPELINE);
        telemetry.addLine("Press PLAY to read AprilTags.");
        telemetry.update();
    }

    @Override
    public void start() {
        limelight.start();
    }

    @Override
    public void loop() {
        LLResult result = limelight.getLatestResult();
        if (result == null) {
            telemetry.addData("Target", "Waiting for camera data");
        } else {

            long ageMs = result.getStaleness();
            telemetry.addData("Result age (ms)", ageMs);

            if (ageMs > MAX_RESULT_AGE_MS) {

                telemetry.addData("Target", "Camera data is stale");

            } else if (!result.isValid() || result.getFiducialResults().isEmpty()) {

                telemetry.addData("Target", "No AprilTag visible");

            } else {

                telemetry.addData("Target", "AprilTag detected");
                for (LLResultTypes.FiducialResult tag : result.getFiducialResults()) {
                    telemetry.addData("Tag " + tag.getFiducialId(), "tx %.2f°, ty %.2f°",
                            tag.getTargetXDegrees(), tag.getTargetYDegrees());
                }
            }
        }
        telemetry.update();
    }

    @Override
    public void stop() {
        if (limelight != null) {
            limelight.stop();
        }
    }
}
