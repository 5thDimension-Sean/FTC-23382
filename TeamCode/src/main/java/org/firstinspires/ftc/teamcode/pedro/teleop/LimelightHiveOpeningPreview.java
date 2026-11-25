package org.firstinspires.ftc.teamcode.pedro.teleop;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.pedro.Constants;

@TeleOp(name = "Limelight Hive Opening Preview", group = "Calibration")
public class LimelightHiveOpeningPreview extends OpMode {
    private final LimelightFrames frames = new LimelightFrames();
    private Limelight3A limelight;
    private boolean blue = true;

    @Override
    public void init() {
        limelight = hardwareMap.get(Limelight3A.class, Constants.LIMELIGHT_NAME);
        limelight.pipelineSwitch(LimelightFrames.PIPELINE);
        telemetry.addLine("PLAY starts the camera. D-pad left selects Red, right selects Blue.");
    }

    @Override
    public void start() {
        limelight.start();
    }

    @Override
    public void loop() {
        if (gamepad1.dpad_left != gamepad1.dpad_right) {
            blue = gamepad1.dpad_right;
        }
        telemetry.addData("Alliance", blue ? "Blue" : "Red");
        LLResult frame = frames.freshResult(limelight);
        HiveOpeningVision.Opening opening = HiveOpeningVision.find(frame, blue, telemetry, true);
        handleOpening(frame, opening);
        telemetry.update();
    }

    protected void handleOpening(LLResult frame, HiveOpeningVision.Opening opening) {
    }

    @Override
    public void stop() {
        if (limelight != null) {
            limelight.stop();
        }
    }
}
