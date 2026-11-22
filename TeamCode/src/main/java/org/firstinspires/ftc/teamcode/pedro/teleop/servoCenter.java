package org.firstinspires.ftc.teamcode.pedro.teleop;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.PwmControl;
import com.qualcomm.robotcore.hardware.ServoImplEx;

@TeleOp(name = "center speed Servo", group = "Calibration")
public class servoCenter extends LinearOpMode {

    private ServoImplEx blocker;

    @Override
    public void runOpMode() {
        blocker = hardwareMap.get(ServoImplEx.class, "blocker");
        blocker.setPwmRange(new PwmControl.PwmRange(500, 2500));
        telemetry.addLine("Press PLAY to move servo to exact 0.5 center (2.5 turns).");
        telemetry.update();
        waitForStart();
        blocker.setPosition(0.5);

        while (opModeIsActive()) {
            telemetry.addData("Target Position", blocker.getPosition());
            telemetry.addLine("Servo is now centered. Power off robot and attach horn.");
            telemetry.update();
        }
    }
}