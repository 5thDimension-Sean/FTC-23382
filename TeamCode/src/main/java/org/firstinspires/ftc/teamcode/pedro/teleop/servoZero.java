package org.firstinspires.ftc.teamcode.pedro.teleop;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.PwmControl;
import com.qualcomm.robotcore.hardware.ServoImplEx;

@TeleOp(name = "Zero 5-Turn Servo", group = "Calibration")
public class servoZero extends LinearOpMode {

    private ServoImplEx fiveTurnServo;

    @Override
    public void runOpMode() {
        fiveTurnServo = hardwareMap.get(ServoImplEx.class, "turretServo");
        fiveTurnServo.setPwmRange(new PwmControl.PwmRange(500, 2500));
        telemetry.addLine("Press PLAY to move servo to exact 0.5 center (2.5 turns).");
        telemetry.update();
        waitForStart();

        fiveTurnServo.setPosition(0.5);

        while (opModeIsActive()) {
            telemetry.addData("Target Position", fiveTurnServo.getPosition());
            telemetry.addLine("Servo is now centered. Power off robot and attach horn.");
            telemetry.update();
        }
    }
}