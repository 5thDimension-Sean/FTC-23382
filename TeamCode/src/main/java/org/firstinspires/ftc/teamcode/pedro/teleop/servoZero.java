package org.firstinspires.ftc.teamcode.pedro.teleop;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.PwmControl;
import com.qualcomm.robotcore.hardware.ServoImplEx;

@TeleOp(name = "zero 5-Turn Servo", group = "Calibration")
public class servoZero extends LinearOpMode {

    private ServoImplEx loaderLeft;
    private ServoImplEx loaderRight;

    @Override
    public void runOpMode() {
        loaderLeft = hardwareMap.get(ServoImplEx.class, "loaderLeft");
        loaderRight = hardwareMap.get(ServoImplEx.class, "loaderRight");
        loaderLeft.setPwmRange(new PwmControl.PwmRange(500, 2500));
        loaderRight.setPwmRange(new PwmControl.PwmRange(500, 2500));
        telemetry.addLine("Press PLAY to move servo to exact 0.5 center (2.5 turns).");
        telemetry.update();
        waitForStart();

        loaderLeft.setPosition(1.0);
        loaderRight.setPosition(0.0);

        while (opModeIsActive()) {
            telemetry.addData("Target Position", loaderLeft.getPosition());
            telemetry.addData("Target Position", loaderRight.getPosition());
            telemetry.addLine("Servo is now centered. Power off robot and attach horn.");
            telemetry.update();
        }
    }
}