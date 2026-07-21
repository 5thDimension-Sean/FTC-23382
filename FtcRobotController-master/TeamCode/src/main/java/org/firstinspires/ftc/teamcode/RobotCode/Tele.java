package org.firstinspires.ftc.teamcode.RobotCode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.HardwareDefinitions;

@TeleOp(name="23382 Tele-op")
public class Tele extends LinearOpMode {

    HardwareDefinitions robot = new HardwareDefinitions();

    @Override
    public void runOpMode() {
        robot.init(hardwareMap, telemetry);

        waitForStart();

            while (opModeIsActive()) {
                double y = -gamepad1.left_stick_y;
                double x = gamepad1.left_stick_x * 1.1;
                double rx = gamepad1.right_stick_x;

                double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
                double frontLeftPower = (y + x + rx) / denominator;
                double backLeftPower = (y - x + rx) / denominator;
                double frontRightPower = (y - x - rx) / denominator;
                double backRightPower = (y + x - rx) / denominator;

                robot.fl.setPower(frontLeftPower);
                robot.bl.setPower(backLeftPower);
                robot.fr.setPower(frontRightPower);
                robot.br.setPower(backRightPower);

        }
    }
}

