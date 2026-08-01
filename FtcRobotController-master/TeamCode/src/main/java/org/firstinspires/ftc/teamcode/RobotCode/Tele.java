package org.firstinspires.ftc.teamcode.RobotCode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.HardwareDefinitions;
import org.firstinspires.ftc.teamcode.pedroPathing.constants.LConstants;

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

            // Debugging Encoders
            telemetry.addData("FL Ticks (Raw | Adj)", "%d | %.1f", robot.fl.getCurrentPosition(), robot.fl.getCurrentPosition() * LConstants.driveEncoderConstants.leftFrontEncoderDirection);
            telemetry.addData("FR Ticks (Raw | Adj)", "%d | %.1f", robot.fr.getCurrentPosition(), robot.fr.getCurrentPosition() * LConstants.driveEncoderConstants.rightFrontEncoderDirection);
            telemetry.addData("BL Ticks (Raw | Adj)", "%d | %.1f", robot.bl.getCurrentPosition(), robot.bl.getCurrentPosition() * LConstants.driveEncoderConstants.leftRearEncoderDirection);
            telemetry.addData("BR Ticks (Raw | Adj)", "%d | %.1f", robot.br.getCurrentPosition(), robot.br.getCurrentPosition() * LConstants.driveEncoderConstants.rightRearEncoderDirection);
            telemetry.addLine("\nMove robot FORWARD. Adjusted values should all be POSITIVE.");
            telemetry.update();
        }
    }
}

