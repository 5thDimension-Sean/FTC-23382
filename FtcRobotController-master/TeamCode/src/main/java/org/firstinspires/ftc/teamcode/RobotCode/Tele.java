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

        }
    }
}
