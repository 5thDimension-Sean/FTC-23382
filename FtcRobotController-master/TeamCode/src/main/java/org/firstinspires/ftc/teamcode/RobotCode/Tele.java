package org.firstinspires.ftc.teamcode.RobotCode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.HardwareDefinitions;

@TeleOp(name="My TeleOp")
public class Tele extends LinearOpMode {

    // 1. Correct instantiation syntax
    HardwareDefinitions robot = new HardwareDefinitions();

    @Override
    public void runOpMode() {
        // 2. Pass both hardwareMap and telemetry down to the class
        robot.init(hardwareMap, telemetry);

        waitForStart();

        while (opModeIsActive()) {
            // Your main robot loop code here
        }
    }
}
