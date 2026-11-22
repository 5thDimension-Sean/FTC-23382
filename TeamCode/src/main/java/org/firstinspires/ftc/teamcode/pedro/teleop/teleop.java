package org.firstinspires.ftc.teamcode.pedro.teleop;

import com.pedropathing.follower.Follower;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.pedro.Constants;
import org.firstinspires.ftc.teamcode.pedro.subsystem.intake;
import org.firstinspires.ftc.teamcode.pedro.subsystem.turret;

@TeleOp(name = "23382 Teleop")
public class teleop extends OpMode {

    private Follower follower;
    intake i = new intake();
    turret t = new turret();

    @Override
    public void init() {
        follower = Constants.create(hardwareMap);
        //i.init(hardwareMap);
        t.init(hardwareMap);
    }

    @Override
    public void loop() {
        follower.manual(
                -gamepad1.left_stick_y,
                -gamepad1.left_stick_x,
                -gamepad1.right_stick_x
        );
        if (gamepad1.left_bumper) {
            t.setPower(0.75);
        } else if (gamepad1.right_bumper) {
            t.setPower(0.5);
        } else {
            t.setPower(0.0);
        }

        follower.update();
    }
}