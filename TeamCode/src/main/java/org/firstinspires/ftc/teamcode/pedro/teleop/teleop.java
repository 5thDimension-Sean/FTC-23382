package org.firstinspires.ftc.teamcode.pedro.teleop;

import com.pedropathing.follower.Follower;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.pedro.Constants;
import org.firstinspires.ftc.teamcode.pedro.subsystem.intake;
import org.firstinspires.ftc.teamcode.pedro.subsystem.loader;
import org.firstinspires.ftc.teamcode.pedro.subsystem.turret;

@TeleOp(name = "23382 Teleop")
public class teleop extends OpMode {

    private Follower follower;
    intake intake = new intake();
    turret turret = new turret();
    loader loader = new loader();
    private int pressed = 0;

    @Override
    public void init() {
        follower = Constants.create(hardwareMap);
        //i.init(hardwareMap);
        //t.init(hardwareMap);
        //l.init(hardwareMap);
    }

    @Override
    public void loop() {
        follower.manual(
                -gamepad1.left_stick_y,
                -gamepad1.left_stick_x,
                -gamepad1.right_stick_x
        );
        if(gamepad1.right_trigger != 0){
            intake.setPower(Math.max(gamepad1.right_trigger+0.5, 1));
        } else if(gamepad1.right_bumper){
            intake.setPower(-1);
        }

        if (gamepad1.a && pressed % 2 == 0){
            loader.position("open");
            pressed+=1;
        } else if(gamepad1.a && pressed %2 != 0){
            loader.position("close");
            pressed-=1;
        }

        if(gamepad1.x){
            loader.positionManual(0);
        } else if(gamepad1.y){
            loader.positionManual(1);
        }
        follower.update();
    }
}