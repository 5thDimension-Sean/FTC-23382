package org.firstinspires.ftc.teamcode.RobotCode.tuning;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import com.pedropathing.follower.Follower;

import org.firstinspires.ftc.teamcode.RobotCode.Tuning;
import org.firstinspires.ftc.teamcode.RobotCode.constants.FConstants;

@TeleOp(name = "Localization Test", group = "Pedro Tuning")
public class LocalizationTest extends OpMode {
    private Follower follower;
    private DcMotorEx fl, fr, bl, br;

    @Override
    public void init() {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        Tuning.init();

        follower = FConstants.createFollower(hardwareMap);

        fl = hardwareMap.get(DcMotorEx.class, "frontleft");
        fr = hardwareMap.get(DcMotorEx.class, "frontright");
        bl = hardwareMap.get(DcMotorEx.class, "backleft");
        br = hardwareMap.get(DcMotorEx.class, "backright");

        fl.setDirection(DcMotorSimple.Direction.REVERSE);
        bl.setDirection(DcMotorSimple.Direction.REVERSE);

        for (DcMotorEx m : new DcMotorEx[]{fl, fr, bl, br}) {
            m.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
            m.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }
    }

    @Override
    public void loop() {
        follower.update();

        double y  = -gamepad1.left_stick_y;
        double x  =  gamepad1.left_stick_x * 1.1;
        double rx =  gamepad1.right_stick_x;
        double d  = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);

        fl.setPower((y + x + rx) / d);
        bl.setPower((y - x + rx) / d);
        fr.setPower((y - x - rx) / d);
        br.setPower((y + x - rx) / d);

        Tuning.drawDebug(follower);

        telemetry.addData("x",       follower.getPose().getX());
        telemetry.addData("y",       follower.getPose().getY());
        telemetry.addData("heading", Math.toDegrees(follower.getPose().getHeading()));
        telemetry.update();
    }
}
