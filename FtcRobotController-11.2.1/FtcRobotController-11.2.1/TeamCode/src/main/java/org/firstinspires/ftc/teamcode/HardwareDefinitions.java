package org.firstinspires.ftc.teamcode;

import static com.qualcomm.robotcore.hardware.DcMotor.ZeroPowerBehavior.FLOAT;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import org.firstinspires.ftc.robotcore.external.Telemetry;

public class HardwareDefinitions {
    public DcMotor fl = null;
    public DcMotor fr = null;
    public DcMotor bl = null;
    public DcMotor br = null;
    public IMU imu = null;

    public void init(HardwareMap hardwareMap, Telemetry telemetry) {
        try {
            // Motor configuration
            fl = hardwareMap.get(DcMotor.class, "frontleft");
            fr = hardwareMap.get(DcMotor.class, "frontright");
            bl = hardwareMap.get(DcMotor.class, "backleft");
            br = hardwareMap.get(DcMotor.class, "backright");

            fl.setDirection(DcMotorSimple.Direction.REVERSE);
            bl.setDirection(DcMotorSimple.Direction.REVERSE);

            fl.setZeroPowerBehavior(FLOAT);
            fr.setZeroPowerBehavior(FLOAT);
            bl.setZeroPowerBehavior(FLOAT);
            br.setZeroPowerBehavior(FLOAT);

            fl.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            fr.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            bl.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            br.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            // IMU configuration
            imu = hardwareMap.get(IMU.class, "imu");

            telemetry.addLine("Hardware Init Sequence Complete");
        } catch (Exception e) {
            telemetry.addLine("Init Warning: Some hardware not found");
        }
        telemetry.update();
    }
}
