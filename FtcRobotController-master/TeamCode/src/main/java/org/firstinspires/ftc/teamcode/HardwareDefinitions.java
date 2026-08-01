package org.firstinspires.ftc.teamcode;

import static com.qualcomm.robotcore.hardware.DcMotor.ZeroPowerBehavior.FLOAT;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.TouchSensor;
public class HardwareDefinitions {
    public DcMotor fl = null;
    public DcMotor fr = null;
    public DcMotor bl = null;
    public DcMotor br = null;
    public IMU imu = null;
    public DistanceSensor frontDistance;
    public DcMotorEx parallel;
    public DcMotorEx perpendicular;
    public ColorSensor colorSensor;
    public TouchSensor limitSwitch;

    public void init(HardwareMap hardwareMap, Telemetry telemetry) {
        // Motor configuration
        fl = hardwareMap.get(DcMotor.class, "frontleft");
        fr = hardwareMap.get(DcMotor.class, "frontright");
        bl = hardwareMap.get(DcMotor.class, "backleft");
        br = hardwareMap.get(DcMotor.class, "backright");

        fl.setDirection(DcMotorSimple.Direction.REVERSE);
        bl.setDirection(DcMotorSimple.Direction.REVERSE);

        fr.setZeroPowerBehavior(FLOAT);
        fl.setZeroPowerBehavior(FLOAT);
        bl.setZeroPowerBehavior(FLOAT);
        br.setZeroPowerBehavior(FLOAT);

        fl.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        fr.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        bl.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        br.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        fl.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        fr.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        bl.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        br.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        telemetry.addLine("Motors Initialized");
        telemetry.update();

        // IMU configuration

        imu = hardwareMap.get(IMU.class, "imu");
        IMU.Parameters parameters = new IMU.Parameters(
                new RevHubOrientationOnRobot(
                        RevHubOrientationOnRobot.LogoFacingDirection.UP,
                        RevHubOrientationOnRobot.UsbFacingDirection.FORWARD
                )
        );
        telemetry.addLine("IMU Initialized");
        telemetry.update();

        // Odometry configuration
        //parallel = hardwareMap.get(DcMotorEx.class, "fl");
        //perpendicular = hardwareMap.get(DcMotorEx.class, "fr");

        //parallel.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        //perpendicular.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        //parallel.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        //perpendicular.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        telemetry.addLine("Odometry Initialized");
        telemetry.update();

        //frontDistance = hardwareMap.get(DistanceSensor.class, "frDist");
        //colorSensor = hardwareMap.get(ColorSensor.class, "color_sensor");
        //limitSwitch = hardwareMap.get(TouchSensor.class, "limit_switch");
        telemetry.addLine("Sensors Initialized");
        telemetry.update();
    }
}
