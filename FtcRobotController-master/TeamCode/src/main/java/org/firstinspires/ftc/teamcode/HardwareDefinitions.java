package org.firstinspires.ftc.teamcode;

import static com.qualcomm.robotcore.hardware.DcMotor.ZeroPowerBehavior.FLOAT;
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
    public DcMotor fLmotor = null;
    public DcMotor fRmotor = null;
    public DcMotor bLmotor = null;
    public DcMotor bRmotor = null;
    public IMU imu = null;
    public DistanceSensor frontDistance;
    public DcMotorEx parallel;
    public DcMotorEx perpendicular;
    public ColorSensor colorSensor;
    public TouchSensor limitSwitch;

    public void init(HardwareMap hardwareMap, Telemetry telemetry) {
        // Motor configuration
        fLmotor = hardwareMap.get(DcMotor.class, "frontLeftMotor");
        fRmotor = hardwareMap.get(DcMotor.class, "frontRightMotor");
        bLmotor = hardwareMap.get(DcMotor.class, "backLeftMotor");
        bRmotor = hardwareMap.get(DcMotor.class, "backRightMotor");

        fRmotor.setDirection(DcMotorSimple.Direction.REVERSE);
        bRmotor.setDirection(DcMotorSimple.Direction.REVERSE);

        fRmotor.setZeroPowerBehavior(FLOAT);
        fLmotor.setZeroPowerBehavior(FLOAT);
        bLmotor.setZeroPowerBehavior(FLOAT);
        bRmotor.setZeroPowerBehavior(FLOAT);

        fLmotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        fRmotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        bLmotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        bRmotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        telemetry.addLine("Motors Initialized");
        telemetry.update();

        // IMU configuration
        imu = hardwareMap.get(IMU.class, "imu");
        telemetry.addLine("IMU Initialized");
        telemetry.update();

        // Odometry configuration
        parallel = hardwareMap.get(DcMotorEx.class, "frontLeftMotor");
        perpendicular = hardwareMap.get(DcMotorEx.class, "frontRightMotor");

        parallel.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        perpendicular.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        parallel.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        perpendicular.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        telemetry.addLine("Odometry Initialized");
        telemetry.update();

        frontDistance = hardwareMap.get(DistanceSensor.class, "frDist");
        colorSensor = hardwareMap.get(ColorSensor.class, "color_sensor");
        limitSwitch = hardwareMap.get(TouchSensor.class, "limit_switch");
        telemetry.addLine("Sensors Initialized");
        telemetry.update();
    }
}
