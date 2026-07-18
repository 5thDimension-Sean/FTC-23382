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
        fl = hardwareMap.get(DcMotor.class, "fl");
        fr = hardwareMap.get(DcMotor.class, "fr");
        bl = hardwareMap.get(DcMotor.class, "bl");
        br = hardwareMap.get(DcMotor.class, "br");

        fr.setDirection(DcMotorSimple.Direction.REVERSE);
        br.setDirection(DcMotorSimple.Direction.REVERSE);

        fr.setZeroPowerBehavior(FLOAT);
        fl.setZeroPowerBehavior(FLOAT);
        bl.setZeroPowerBehavior(FLOAT);
        br.setZeroPowerBehavior(FLOAT);

        fl.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        fr.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        bl.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        br.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        telemetry.addLine("Motors Initialized");
        telemetry.update();

        // IMU configuration
        imu = hardwareMap.get(IMU.class, "imu");
        telemetry.addLine("IMU Initialized");
        telemetry.update();

        // Odometry configuration
        parallel = hardwareMap.get(DcMotorEx.class, "fl");
        perpendicular = hardwareMap.get(DcMotorEx.class, "fr");

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
