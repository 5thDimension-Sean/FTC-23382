package org.firstinspires.ftc.teamcode.RobotCode.tuning;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.configuration.typecontainers.MotorConfigurationType;

import com.pedropathing.follower.Follower;

import org.firstinspires.ftc.teamcode.RobotCode.constants.FConstants;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

/**
 * Runs the robot forward until VELOCITY (in/s) is reached, then cuts power.
 * Measures average deceleration and displays it.
 * Put the result into FConstants.followerConstants.forwardZeroPowerAcceleration (it will be negative).
 * Press A/cross to stop early.
 */
@Config
@Autonomous(name = "Forward Zero Power Accel Tuner", group = "Pedro Tuning")
public class ForwardZeroPowerAccelTuner extends OpMode {
    public static double VELOCITY = 30;

    private Follower follower;
    private DcMotorEx fl, fr, bl, br;
    private List<DcMotorEx> motors;
    private ArrayList<Double> accelerations = new ArrayList<>();

    private double prevVelocity;
    private long prevTimeNano;
    private double prevX;
    private long prevXTimeNano;

    private boolean stopping;
    private boolean done;

    @Override
    public void init() {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        follower = FConstants.createFollower(hardwareMap);

        fl = hardwareMap.get(DcMotorEx.class, "frontleft");
        fr = hardwareMap.get(DcMotorEx.class, "frontright");
        bl = hardwareMap.get(DcMotorEx.class, "backleft");
        br = hardwareMap.get(DcMotorEx.class, "backright");
        fl.setDirection(DcMotorSimple.Direction.REVERSE);
        bl.setDirection(DcMotorSimple.Direction.REVERSE);
        motors = Arrays.asList(fl, fr, bl, br);

        for (DcMotorEx m : motors) {
            MotorConfigurationType t = m.getMotorType().clone();
            t.setAchieveableMaxRPMFraction(1.0);
            m.setMotorType(t);
            m.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        }

        prevX = 0;
        prevXTimeNano = System.nanoTime();

        telemetry.addLine("Robot will drive forward until reaching " + VELOCITY + " in/s, then cut power.");
        telemetry.addLine("Make sure you have room. Press A to stop.");
        telemetry.update();
    }

    @Override
    public void start() {
        for (DcMotorEx m : motors) m.setPower(1);
    }

    @Override
    public void loop() {
        if (gamepad1.a || gamepad1.cross) {
            for (DcMotorEx m : motors) { m.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE); m.setPower(0); }
            requestOpModeStop();
            return;
        }

        follower.update();
        double x = follower.getPose().getX();
        long now = System.nanoTime();
        double dt = (now - prevXTimeNano) / 1e9;
        double velocity = (dt > 0) ? (x - prevX) / dt : 0;
        prevX = x;
        prevXTimeNano = now;

        if (!done) {
            if (!stopping) {
                if (velocity > VELOCITY) {
                    prevVelocity = velocity;
                    prevTimeNano = now;
                    stopping = true;
                    for (DcMotorEx m : motors) m.setPower(0);
                }
            } else {
                double dtStop = (now - prevTimeNano) / 1e9;
                if (dtStop > 0) {
                    accelerations.add((velocity - prevVelocity) / dtStop);
                }
                prevVelocity = velocity;
                prevTimeNano = now;
                if (velocity < 0.1) done = true;
            }
        } else {
            double avg = 0;
            for (double a : accelerations) avg += a;
            avg /= accelerations.size();
            telemetry.addData("forwardZeroPowerAcceleration", avg);
            telemetry.update();
        }
    }
}
