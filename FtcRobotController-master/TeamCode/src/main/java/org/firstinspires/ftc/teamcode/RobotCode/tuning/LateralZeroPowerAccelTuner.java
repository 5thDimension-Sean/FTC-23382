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
 * Runs the robot sideways (right) until VELOCITY (in/s) is reached, then cuts power.
 * Measures average deceleration and displays it.
 * Put the result into FConstants.followerConstants.lateralZeroPowerAcceleration (will be negative).
 * Press A/cross to stop early.
 */
@Config
@Autonomous(name = "Lateral Zero Power Accel Tuner", group = "Pedro Tuning")
public class LateralZeroPowerAccelTuner extends OpMode {
    public static double VELOCITY = 30;

    private Follower follower;
    private DcMotorEx fl, fr, bl, br;
    private List<DcMotorEx> motors;
    private ArrayList<Double> accelerations = new ArrayList<>();

    private double prevVelocity;
    private long prevTimeNano;
    private double prevY;
    private long prevYTimeNano;

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

        prevY = 0;
        prevYTimeNano = System.nanoTime();

        telemetry.addLine("Robot will strafe right until reaching " + VELOCITY + " in/s, then cut power.");
        telemetry.addLine("Make sure you have room. Press A to stop.");
        telemetry.update();
    }

    @Override
    public void start() {
        // Strafe right: fl=+1, fr=-1, bl=-1, br=+1 (with reversed left motors already applied)
        fl.setPower(-1);
        fr.setPower(-1);
        bl.setPower(1);
        br.setPower(1);
    }

    @Override
    public void loop() {
        if (gamepad1.a || gamepad1.cross) {
            for (DcMotorEx m : motors) { m.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE); m.setPower(0); }
            requestOpModeStop();
            return;
        }

        follower.update();
        double y = follower.getPose().getY();
        long now = System.nanoTime();
        double dt = (now - prevYTimeNano) / 1e9;
        double velocity = (dt > 0) ? Math.abs((y - prevY) / dt) : 0;
        prevY = y;
        prevYTimeNano = now;

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
            telemetry.addData("lateralZeroPowerAcceleration", avg);
            telemetry.update();
        }
    }
}
