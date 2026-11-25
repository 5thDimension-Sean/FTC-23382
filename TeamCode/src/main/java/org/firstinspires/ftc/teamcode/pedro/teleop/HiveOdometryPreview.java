package org.firstinspires.ftc.teamcode.pedro.teleop;

import com.pedropathing.follower.Follower;
import com.pedropathing.math.Pose;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.PwmControl;
import com.qualcomm.robotcore.hardware.ServoImplEx;

import org.firstinspires.ftc.teamcode.pedro.Constants;

@TeleOp(name = "Hive Odometry Preview", group = "Calibration")
public class HiveOdometryPreview extends LimelightHiveOpeningPreview {
    private static final double DRIVE_SCALE = 0.30;

    private final HiveOdometryAim odometry = new HiveOdometryAim();
    private Follower follower;
    private ServoImplEx servo;
    private HiveTurretAim aim;
    private boolean previousA;
    private boolean previousY;

    @Override
    public void init() {
        super.init();
        follower = Constants.create(hardwareMap);
        follower.setPose(Pose.zero());
        servo = hardwareMap.get(ServoImplEx.class, Constants.TURRET_ROTATION_SERVO_NAME);
        servo.setPwmRange(new PwmControl.PwmRange(500, 2500));
        aim = new HiveTurretAim(servo, hardwareMap.appContext);
        hardwareMap.get(DcMotorEx.class, Constants.TURRET_FLYWHEEL_MOTOR_NAME).setPower(0);
    }

    @Override
    public void start() {
        super.start();
        previousA = gamepad1.a;
        previousY = gamepad1.y;
    }

    @Override
    protected void handleOpening(LLResult frame, HiveOpeningVision.Opening opening) {
        follower.manual(-DRIVE_SCALE * gamepad1.left_stick_y,
                -DRIVE_SCALE * gamepad1.left_stick_x, -DRIVE_SCALE * gamepad1.right_stick_x);
        follower.update();
        Pose pose = follower.pose();
        boolean captureA = gamepad1.a && !previousA;
        boolean captureY = gamepad1.y && !previousY;
        double command = servo.getPosition();
        if (!gamepad1.left_bumper) {
            if (captureA) {
                odometry.captureFirst(pose, opening, command);
            } else if (captureY) {
                odometry.captureSecond(pose, opening, command);
            }
        }
        previousA = gamepad1.a;
        previousY = gamepad1.y;

        odometry.observe(opening);
        boolean enabled = gamepad1.left_bumper && !captureA && !captureY;
        String result = aim.update(frame, opening, enabled && odometry.isMapped(opening),
                false, false, telemetry);
        if (enabled && opening == null && aim.isCalibrated()) {
            result = odometry.reacquire(pose, aim);
        }
        telemetry.addData("Odometry setup", odometry.status());
        telemetry.addData("Pinpoint position (in)", "X %.1f / Y %.1f", pose.x(), pose.y());
        telemetry.addData("Pinpoint heading (deg)", "%.1f", Math.toDegrees(pose.heading()));
        odometry.showPrediction(pose, telemetry);
        telemetry.addData("Servo command", "%.4f", command);
        telemetry.addData("Reacquisition test", result);
        telemetry.addLine("Turret straight: A then Y record the same aligned cell from two spots.");
        telemetry.addLine("After both points are saved, hold LB to test. Release LB to hold.");
    }
}
