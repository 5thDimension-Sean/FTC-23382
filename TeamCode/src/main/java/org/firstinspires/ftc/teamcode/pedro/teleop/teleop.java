package org.firstinspires.ftc.teamcode.pedro.teleop;

import com.pedropathing.follower.Follower;
import com.pedropathing.math.Pose;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.pedro.Constants;
import org.firstinspires.ftc.teamcode.pedro.subsystem.intake;
import org.firstinspires.ftc.teamcode.pedro.subsystem.loader;
import org.firstinspires.ftc.teamcode.pedro.subsystem.turret;

@TeleOp(name = "23382 Teleop")
public class teleop extends OpMode {

    private static final int TELEOP_LIMELIGHT_POLL_HZ = 200;
    private static final double DRIVE_INPUT_THRESHOLD = 0.10;

    private Follower follower;
    private final LimelightFrames visionFrames = new LimelightFrames();
    private final HiveOdometryAim hiveOdometry = new HiveOdometryAim();
    private boolean previousOdometryButton;
    private HiveTurretAim hiveAim;
    private boolean blueAlliance = true;
    private boolean previousAllianceButton;
    private boolean handsFreeAim;
    private boolean previousAimButton;
    private boolean recenteredAtStart;
    intake intake = new intake();
    turret turret = new turret();
    loader loader = new loader();
    private int pressed = 0;

    @Override
    public void init() {
        follower = Constants.create(hardwareMap);
        intake.init(hardwareMap);
        turret.init(hardwareMap, TELEOP_LIMELIGHT_POLL_HZ);
        hiveAim = new HiveTurretAim(turret.turretServo, hardwareMap.appContext);
        loader.init(hardwareMap);
    }

    @Override
    public void start() {
        previousAimButton = gamepad1.right_stick_button;
        previousOdometryButton = gamepad1.left_stick_button;
        previousAllianceButton = gamepad1.dpad_down;
        recenteredAtStart = hiveAim.restoreCenterIfNeeded();
    }

    @Override
    public void loop() {
        follower.manual(
                -gamepad1.left_stick_y,
                -gamepad1.left_stick_x,
                -gamepad1.right_stick_x
        );

        if(gamepad1.right_trigger != 0){
            intake.setPowerGradual(Math.min(gamepad1.right_trigger+0.5, 1));
            turret.setFlywheelVelocity(2500);
        } else if(gamepad1.right_bumper){
            intake.setPowerGradual(-1);
        } else{
            intake.setPowerGradual(0);
            turret.setFlywheelVelocity(0);
        }
        if(gamepad1.b){
            turret.moveTurretManual(1);
        } else if(gamepad1.x){
            turret.moveTurretManual(0);
        }
        if(gamepad1.y){
            turret.moveLink("open");
        } else if(gamepad1.a){
            turret.moveLink("close");
        }
        if(gamepad1.left_trigger_pressed){
            turret.moveBlocker("open");
        } else if(gamepad1.left_bumper){
            turret.moveBlocker("close");
        }

        if(gamepad2.b){
            turret.moveTurretManual(1);
        } else if(gamepad2.x){
            turret.moveTurretManual(0);
        }
        if(gamepad2.y){
            turret.moveLinkManual(1);
        } else if(gamepad2.a){
            turret.moveLinkManual(0);
        }
        if(gamepad2.left_trigger_pressed){
            turret.moveBlockerManual(1);
        } else if(gamepad2.left_bumper){
            turret.moveBlockerManual(0);
        }
        updateHiveAim();
        telemetry.addData("Target Position(block)", turret.blocker.getPosition());
        telemetry.addData("Target Position(linkage)", turret.linkage.getPosition());
        telemetry.addData("Target Position(turret)", turret.turretServo.getPosition());
        telemetry.addData("Target Position(loaderLeft)", loader.loaderLeft.getPosition());
        telemetry.addData("Target Position(loaderRight)", loader.loaderRight.getPosition());
        telemetry.update();
        /*
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
        */
        follower.update();

    }

    private void updateHiveAim() {
        boolean allianceButton = gamepad1.dpad_down;
        if (allianceButton && !previousAllianceButton) {
            blueAlliance = !blueAlliance;
            hiveOdometry.clearLastSeen();
        }
        previousAllianceButton = allianceButton;
        telemetry.addData("Hive alliance (D-pad down)", blueAlliance ? "Blue" : "Red");
        telemetry.addData("Hive calibration", hiveAim.isCalibrated() ? "Saved" : "Required");
        if (recenteredAtStart) {
            telemetry.addData("Turret startup", "Restored measured center %.4f",
                    HiveTurretAim.STRAIGHT_AHEAD_POSITION);
        }
        LLResult frame = visionFrames.freshResult(turret.getLimelight());
        HiveOpeningVision.Opening opening = HiveOpeningVision.find(frame, blueAlliance, telemetry, false);
        boolean manual = gamepad1.b || gamepad1.x || gamepad2.b || gamepad2.x;
        boolean aimButton = gamepad1.right_stick_button;
        if (manual || allianceButton) {
            handsFreeAim = false;
        } else if (aimButton && !previousAimButton) {
            handsFreeAim = !handsFreeAim;
        }
        previousAimButton = aimButton;
        telemetry.addData("Hands-free aim (R3)", handsFreeAim ? "On" : "Off");
        boolean odometryButton = gamepad1.left_stick_button;
        boolean capture = odometryButton && !previousOdometryButton;
        previousOdometryButton = odometryButton;
        boolean driving = driveInputActive();
        boolean track = (handsFreeAim || gamepad2.right_bumper) && !allianceButton && !capture;
        Pose pose = follower.pose();
        if (capture && !handsFreeAim && !gamepad2.right_bumper && !manual && !driving) {
            hiveOdometry.captureNext(pose, opening, turret.turretServo.getPosition());
        }
        if (!allianceButton) {
            hiveOdometry.observe(opening);
        }
        hiveOdometry.mapAutomatically(pose, opening, turret.turretServo.getPosition(),
                frame == null ? Double.NaN : frame.getTimestamp(),
                driving || manual || capture || allianceButton, hiveAim);
        String status = hiveAim.update(frame, opening, track, manual, driving, telemetry);
        if (track && !manual && opening == null && hiveAim.isCalibrated()) {
            status = hiveOdometry.reacquire(pose, hiveAim);
        }
        telemetry.addData("Hive aim", status);
        telemetry.addData("Hive odometry", hiveOdometry.status());
        if (track && opening == null) {
            hiveOdometry.showPrediction(pose, telemetry);
        }
    }

    private boolean driveInputActive() {
        return Math.abs(gamepad1.left_stick_x) > DRIVE_INPUT_THRESHOLD
                || Math.abs(gamepad1.left_stick_y) > DRIVE_INPUT_THRESHOLD
                || Math.abs(gamepad1.right_stick_x) > DRIVE_INPUT_THRESHOLD;
    }

    @Override
    public void stop() {
        turret.getLimelight().stop();
    }
}
