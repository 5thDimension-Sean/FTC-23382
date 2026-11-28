package org.firstinspires.ftc.teamcode.pedro.subsystem;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.pedropathing.math.Pose;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PwmControl;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoImplEx;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.limelightvision.LLResultTypes.FiducialResult;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;


import org.firstinspires.ftc.teamcode.pedro.Constants;

public class turret {
    private DcMotorEx flywheel;
    public ServoImplEx linkage;
    public ServoImplEx turretServo;
    public ServoImplEx blocker;
    Limelight3A limelight;
    private final LimelightFrames visionFrames = new LimelightFrames();
    private double lastHiveFrame = Double.NaN;
    private boolean lastBlue;
    private String hiveStatus;
    private int uprightCell = -1; // 0 = audience side, 1 = scoring-table side, -1 = unknown


    public void init(HardwareMap hardwareMap){
        limelight = hardwareMap.get(Limelight3A.class, Constants.LIMELIGHT_NAME);
        limelight.setPollRateHz(100);
        limelight.start();
        limelight.pipelineSwitch(LimelightFrames.PIPELINE);

        linkage = hardwareMap.get(ServoImplEx.class, Constants.TURRET_LINKAGE_SERVO_NAME);
        linkage.setPwmRange(new PwmControl.PwmRange(500, 2500));

        turretServo = hardwareMap.get(ServoImplEx.class, Constants.TURRET_ROTATION_SERVO_NAME);
        turretServo.setPwmRange(new PwmControl.PwmRange(500, 2500));
        turretServo.setDirection(Servo.Direction.REVERSE);

        blocker = hardwareMap.get(ServoImplEx.class, Constants.TURRET_BLOCKER_SERVO_NAME);
        blocker.setPwmRange(new PwmControl.PwmRange(500, 2500));


        flywheel = hardwareMap.get(DcMotorEx.class, Constants.TURRET_FLYWHEEL_MOTOR_NAME);
        flywheel.setPower(0);
        flywheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        flywheel.setVelocityPIDFCoefficients(
                Constants.TURRET_FLYWHEEL_KP,
                Constants.TURRET_FLYWHEEL_KI,
                Constants.TURRET_FLYWHEEL_KD,
                Constants.TURRET_FLYWHEEL_KF
        );
    }

    /** Reports the selected alliance's upright Hive cell alongside the current Pinpoint pose. */
    public void trackHive(Pose pose, Telemetry telemetry) {
        if (pose != null) {
            telemetry.addData("Pinpoint (in, deg)", "%.1f, %.1f, %.1f",
                    pose.x(), pose.y(), Math.toDegrees(pose.heading()));
        }
        boolean blue = Constants.colorLeft.equals(Constants.selectedTeam);
        if (!blue && !Constants.colorRight.equals(Constants.selectedTeam)) {
            telemetry.addData("Hive", "Select alliance during INIT");
            return;
        }
        LLResult result = visionFrames.freshResult(limelight);
        if (result == null) {
            lastHiveFrame = Double.NaN;
            telemetry.addData("Hive", "Waiting for fresh AprilTags on pipeline 0");
            return;
        }
        if (result.getTimestamp() != lastHiveFrame || blue != lastBlue) {
            hiveStatus = identifyUprightCell(result, blue);
            uprightCell = hiveStatus.startsWith("Audience") ? 0 : hiveStatus.startsWith("Scoring") ? 1 : -1;
            lastHiveFrame = result.getTimestamp();
            lastBlue = blue;
        }
        telemetry.addData("Hive (" + Constants.selectedTeam + ")", hiveStatus);
    }

    private static String identifyUprightCell(LLResult result, boolean blue) {
        // Each alliance has two clusters of four tags. Blue's first cluster faces the audience.
        int firstId = blue ? 38 : 30;
        boolean[] upright = new boolean[2];
        boolean[] inverted = new boolean[2];
        for (FiducialResult tag : result.getFiducialResults()) {
            int member = tag.getFiducialId() - firstId;
            if (member < 0 || member >= 8) {
                continue;
            }
            Pose3D tagPose = tag.getTargetPoseCameraSpace();
            if (tagPose == null) {
                continue;
            }
            // Same viewing-axis roll convention and ambiguity band as HiveOpeningVision.
            YawPitchRollAngles angles = tagPose.getOrientation();
            double x = angles.getRoll(AngleUnit.RADIANS);
            double y = angles.getPitch(AngleUnit.RADIANS);
            double z = angles.getYaw(AngleUnit.RADIANS);
            double roll = Math.abs(Math.toDegrees(Math.atan2(Math.sin(z) * Math.cos(y),
                    Math.sin(z) * Math.sin(y) * Math.sin(x) + Math.cos(z) * Math.cos(x))));
            int cell = member / 4;
            if (roll < 80) {
                upright[cell] = true;
            } else if (roll > 100) {
                inverted[cell] = true;
            }
        }
        int audience = blue ? 0 : 1;
        int scoringTable = 1 - audience;
        boolean audienceUp = upright[audience] && !inverted[audience];
        boolean scoringUp = upright[scoringTable] && !inverted[scoringTable];
        if (audienceUp == scoringUp) {
            return audienceUp ? "Both cells appear upright" : "No clear upright cell";
        }
        return audienceUp ? "Audience cell upright" : "Scoring-table cell upright";
    }

    /**
     * Points the turret at the HIVE from odometry: bearing = atan2(dy, dx) in the start-relative frame, minus the robot heading, converted to a servo position.
     */
    public void aimAtGoal(Pose pose, Telemetry telemetry) {
        double goalX = Constants.GOAL_FORWARD_IN;
        double goalY = Constants.GOAL_LEFT_IN;
        double fieldBearing = Math.atan2(goalY - pose.y(), goalX - pose.x());
        double relative = Math.toDegrees(AngleUnit.normalizeRadians(fieldBearing - pose.heading()));
        double target = Constants.TURRET_CENTER + Constants.TURRET_SIGN * relative / Constants.TURRET_DEG_PER_UNIT;
        double clipped = Math.max(Constants.TURRET_MIN, Math.min(Constants.TURRET_MAX, target));
        turretServo.setPosition(clipped);
        telemetry.addData("Turret aim", "goal (%.1f, %.1f) turret %.1f deg -> servo %.3f%s",
                goalX, goalY, relative, clipped, clipped != target ? " (LIMIT)" : "");
    }

    public void stopTracking() {
        limelight.stop();
    }

    /**
     * Sets the flywheel's target velocity in encoder ticks per second.
     *
     * @param ticksPerSecond Signed target velocity. Use 0 to stop the flywheel.
     */
    public void setFlywheelVelocity(double ticksPerSecond) {
        flywheel.setVelocity(ticksPerSecond);
    }

    /**
     * Moves the linkage servo to a predefined position.
     *
     * @param position "open" (0.61.0) or "close" (0.0), case-insensitive.
     */
    public void moveLink(String position){
        if(position.equalsIgnoreCase("open")){
            linkage.setPosition(0.61);
        } else if(position.equalsIgnoreCase("close")){
            linkage.setPosition(0.0);
        }
    }
    /**
     * Moves the blocker servo to a predefined position.
     *
     * @param position "open" (0.47) or "close" (0.17), case-insensitive.
     */
    public void moveBlocker(String position){
        if(position.equalsIgnoreCase("open")){
            blocker.setPosition(0.17);
        } else if(position.equalsIgnoreCase("close")){
            blocker.setPosition(0.47);
        }
    }
    /**
     * Incrementally adjusts the linkage servo position by 0.01 per call.
     *
     * @param direction 0 to decrease position, 1 to increase position.
     */
    public void moveLinkManual(int direction){
        if(direction == 0){
            linkage.setPosition(linkage.getPosition()-0.005);
        }else if(direction == 1){
            linkage.setPosition(linkage.getPosition()+0.005);
        }
    }
    /**
     * Incrementally adjusts the turret servo position by 0.01 per call.
     *
     * @param direction 0 to go left, 1 to go right.
     */
    public void moveTurretManual(int direction){
        if(direction == 0){
            turretServo.setPosition(turretServo.getPosition()-0.005);
        }else if(direction == 1){
            turretServo.setPosition(turretServo.getPosition()+0.005);
        }
    }
    /**
     * Incrementally adjusts the blocker servo position by 0.01 per call.
     *
     * @param direction 0 to go left, 1 to go right.
     */
    public void moveBlockerManual(int direction){
        if(direction == 0){
            blocker.setPosition(blocker.getPosition()-0.005);
        }else if(direction == 1){
            blocker.setPosition(blocker.getPosition()+0.005);
        }
    }
}
