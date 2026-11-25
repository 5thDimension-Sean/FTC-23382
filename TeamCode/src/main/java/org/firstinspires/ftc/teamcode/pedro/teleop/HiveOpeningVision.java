package org.firstinspires.ftc.teamcode.pedro.teleop;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes.FiducialResult;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;

final class HiveOpeningVision {
    private static final String[] CELL_NAMES = {"Red scoring table", "Red audience",
            "Blue audience", "Blue scoring table"};
    // FTC SDK 12 BIOBUZZ cluster coordinates, in inches, in printed-tag axes.
    private static final double[] MEMBER_X = {-6.50, -2.75, 2.75, 6.50};
    private static final double MEMBER_Y = 7.1874;
    private static final double MEMBER_Z = -5.622;

    static Opening find(LLResult result, boolean blue, Telemetry telemetry, boolean showTags) {
        if (result == null) {
            telemetry.addData("Selected cell", "Waiting for fresh Pipeline 0 tags");
            return null;
        }
        telemetry.addData("Result age (ms)", result.getStaleness());
        return showOpenings(result, blue, telemetry, showTags);
    }

    static final class Opening {
        final double x;
        final double z;

        Opening(double x, double z) {
            this.x = x;
            this.z = z;
        }
    }

    private static Opening showOpenings(LLResult result, boolean blue, Telemetry telemetry,
            boolean showTags) {
        CellEstimate[] cells = {new CellEstimate(), new CellEstimate(),
                new CellEstimate(), new CellEstimate()};
        for (FiducialResult tag : result.getFiducialResults()) {
            int member = tag.getFiducialId() - 30;
            if (member < 0 || member >= 16) {
                if (showTags) {
                    telemetry.addData("Tag " + tag.getFiducialId(), "Outside Hive clusters");
                }
                continue;
            }
            showOpening(tag, member % 4, cells[member / 4], showTags ? telemetry : null);
        }
        int first = blue ? 2 : 0;
        boolean firstUp = cells[first].isUp();
        boolean secondUp = cells[first + 1].isUp();
        if (firstUp == secondUp) {
            telemetry.addData("Selected cell", firstUp ? "Both appear upright" : "No clear upright cell");
            return null;
        }

        int selected = firstUp ? first : first + 1;
        CellEstimate cell = cells[selected];
        telemetry.addData("Selected cell", CELL_NAMES[selected]);
        telemetry.addData("Opening bearing (deg, right positive)", "%+.2f",
                Math.toDegrees(Math.atan2(cell.sumX, cell.sumZ)));
        telemetry.addData("Tags used / bearing spread", "%d / %.2f deg",
                cell.upright, cell.maxBearing - cell.minBearing);
        return new Opening(cell.sumX / cell.upright, cell.sumZ / cell.upright);
    }

    private static void showOpening(FiducialResult tag, int member, CellEstimate cell,
            Telemetry telemetry) {
        String label = "Tag " + tag.getFiducialId();
        Pose3D pose = tag.getTargetPoseCameraSpace();
        if (pose == null) {
            if (telemetry != null) {
                telemetry.addData(label, "No 3D pose");
            }
            return;
        }
        Position position = pose.getPosition().toUnit(DistanceUnit.INCH);
        YawPitchRollAngles angles = pose.getOrientation();
        // The Limelight SDK stores the X, Y, Z rotations in roll, pitch, yaw.
        double rx = angles.getRoll(AngleUnit.RADIANS);
        double ry = angles.getPitch(AngleUnit.RADIANS);
        double rz = angles.getYaw(AngleUnit.RADIANS);
        double cx = Math.cos(rx), sx = Math.sin(rx);
        double cy = Math.cos(ry), sy = Math.sin(ry);
        double cz = Math.cos(rz), sz = Math.sin(rz);

        // Translate from tag to opening in tag axes, then rotate X, Y, Z into camera axes.
        double x = -MEMBER_X[member];
        double y = -MEMBER_Y * cx + MEMBER_Z * sx;
        double z = -MEMBER_Y * sx - MEMBER_Z * cx;
        double rotatedX = cy * x + sy * z;
        double openingX = position.x + cz * rotatedX - sz * y;
        double openingZ = position.z - sy * x + cy * z;
        if (!(position.z > 0 && openingZ > 0) || !Double.isFinite(openingX)
                || !Double.isFinite(openingZ)) {
            if (telemetry != null) {
                telemetry.addData(label, "Invalid 3D pose");
            }
            return;
        }

        // Extract rotation about the viewing axis using the FTC intrinsic Y-X-Z convention.
        double roll = Math.toDegrees(Math.atan2(sz * cy, sz * sy * sx + cz * cx));
        double bearing = Math.toDegrees(Math.atan2(openingX, openingZ));
        if (telemetry != null) {
            telemetry.addData(label, "tx %+.1f, opening %+.2f, roll %+.1f",
                    tag.getTargetXDegrees(), bearing, roll);
        }
        if (Math.abs(roll) < 80) {
            cell.add(openingX, openingZ, bearing);
        } else if (Math.abs(roll) > 100) {
            cell.inverted = true;
        }
    }

    private static final class CellEstimate {
        private int upright;
        private boolean inverted;
        private double sumX;
        private double sumZ;
        private double minBearing = Double.POSITIVE_INFINITY;
        private double maxBearing = Double.NEGATIVE_INFINITY;

        void add(double x, double z, double bearing) {
            upright++;
            sumX += x;
            sumZ += z;
            minBearing = Math.min(minBearing, bearing);
            maxBearing = Math.max(maxBearing, bearing);
        }

        boolean isUp() {
            return upright > 0 && !inverted;
        }
    }
}
