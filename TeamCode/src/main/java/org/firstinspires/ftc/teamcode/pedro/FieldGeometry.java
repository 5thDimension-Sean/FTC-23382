package org.firstinspires.ftc.teamcode.pedro;

import com.pedropathing.math.Pose;

/**
 * Pure geometry helpers (no hardware), kept separate so they are easy to unit test.
 * Angles are radians, distances are inches. Robot frame: +x forward, +y left.
 */
public final class FieldGeometry {
    private FieldGeometry() {}

    /** Wraps an angle to (-PI, PI]. */
    public static double wrap(double angle) {
        while (angle > Math.PI) angle -= 2 * Math.PI;
        while (angle <= -Math.PI) angle += 2 * Math.PI;
        return angle;
    }

    /**
     * Corners of the robot rectangle in field coordinates, in drawing order
     * (front-left, front-right, back-right, back-left).
     *
     * @return {{x, y} x4}
     */
    public static double[][] robotCorners(Pose pose, double length, double width) {
        double f = length / 2.0;
        double l = width / 2.0;
        double[][] local = {{f, l}, {f, -l}, {-f, -l}, {-f, l}};
        double[][] out = new double[4][2];
        for (int i = 0; i < 4; i++) {
            out[i] = robotToField(pose, local[i][0], local[i][1]);
        }
        return out;
    }

    /** Converts a point given in the robot frame (forward, left) to field coordinates. */
    public static double[] robotToField(Pose pose, double forward, double left) {
        double c = Math.cos(pose.heading());
        double s = Math.sin(pose.heading());
        return new double[]{
                pose.x() + forward * c - left * s,
                pose.y() + forward * s + left * c
        };
    }

    /**
     * FTC field frame (origin = field centre, what the Limelight reports) -> Pedro frame
     * (origin = bottom-left corner of the 144 in field).
     *
     * @param rotation how far the FTC frame is rotated relative to the Pedro frame, radians
     */
    public static Pose ftcToPedro(double xIn, double yIn, double headingRad, double rotation) {
        double c = Math.cos(rotation);
        double s = Math.sin(rotation);
        double half = Constants.fieldSize / 2.0;
        return new Pose(
                xIn * c - yIn * s + half,
                xIn * s + yIn * c + half,
                wrap(headingRad + rotation));
    }

    /** Pedro heading -> FTC-frame heading (what the Limelight wants as the robot yaw). */
    public static double pedroHeadingToFtc(double pedroHeading, double rotation) {
        return wrap(pedroHeading - rotation);
    }
}
