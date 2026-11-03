package org.firstinspires.ftc.teamcode.pedro;

import com.bylazar.field.FieldManager;
import com.bylazar.field.PanelsField;
import com.bylazar.field.Style;
import com.pedropathing.math.Pose;

/**
 * Draws the robot on the Panels field view. The robot is a rectangle of
 * Constants.robotLength x Constants.robotWidth, rotated by the odometry heading, with a line out
 * of the front so you can see which way it faces.
 *
 * This is the only class that touches the Panels API. If Panels is missing or changes, drawing
 * switches itself off instead of crashing the OpMode.
 *
 * Call drawRobot(...) (and optionally drawTarget/drawAimLine) then update() once per loop.
 */
public final class FieldDrawer {
    private static final Style ROBOT = new Style("", "#3F51B5", 0.75);
    private static final Style TARGET = new Style("", "#F44336", 0.75);
    private static final Style AIM_OK = new Style("", "#4CAF50", 0.75);
    private static final Style AIM_NO = new Style("", "#9E9E9E", 0.75);

    private static FieldManager field;
    private static boolean broken = false;

    private FieldDrawer() {}

    private static FieldManager field() {
        if (field == null) {
            field = PanelsField.INSTANCE.getField();
            field.setOffsets(PanelsField.INSTANCE.getPresets().getPEDRO_PATHING());
        }
        return field;
    }

    /** Robot rectangle + heading line at the given pose. */
    public static void drawRobot(Pose pose) {
        if (broken) return;
        try {
            FieldManager f = field();
            double[][] c = FieldGeometry.robotCorners(pose, Constants.robotLength, Constants.robotWidth);
            f.setStyle(ROBOT);
            for (int i = 0; i < 4; i++) {
                double[] from = c[i];
                double[] to = c[(i + 1) % 4];
                f.moveCursor(from[0], from[1]);
                f.line(to[0], to[1]);
            }
            double[] nose = FieldGeometry.robotToField(pose, Constants.robotLength / 2.0, 0);
            f.moveCursor(pose.x(), pose.y());
            f.line(nose[0], nose[1]);
        } catch (Throwable t) {
            broken = true;
        }
    }

    /** Marks the hive being aimed at. */
    public static void drawTarget(double x, double y) {
        if (broken) return;
        try {
            FieldManager f = field();
            f.setStyle(TARGET);
            f.moveCursor(x, y);
            f.circle(3);
        } catch (Throwable t) {
            broken = true;
        }
    }

    /** Line from the robot to the hive: green when the shot is OK, grey when it is refused. */
    public static void drawAimLine(Pose robot, double targetX, double targetY, boolean canShoot) {
        if (broken) return;
        try {
            FieldManager f = field();
            f.setStyle(canShoot ? AIM_OK : AIM_NO);
            f.moveCursor(robot.x(), robot.y());
            f.line(targetX, targetY);
        } catch (Throwable t) {
            broken = true;
        }
    }

    /** Sends everything drawn since the last call to Panels. */
    public static void update() {
        if (broken) return;
        try {
            field().update();
        } catch (Throwable t) {
            broken = true;
        }
    }
}
