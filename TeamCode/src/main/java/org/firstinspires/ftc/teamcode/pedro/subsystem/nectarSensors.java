package org.firstinspires.ftc.teamcode.pedro.subsystem;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.pedro.Constants;

import com.qualcomm.robotcore.hardware.DistanceSensor;

/**
 * Two REV color sensors (colorLeft, colorRight). NECTAR is the red or blue ball; POLLEN is yellow.
 * A sensor sees NECTAR when something is close and its color is clearly red or clearly blue.
 */
public class nectarSensors {
    /** A ball closer than this to the sensor counts as present (REV V3 distance, cm). TUNE. */
    public static double NEAR_CM = 5.0;
    /** How much one channel must exceed the others to count as red/blue (yellow has red ~ green). */
    public static double DOMINANCE = 1.4;

    private NormalizedColorSensor left;
    private NormalizedColorSensor right;

    public void init(HardwareMap hardwareMap) {
        left = hardwareMap.tryGet(NormalizedColorSensor.class, Constants.COLOR_LEFT_SENSOR_NAME);
        right = hardwareMap.tryGet(NormalizedColorSensor.class, Constants.COLOR_RIGHT_SENSOR_NAME);
    }

    /** True if either sensor currently sees a red or blue NECTAR. */
    public boolean nectarPresent(Telemetry telemetry) {
        boolean l = sees(left, "L", telemetry);
        boolean r = sees(right, "R", telemetry);
        return l || r;
    }

    private static boolean sees(NormalizedColorSensor sensor, String label, Telemetry telemetry) {
        if (sensor == null) {
            telemetry.addData("Color " + label, "missing from robot config");
            return false;
        }
        double cm = sensor instanceof DistanceSensor
                ? ((DistanceSensor) sensor).getDistance(DistanceUnit.CM) : 0;
        NormalizedRGBA c = sensor.getNormalizedColors();
        boolean near = !Double.isNaN(cm) && cm <= NEAR_CM;
        boolean red = c.red > DOMINANCE * c.green && c.red > DOMINANCE * c.blue;
        boolean blue = c.blue > DOMINANCE * c.red && c.blue > DOMINANCE * c.green * 0.85;
        telemetry.addData("Color " + label, "%.1f cm  r%.3f g%.3f b%.3f  %s", cm,
                c.red, c.green, c.blue, !near ? "none" : red ? "RED nectar" : blue ? "BLUE nectar" : "pollen/other");
        return near && (red || blue);
    }
}
