package org.firstinspires.ftc.teamcode.pedro.procedures;

import com.pedropathing.math.Pose;
import com.pedropathing.revhub.localizers.PinpointConfig;
import com.pedropathing.revhub.localizers.PinpointLocalizer;
import com.pedropathing.tuning.autotune.Inputs;
import com.pedropathing.tuning.autotune.Procedure;
import com.pedropathing.tuning.autotune.TuningOpMode;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.pedro.Constants;

import java.util.*;

public class PinpointTuner extends Procedure {
    enum PodType {
        SWING_ARM,
        FOUR_BAR,
        CUSTOM
    }
    static GoBildaPinpointDriver.EncoderDirection encoderDirection(boolean reversed) {
        if (reversed) {
            return GoBildaPinpointDriver.EncoderDirection.REVERSED;
        }
        return GoBildaPinpointDriver.EncoderDirection.FORWARD;
    }

    static GoBildaPinpointDriver.GoBildaOdometryPods standardPod(PodType podType) {
        if (podType == PodType.SWING_ARM) {
            return GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_SWINGARM_POD;
        }
        return GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD;
    }

    public PinpointTuner() {
        super("Pinpoint Tuner", "A procedure for tuning the Pinpoint localizer.");
    }

    @Override
    public void run() throws InterruptedException {
        Inputs inputs = inputs("Setup", "Set Pinpoint HardwareMap Name and Odometry Pod Type");
        Inputs.Field<String> pinpointName = inputs.s("HardwareMap Name").withDefault(Constants.PINPOINT_NAME);
        Inputs.Field<PodType> podType = inputs.e("Odometry Pod Type", PodType.class).withDefault(PodType.FOUR_BAR);
        awaitInputs(inputs);

        OptionalDouble customPodScalar = OptionalDouble.empty();

        if (podType.get() == PodType.CUSTOM) {
            Inputs inputsCustom = inputs("Custom Scalar Identification Push Distance", "Set the distance you will push your robot forward in inches");
            Inputs.Field<Double> distance = inputsCustom.d("Distance").withDefault(48.0);
            awaitInputs(inputsCustom);
            customPodScalar = OptionalDouble.of(runOpMode(new PinpointCustomPodScalar(distance.get(), pinpointName.get())));
        }

        boolean forwardPodReversed = runOpMode(new PinpointForwardDirection(pinpointName.get(), podType.get(), customPodScalar));
        boolean strafePodReversed = runOpMode(new PinpointStrafeDirection(pinpointName.get(), podType.get(), customPodScalar));

        List<Double> offsets = runOpMode(new PinpointOffsets(pinpointName.get(), podType.get(), customPodScalar, forwardPodReversed, strafePodReversed));

        result("name", pinpointName.get());

        String podConfigLine;
        if (customPodScalar.isPresent()) {
            result("podType", "Custom");
            result("ticksPerUnit", customPodScalar.getAsDouble());
            podConfigLine = "    c.ticksPerUnit.set(OptionalDouble.of(" + customPodScalar.getAsDouble() + "));\n";
        } else {
            GoBildaPinpointDriver.GoBildaOdometryPods pod = standardPod(podType.get());
            result("podType", pod);
            podConfigLine = "    c.podType.set(GoBildaPinpointDriver.GoBildaOdometryPods." + pod.name() + ");\n";
        }

        GoBildaPinpointDriver.EncoderDirection xDirection = encoderDirection(forwardPodReversed);
        GoBildaPinpointDriver.EncoderDirection yDirection = encoderDirection(strafePodReversed);
        result("xPodDirection", xDirection);
        result("yPodDirection", yDirection);
        result("xPodOffset", offsets.get(0));
        result("yPodOffset", offsets.get(1));

        code(Language.JAVA,"public static PinpointConfig localizerConfig = new PinpointConfig(c -> {\n" +
                "    c.name.set(\"" + pinpointName.get() + "\");\n" +
                podConfigLine +
                "    c.xPodOffset.set(" + offsets.get(0) + ");\n" +
                "    c.yPodOffset.set(" + offsets.get(1) + ");\n" +
                "    c.xPodDirection.set(GoBildaPinpointDriver.EncoderDirection." + xDirection.name() + ");\n" +
                "    c.yPodDirection.set(GoBildaPinpointDriver.EncoderDirection." + yDirection.name() + ");\n" +
                "    c.globalDistanceUnit.set(DistanceUnit.INCH);\n" +
                "    c.offsetUnits.set(DistanceUnit.INCH);\n" +
                "});");
    }
}

class PinpointCustomPodScalar extends TuningOpMode<Double> {

    String name;
    double distance;

    public PinpointCustomPodScalar(Double distance, String name) {
        super("Custom Scalar Identification",
                "Determines the scalar for the custom pods of the Pinpoint localizer. \n"
                        + "Push your robot forward " + distance + " inches exactly and then stop the Opmode",
                true);
        this.name = name;
        this.distance = distance;
    }

    @Override
    protected Double runTuningOpMode() throws InterruptedException {
        PinpointConfig config = new PinpointConfig(c -> {
            c.name.set(name);
            c.ticksPerUnit.set(OptionalDouble.of(1.0));
            c.xPodDirection.set(GoBildaPinpointDriver.EncoderDirection.FORWARD);
            c.yPodDirection.set(GoBildaPinpointDriver.EncoderDirection.FORWARD);
            c.xPodOffset.set(0.0);
            c.yPodOffset.set(0.0);
        });
        PinpointLocalizer localizer = new PinpointLocalizer(hardwareMap, config);
        localizer.setPose(new Pose(0, 0));
        Thread.sleep(1000);
        waitForStart();
        localizer.setPose(Pose.zero());
        localizer.update();
        while (!isStopRequested()) {
            localizer.update();
        }
        return Math.abs((localizer.pose().x() / distance));
    }
}

class PinpointForwardDirection extends TuningOpMode<Boolean> {
    String name;
    PinpointTuner.PodType podType;
    OptionalDouble customPodScalar;

    public PinpointForwardDirection(String name, PinpointTuner.PodType podType, OptionalDouble customPodScalar) {
        super("Forward Direction Identification",
                "Determines if your forward pod needs to be reversed. \n"
                        + "Push your robot forward and then stop the Opmode",
                true);
        this.name = name;
        this.podType = podType;
        this.customPodScalar = customPodScalar;
    }

    @Override
    protected Boolean runTuningOpMode() throws InterruptedException {
        PinpointConfig config = new PinpointConfig(c -> {
            c.name.set(name);
            c.xPodDirection.set(GoBildaPinpointDriver.EncoderDirection.FORWARD);
            c.yPodDirection.set(GoBildaPinpointDriver.EncoderDirection.FORWARD);
            c.xPodOffset.set(0.0);
            c.yPodOffset.set(0.0);
            if (customPodScalar.isPresent()) {
                c.encoderResolutionUnit.set(DistanceUnit.INCH);
                c.ticksPerUnit.set(OptionalDouble.of(customPodScalar.getAsDouble()));
            } else {
                c.podType.set(PinpointTuner.standardPod(podType));
            }
        });
        PinpointLocalizer localizer = new PinpointLocalizer(hardwareMap, config);
        localizer.setPose(new Pose(0, 0));
        Thread.sleep(1000);
        waitForStart();
        localizer.setPose(Pose.zero());
        localizer.update();

        // Keep the last reading taken before Stop. After Stop the Pinpoint reports a reset
        // pose (x = 0), so reading it afterwards always returns false (not reversed).
        double x = 0;
        while (!isStopRequested()) {
            localizer.update();
            if (!isStopRequested()) {
                x = localizer.pose().x();
            }
        }

        return x < 0;
    }
}

class PinpointStrafeDirection extends TuningOpMode<Boolean> {
    String name;
    PinpointTuner.PodType podType;
    OptionalDouble customPodScalar;

    public PinpointStrafeDirection(String name, PinpointTuner.PodType podType, OptionalDouble customPodScalar) {
        super("Strafe Direction Identification",
                "Determines if your strafe pod needs to be reversed. \n"
                        + "Push your robot to the left and then stop the Opmode",
                true);
        this.name = name;
        this.podType = podType;
        this.customPodScalar = customPodScalar;
    }

    @Override
    protected Boolean runTuningOpMode() throws InterruptedException {
        PinpointConfig config = new PinpointConfig(c -> {
            c.name.set(name);
            c.xPodDirection.set(GoBildaPinpointDriver.EncoderDirection.FORWARD);
            c.yPodDirection.set(GoBildaPinpointDriver.EncoderDirection.FORWARD);
            c.xPodOffset.set(0.0);
            c.yPodOffset.set(0.0);
            if (customPodScalar.isPresent()) {
                c.encoderResolutionUnit.set(DistanceUnit.INCH);
                c.ticksPerUnit.set(OptionalDouble.of(customPodScalar.getAsDouble()));
            } else {
                c.podType.set(PinpointTuner.standardPod(podType));
            }
        });
        PinpointLocalizer localizer = new PinpointLocalizer(hardwareMap, config);
        localizer.setPose(new Pose(0, 0));
        Thread.sleep(1000);
        waitForStart();
        localizer.setPose(Pose.zero());
        localizer.update();
        // Keep the last reading taken before Stop. After Stop the Pinpoint reports a reset
        // pose (y = 0), so reading it afterwards always returns false (not reversed).
        double y = 0;
        while (!isStopRequested()) {
            localizer.update();
            if (!isStopRequested()) {
                y = localizer.pose().y();
            }
        }

        return y < 0;
    }
}

class PinpointOffsets extends TuningOpMode<List<Double>> {
    String name;
    PinpointTuner.PodType podType;
    OptionalDouble customPodScalar =  OptionalDouble.empty();
    boolean forwardPodReversed, strafePodReversed;
    private Pose previous = Pose.zero();

    public PinpointOffsets(String name, PinpointTuner.PodType podType, OptionalDouble customPodScalar, Boolean forwardPodReversed, Boolean strafePodReversed) {
        super("Offsets Identification",
                "Automatically identifies the offsets for your Pinpoint localizer. \n"
                        + "Spin your robot in place 180 degrees counterclockwise and then stop the Opmode",
                true);
        this.name = name;
        this.podType = podType;
        if (customPodScalar.isPresent()) {
            this.customPodScalar = customPodScalar;
        }
        this.forwardPodReversed = forwardPodReversed;
        this.strafePodReversed = strafePodReversed;
    }

    @Override
    protected List<Double> runTuningOpMode() throws InterruptedException {
        PinpointConfig config = new PinpointConfig(c -> {
            c.name.set(name);
            c.xPodDirection.set(PinpointTuner.encoderDirection(forwardPodReversed));
            c.yPodDirection.set(PinpointTuner.encoderDirection(strafePodReversed));
            if (customPodScalar.isPresent()) {
                c.encoderResolutionUnit.set(DistanceUnit.INCH);
                c.ticksPerUnit.set(OptionalDouble.of(customPodScalar.getAsDouble()));
                c.resetMode.set(PinpointLocalizer.ResetMode.RESET_AND_RECALIBRATE_IMU);
            } else {
                c.podType.set(PinpointTuner.standardPod(podType));
            }
            c.xPodOffset.set(0.0);
            c.yPodOffset.set(0.0);
            c.globalDistanceUnit.set(DistanceUnit.INCH);
            c.offsetUnits.set(DistanceUnit.INCH);
        });
        PinpointLocalizer localizer = new PinpointLocalizer(hardwareMap, config);
        if (customPodScalar.isPresent()) {
            localizer.reset();
        }
        localizer.setPose(Pose.zero());
        localizer.update();


        Thread.sleep(1000);
        waitForStart();
        localizer.setPose(Pose.zero());
        localizer.update();

        localizer.setPose(Pose.zero());

        while (!isStopRequested()) {
            previous = localizer.pose();
            localizer.update();

            telemetry.addData("heading", localizer.pose().heading());
            telemetry.addData("pose", localizer.pose());
            telemetry.addData("previous", previous);
            telemetry.update();
        }

        if (localizer.pose().x() != Pose.zero().x() || localizer.pose().y() != Pose.zero().y()) {
            previous =  localizer.pose();
        }

        return List.of(((-previous.y()) / 2.0), ((-previous.x()) / 2.0));
    }
}