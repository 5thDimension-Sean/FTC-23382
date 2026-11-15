package org.firstinspires.ftc.teamcode.pedro.auto;
import com.pedropathing.api.Paths;
import com.pedropathing.ivy.Command;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.pedropathing.follower.Follower;
import org.firstinspires.ftc.teamcode.pedro.Constants;
import com.pedropathing.api.PoseFactory;
import com.pedropathing.math.Pose;
import static com.pedropathing.api.Paths.*;
import com.pedropathing.paths.Path;
import com.pedropathing.ivy.Scheduler;
import static com.pedropathing.ivy.Scheduler.schedule;
import static com.pedropathing.ivy.groups.Groups.sequential;
import static com.pedropathing.ivy.pedro.PedroCommands.follow;
@Autonomous(name = "Flower_R", group = "Auto")
public class FlowerRed extends OpMode {
    private Follower follower;
    private final PoseFactory poseFactory = PoseFactory.degrees();

    private final Pose start = poseFactory.of(58.75, 9.5, 270);
    private final Pose garden = poseFactory.of(7, 10, 270);
    private final Pose gardenControl1 = poseFactory.of(27, 30, 0);
    private final Pose centerStart = poseFactory.of(7, 10, 270);
    private final Pose center = poseFactory.of(50, 10, 270);
    private final Pose travelToFlowerStart = poseFactory.of(50, 10, 270);
    private final Pose travelToFlower = poseFactory.of(25, 47, 270);
    private final Pose travelToFlowerControl1 = poseFactory.of(26.2107, 32.3836, 0);
    private final Pose goToFlowerStart = poseFactory.of(25, 47, 180);
    private final Pose goToFlower = poseFactory.of(17, 47, 180);
    private final Pose backUp = poseFactory.of(25, 47, 0);
    private final Pose waitForTip = poseFactory.of(57, 16, 135.9094);
    private final Pose park = poseFactory.of(9.37, 93.0355, 94.9423);
    private final Pose parkControl1 = poseFactory.of(14.3083, 35.929, 0);

    public Path garden() {
        return Paths.curve(start, gardenControl1, garden).linear(start, garden);
    }

    public Path center() {
        return Paths.line(centerStart, center).linear(centerStart, center);
    }

    public Path travelToFlower() {
        return Paths.curve(travelToFlowerStart, travelToFlowerControl1, travelToFlower).linear(travelToFlowerStart, travelToFlower);
    }

    public Path goToFlower() {
        return Paths.line(goToFlowerStart, goToFlower).linear(goToFlowerStart, goToFlower);
    }

    public Path backUp() {
        return Paths.line(goToFlower, backUp).tangent();
    }

    public Path waitForTip() {
        return Paths.line(backUp, waitForTip).reverseTangent();
    }

    public Path park() {
        return Paths.curve(waitForTip, parkControl1, park).tangent();
    }
    private Command autoRoutine() {
        return sequential(
                follow(follower, garden()),
                follow(follower, center()),
                follow(follower, travelToFlower()),
                follow(follower, goToFlower()),
                follow(follower, backUp()),
                follow(follower, waitForTip()),
                follow(follower, park())
        );
    }

    @Override
    public void init() {
        Scheduler.reset();
        follower = Constants.create(hardwareMap);
        follower.setPose(start);
        follower.update();
    }
    @Override
    public void start() {
        schedule(autoRoutine());
    }
    @Override
    public void loop() {
        follower.update();
        Scheduler.execute();

        telemetry.addData("X", follower.pose().x());
        telemetry.addData("Y", follower.pose().y());
        telemetry.addData("Heading", Math.toDegrees(follower.pose().heading()));
        telemetry.update();
    }
}
