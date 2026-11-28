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
@Autonomous(name = "Left_R", group = "Auto", preselectTeleOp = Constants.DRIVER_TELEOP_NAME)
public class FarRed extends OpMode {
    private Follower follower;
    private Command routine;
    private final PoseFactory poseFactory = PoseFactory.degrees();

    private final Pose start = poseFactory.of(57.75, 131.85, 90); // change starting positions
    private final Pose waitShoot = poseFactory.of(47, 115, 90);
    private final Pose waitShootControl1 = poseFactory.of(58, 116, 0);
    private final Pose goToFlower = poseFactory.of(47, 120, 90);
    private final Pose backUp = poseFactory.of(47, 115, 90);
    private final Pose parkStart = poseFactory.of(47, 115, 90);
    private final Pose park = poseFactory.of(8, 123, 90);

    public Path waitShoot() {
        return Paths.curve(start, waitShootControl1, waitShoot).linear(start, waitShoot);
    }

    public Path goToFlower() {
        return Paths.line(waitShoot, goToFlower).tangent();
    }

    public Path backUp() {
        return Paths.line(goToFlower, backUp).reverseTangent();
    }

    public Path park() {
        return Paths.line(parkStart, park).linear(parkStart, park);
    }
    private Command autoRoutine() {
        return sequential(
                follow(follower, waitShoot()),
                follow(follower, goToFlower()),
                follow(follower, backUp()),
                follow(follower, park())
        );
    }

    @Override
    public void init() {
        Constants.selectedTeam = Constants.colorRight;
        Scheduler.reset();
        follower = Constants.create(hardwareMap);
        follower.setPose(start);
        follower.update();
    }
    @Override
    public void start() {
        routine = autoRoutine();
        schedule(routine);
    }
    @Override
    public void loop() {
        follower.update();
        Scheduler.execute();
        if (!Scheduler.isScheduled(routine)) {
            requestOpModeStop();
        }

        telemetry.addData("X", follower.pose().x());
        telemetry.addData("Y", follower.pose().y());
        telemetry.addData("Heading", Math.toDegrees(follower.pose().heading()));
        telemetry.update();
    }
}
