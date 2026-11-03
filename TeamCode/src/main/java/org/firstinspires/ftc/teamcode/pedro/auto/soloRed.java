package org.firstinspires.ftc.teamcode.pedro.auto;
import com.pedropathing.api.Paths;
import com.pedropathing.ivy.Command;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.pedropathing.follower.Follower;
import org.firstinspires.ftc.teamcode.pedro.Alliance;
import org.firstinspires.ftc.teamcode.pedro.Constants;
import org.firstinspires.ftc.teamcode.pedro.FieldDrawer;
import org.firstinspires.ftc.teamcode.pedro.RobotPose;
import com.pedropathing.api.PoseFactory;
import com.pedropathing.math.Pose;
import static com.pedropathing.api.Paths.*;
import com.pedropathing.paths.Path;
import com.pedropathing.ivy.Scheduler;
import static com.pedropathing.ivy.Scheduler.schedule;
import static com.pedropathing.ivy.groups.Groups.sequential;
import static com.pedropathing.ivy.pedro.PedroCommands.follow;
import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;

@Autonomous(name = "soloRed", group = "Auto")
public class soloRed extends OpMode {
    private Follower follower;
    private Path[] estimateRoutine, estimatePark; // built once in init for the time estimate
    private final PoseFactory poseFactory = PoseFactory.degrees();

    private final Pose start = poseFactory.of(58.75, 9.5, 270);
    private final Pose garden = poseFactory.of(7, 10, 270);
    private final Pose gardenControl1 = poseFactory.of(27, 30, 0);
    private final Pose collectMoveStart = poseFactory.of(7, 10, 270);
    private final Pose collectMove = poseFactory.of(25, 10, 270);
    private final Pose travelSideShootStart = poseFactory.of(25, 10, 270);
    private final Pose travelSideShoot = poseFactory.of(47, 120, 90);
    private final Pose travelSideShootControl1 = poseFactory.of(22.3488, 36.5621, 0);
    private final Pose travelSideShootControl2 = poseFactory.of(23.9949, 118.8663, 0);
    private final Pose goToFlower = poseFactory.of(47, 123, 90);
    private final Pose backup = poseFactory.of(47, 115, 90);
    private final Pose goToFlower2 = poseFactory.of(24, 47, 92.0431);
    private final Pose goToFlower2Control1 = poseFactory.of(22.2855, 95.0614, 0);
    private final Pose pickup2 = poseFactory.of(19, 47, 180);
    private final Pose backup2 = poseFactory.of(27.5, 47, 180);
    private final Pose backupShoot = poseFactory.of(58.75, 15, 134.3206);
    private final Pose park = poseFactory.of(11.5226, 91.6426, 87.2125);
    private final Pose parkControl1 = poseFactory.of(9.0535, 40.9305, 0);

    public Path garden() {
        return Paths.curve(start, gardenControl1, garden).linear(start, garden);
    }

    public Path collectMove() {
        return Paths.line(collectMoveStart, collectMove).linear(collectMoveStart, collectMove);
    }

    public Path travelSideShoot() {
        return Paths.curve(travelSideShootStart, travelSideShootControl1, travelSideShootControl2, travelSideShoot).linear(travelSideShootStart, travelSideShoot);
    }

    public Path goToFlower() {
        return Paths.line(travelSideShoot, goToFlower).tangent();
    }

    public Path backup() {
        return Paths.line(goToFlower, backup).reverseTangent();
    }

    public Path goToFlower2() {
        return Paths.curve(backup, goToFlower2Control1, goToFlower2).reverseTangent();
    }

    public Path pickup2() {
        return Paths.line(goToFlower2, pickup2).tangent();
    }

    public Path backup2() {
        return Paths.line(pickup2, backup2).reverseTangent();
    }

    public Path backupShoot() {
        return Paths.line(backup2, backupShoot).reverseTangent();
    }

    public Path park() {
        return Paths.curve(backupShoot, parkControl1, park).tangent();
    }
    // The auto's paths, in order. The manipulator both drives them and uses them for the time estimate.
    private Path[] routinePaths() {
        return new Path[]{garden(), collectMove(), travelSideShoot(), goToFlower(), backup(), goToFlower2(), pickup2(), backup2(), backupShoot()};
    }
    // Kept separate so the "Park at end" setting can skip it.
    private Path[] parkPaths() {
        return new Path[]{park()};
    }
    @Override
    public void init() {
        Scheduler.reset();
        estimateRoutine = routinePaths();
        estimatePark = parkPaths();
        follower = Constants.create(hardwareMap);
        // The auto's first pose is also the shared start pose that teleop continues from.
        RobotPose.startAuto(follower, start, Alliance.RED);
    }
    @Override
    public void init_loop() {
        autoManipulator.initLoop(gamepad1, telemetry, start, estimateRoutine, estimatePark);
    }
    @Override
    public void start() {
        schedule(autoManipulator.start(follower,
                autoManipulator.followAll(follower, routinePaths()),
                autoManipulator.followAll(follower, parkPaths())));
    }
    @Override
    public void loop() {
        follower.update();
        RobotPose.update(follower);
        Scheduler.execute();

        FieldDrawer.drawRobot(follower.pose());
        FieldDrawer.update();

        telemetry.addData("X", follower.pose().x());
        telemetry.addData("Y", follower.pose().y());
        telemetry.addData("Heading", Math.toDegrees(follower.pose().heading()));
        telemetry.update();
    }
    @Override
    public void stop() {
        // Hands the speed limit back and saves the last pose so teleop starts where we ended up.
        autoManipulator.stop(follower);
    }
}
