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

    private final Pose start = poseFactory.of(58.75, 9.5, 180);
    private final Pose garden = poseFactory.of(9.5, 10, 180);
    private final Pose gardenControl1 = poseFactory.of(27, 30, 0);
    private final Pose moveToOtherSideStart = poseFactory.of(9.5, 10, 180);
    private final Pose moveToOtherSide = poseFactory.of(47, 115, 0);
    private final Pose moveToOtherSideControl1 = poseFactory.of(35, 41, 0);
    private final Pose moveToOtherSideControl2 = poseFactory.of(8, 108, 0);
    private final Pose shootloadStart = poseFactory.of(47, 115, 0);
    private final Pose shootload = poseFactory.of(47, 126, 0);
    private final Pose goToLoaderStart = poseFactory.of(47, 126, 0);
    private final Pose goToLoader = poseFactory.of(27, 47, 90);
    private final Pose goToLoaderControl1 = poseFactory.of(26, 97, 0);
    private final Pose collectBallsStart = poseFactory.of(27, 47, 90);
    private final Pose collectBalls = poseFactory.of(14, 47, 90);
    private final Pose goToStartShootStart = poseFactory.of(14, 47, 90);
    private final Pose goToStartShoot = poseFactory.of(58.75, 9.5, 180);
    private final Pose parkStart = poseFactory.of(58.75, 9.5, 180);
    private final Pose park = poseFactory.of(16, 95, 90);
    private final Pose parkControl1 = poseFactory.of(22, 30, 0);

    public Path garden() {
        return Paths.curve(start, gardenControl1, garden).linear(start, garden);
    }

    public Path moveToOtherSide() {
        return Paths.curve(moveToOtherSideStart, moveToOtherSideControl1, moveToOtherSideControl2, moveToOtherSide).linear(moveToOtherSideStart, moveToOtherSide);
    }

    public Path shootLoad() {
        return Paths.line(shootloadStart, shootload).linear(shootloadStart, shootload);
    }

    public Path goToLoader() {
        return Paths.curve(goToLoaderStart, goToLoaderControl1, goToLoader).linear(goToLoaderStart, goToLoader);
    }

    public Path collectBalls() {
        return Paths.line(collectBallsStart, collectBalls).linear(collectBallsStart, collectBalls);
    }

    public Path goToStartShoot() {
        return Paths.line(goToStartShootStart, goToStartShoot).linear(goToStartShootStart, goToStartShoot);
    }

    public Path park() {
        return Paths.curve(parkStart, parkControl1, park).linear(parkStart, park);
    }
    // The auto's paths, in order. The manipulator both drives them and uses them for the time estimate.
    private Path[] routinePaths() {
        return new Path[]{garden(), moveToOtherSide(), shootLoad(), goToLoader(), collectBalls(), goToStartShoot()};
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
