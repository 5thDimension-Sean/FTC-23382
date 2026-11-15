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
import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;

@Autonomous(name = "Solo_R", group = "Auto")
public class soloRed extends OpMode {
    private Follower follower;
    private final PoseFactory poseFactory = PoseFactory.degrees();
    private final Pose start = poseFactory.of(58.75, 9.5, 270);
    private final Pose garden = poseFactory.of(7, 10, 270);
    private final Pose gardenControl1 = poseFactory.of(27, 30, 0);
    private final Pose centerStart = poseFactory.of(7, 10, 270);
    private final Pose center = poseFactory.of(25, 10, 270);
    private final Pose travelSideShootStart = poseFactory.of(25, 10, 270);
    private final Pose travelSideShoot = poseFactory.of(47, 120, 90);
    private final Pose travelSideShootControl1 = poseFactory.of(24.7546, 36.9419, 0);
    private final Pose travelSideShootControl2 = poseFactory.of(23.9949, 118.8663, 0);
    private final Pose goToFlower = poseFactory.of(47, 123, 90);
    private final Pose backup = poseFactory.of(47, 115, 90);
    private final Pose goToGoalScoreStart = poseFactory.of(47, 115, 90);
    private final Pose goToGoalScore = poseFactory.of(23.672, 47, 180);
    private final Pose goToGoalScoreControl1 = poseFactory.of(26.7172, 113.6748, 0);
    private final Pose goToFlower2 = poseFactory.of(16, 47, 180);
    private final Pose backup2 = poseFactory.of(23, 47, 180);
    private final Pose goalStart = poseFactory.of(23, 47, 180);
    private final Pose goal = poseFactory.of(58.75, 15, 180);
    private final Pose park = poseFactory.of(11.206, 91.4527, 91.6218);
    private final Pose parkControl1 = poseFactory.of(12.7888, 35.5491, 0);

    public Path garden() {
        return Paths.curve(start, gardenControl1, garden).linear(start, garden);
    }

    public Path center() {
        return Paths.line(centerStart, center).linear(centerStart, center);
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

    public Path goToGoalScore() {
        return Paths.curve(goToGoalScoreStart, goToGoalScoreControl1, goToGoalScore).linear(goToGoalScoreStart, goToGoalScore);
    }

    public Path goToFlower2() {
        return Paths.line(goToGoalScore, goToFlower2).tangent();
    }

    public Path backup2() {
        return Paths.line(goToFlower2, backup2).reverseTangent();
    }

    public Path goal() {
        return Paths.line(goalStart, goal).linear(goalStart, goal);
    }

    public Path park() {
        return Paths.curve(goal, parkControl1, park).tangent();
    }
    private Command autoRoutine() {
        return sequential(
                follow(follower, garden()),
                follow(follower, center()),
                follow(follower, travelSideShoot()),
                follow(follower, goToFlower()),
                follow(follower, backup()),
                follow(follower, goToGoalScore()),
                follow(follower, goToFlower2()),
                follow(follower, backup2()),
                follow(follower, goal()),
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
