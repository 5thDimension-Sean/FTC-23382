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
@Autonomous(name = "sohamAuto", group = "Auto")
public class sohamAuto extends OpMode {
    private Follower follower;
    private final PoseFactory poseFactory = PoseFactory.degrees();

    private final Pose start = poseFactory.of(56, 8, 90);
    private final Pose path1 = poseFactory.of(56, 18, 90);
    private final Pose point2Start = poseFactory.of(56, 18, 90);
    private final Pose point2 = poseFactory.of(22.3931, 46.5549, 180);
    private final Pose point3Start = poseFactory.of(22.3931, 46.5549, 180);
    private final Pose point3 = poseFactory.of(58.0192, 116.299, 270);
    private final Pose point3Control1 = poseFactory.of(20.6077, 101.076, 0);
    private final Pose point3Control2 = poseFactory.of(31.8137, 116.967, 0);


    public Path path1() {
        return Paths.line(start, path1).linear(start, path1);
    }

    public Path path2() {
        return Paths.line(point2Start, point2).linear(point2Start, point2);
    }

    public Path path3() {
        return Paths.curve(point3Start, point3Control1, point3Control2, point3).linear(point3Start, point3);
    }
    private Command autoRoutine() {
        return sequential(
                follow(follower, path1()),
                follow(follower, path2()),
                follow(follower, path3())
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
