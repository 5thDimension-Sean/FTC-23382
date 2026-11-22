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
    private final Pose path1 = poseFactory.of(55.8514, 14.2433, 90);
    private final Pose point2Start = poseFactory.of(55.8514, 14.2433, 90);
    private final Pose point2 = poseFactory.of(8.8206, 14.0719, 270);
    private final Pose point3Start = poseFactory.of(8.8206, 14.0719, 270);
    private final Pose point3 = poseFactory.of(8.512, 8.6216, 270);
    private final Pose point4Start = poseFactory.of(8.512, 8.6216, 270);
    private final Pose point4 = poseFactory.of(45.1026, 125.7188, 90);
    private final Pose point4Control1 = poseFactory.of(13.4984, 66.2367, 0);
    private final Pose point5Start = poseFactory.of(45.1026, 125.7188, 90);
    private final Pose point5 = poseFactory.of(46.2094, 127.6129, 90);
    private final Pose point6Start = poseFactory.of(46.2094, 127.6129, 90);
    private final Pose point6 = poseFactory.of(12.2993, 46.7395, 180);
    private final Pose point6Control1 = poseFactory.of(12.5862, 56.5525, 0);
    private final Pose point7Start = poseFactory.of(12.2993, 46.7395, 90);
    private final Pose point7 = poseFactory.of(55.5241, 12.9606, 90);
    private final Pose point8Start = poseFactory.of(55.5241, 12.9606, 90);
    private final Pose point8 = poseFactory.of(4.5766, 99.7708, 360);
    private final Pose point8Control1 = poseFactory.of(12.3073, 38.8354, 0);

    public Path path1() {
        return Paths.line(start, path1).linear(start, path1);
    }

    public Path path2() {
        return Paths.line(point2Start, point2).linear(point2Start, point2);
    }

    public Path path3() {
        return Paths.line(point3Start, point3).linear(point3Start, point3);
    }

    public Path path4() {
        return Paths.curve(point4Start, point4Control1, point4).linear(point4Start, point4);
    }

    public Path path5() {
        return Paths.line(point5Start, point5).linear(point5Start, point5);
    }

    public Path path6() {
        return Paths.curve(point6Start, point6Control1, point6).linear(point6Start, point6);
    }

    public Path path7() {
        return Paths.line(point7Start, point7).linear(point7Start, point7);
    }

    public Path path8() {
        return Paths.curve(point8Start, point8Control1, point8).linear(point8Start, point8);
    }



    private Command autoRoutine() {
        return sequential(
                follow(follower, path1()),
                follow(follower, path2()),
                follow(follower, path3()),
                follow(follower, path4()),
                follow(follower, path5()),
                follow(follower, path6()),
                follow(follower, path7()),
                follow(follower, path8())


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
