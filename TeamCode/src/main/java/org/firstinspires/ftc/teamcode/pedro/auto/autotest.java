package org.firstinspires.ftc.teamcode.pedro.auto;
import com.pedropathing.api.Paths;
import com.pedropathing.ivy.Command;

import org.firstinspires.ftc.teamcode.pedro.Constants;

import com.pedropathing.paths.Path;
import com.pedropathing.ivy.Scheduler;
import static com.pedropathing.ivy.Scheduler.schedule;
import static com.pedropathing.ivy.groups.Groups.sequential;
import static com.pedropathing.ivy.pedro.PedroCommands.follow;
import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;

import static com.pedropathing.api.Paths.*;
import com.pedropathing.api.Paths;

import com.pedropathing.api.PoseFactory;
import com.pedropathing.math.Pose;
import com.pedropathing.paths.Path;

public class Paths {

    private final PoseFactory poseFactory = PoseFactory.degrees();

    private final Pose start = poseFactory.of(56, 8, 90);
    private final Pose path1 = poseFactory.of(55.8514, 14.2433, 90);
    private final Pose point2Start = poseFactory.of(55.8514, 14.2433, 90);
    private final Pose point2 = poseFactory.of(8.8206, 14.0719, 270);
    private final Pose point3Start = poseFactory.of(8.8206, 14.0719, 270);
    private final Pose point3 = poseFactory.of(44.9208, 125.7566, 90);
    private final Pose point4 = poseFactory.of(54.1112, 10, 129.8046);
    private final Pose point4Control1 = poseFactory.of(14.5018, 57.5329, 0);
    private final Pose point5Start = poseFactory.of(54.1112, 10, 90);
    private final Pose point5 = poseFactory.of(55.5241, 12.9606, 90);
    private final Pose point6 = poseFactory.of(7.714, 102.0362, 125);

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
        return Paths.curve(point3, point4Control1, point4).reverseTangent();
    }

    public Path path5() {
        return Paths.line(point5Start, point5).linear(point5Start, point5);
    }

    public Path path6() {
        return Paths.line(point5, point6).constant(point6);
    }
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