package org.firstinspires.ftc.teamcode;

import com.pedropathing.api.Paths;
import com.pedropathing.api.PoseFactory;
import com.pedropathing.follower.Follower;
import com.pedropathing.ivy.Command;
import static com.pedropathing.ivy.Scheduler.schedule;
import static com.pedropathing.ivy.commands.Commands.conditional;
import static com.pedropathing.ivy.commands.Commands.instant;
import static com.pedropathing.ivy.commands.Commands.waitMs;
import static com.pedropathing.ivy.commands.Commands.waitUntil;
import static com.pedropathing.ivy.groups.Groups.parallel;
import static com.pedropathing.ivy.groups.Groups.race;
import static com.pedropathing.ivy.groups.Groups.sequential;
import static com.pedropathing.ivy.pedro.PedroCommands.follow;
import com.pedropathing.ivy.CommandBuilder;
import com.pedropathing.ivy.Scheduler;
import com.pedropathing.math.Pose;
import com.pedropathing.paths.Path;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.pedro.Constants;

@Autonomous
public class RedAuto extends LinearOpMode {
    Robot robot;
    Follower follower;
    private final PoseFactory poseFactory = PoseFactory.degrees();

    private final Pose start = poseFactory.of(58, 9, 90);
    private final Pose point1 = poseFactory.of(9, 9, 90);
    private final Pose path1Control1 = poseFactory.of(30.913, 29.7624, 0);
    private final Pose point2 = poseFactory.of(59.1, 132.5, -90);
    private final Pose point2Control1 = poseFactory.of(24.3586, 35.9429, 0);
    private final Pose point2Control2 = poseFactory.of(10.8764, 127.6923, 0);
    private final Pose point2Control3 = poseFactory.of(69.0675, 99.0022, 0);
    private final Pose point3 = poseFactory.of(9, 96, 0);
    private final Pose point3Control1 = poseFactory.of(68.2832, 116.002, 0);
    private final Pose point3Control2 = poseFactory.of(22.8322, 81.8766, 0);

    public Path path1() {
        return Paths.curve(start, path1Control1, point1).constant(point1);
    }

    public Path path2() {
        return Paths.curve(point1, point2Control1, point2Control2, point2Control3, point2).linear(point1, point2);
    }
    public Path path2_alt() {
        return Paths.curve(point1, path1Control1, start).constant(point1);
    }
    public Path path3() {
        return Paths.curve(point2, point3Control1, point3Control2, point3).linear(point2, point3);
    }
    public Path path3_alt() {
        return  Paths.curve(start, point2Control1, point3).linear(start, point3);
    }
    Command run() {
        return sequential(
            shoot(),
            parallel(
                    follow(follower, path1()),
                    instant(() -> robot.setIntakePower(1))
            ),
            instant(() -> robot.setIntakePower(0)),
            follow(follower, path2()),
            shoot(),
            follow(follower, path3())
        );
    }
    Command shoot() {
        return sequential(
                instant(() -> robot.pollenShooter.setPower(1)),
                waitMs(500),
                instant(() -> robot.setIndexerOpen(true)),
                waitMs(1000),
                instant(() -> robot.pollenShooter.setPower(0)),
                instant(() -> robot.setIndexerOpen(false))
        );
    }
    @Override
    public void runOpMode() {
        // INIT
        robot = new Robot(hardwareMap);
        follower = Constants.create(hardwareMap);
        follower.setPose(point1);
        Scheduler.reset();
        telemetry.addData("Status", "Initialized");
        telemetry.update();
        while (!isStarted() && !isStopRequested()) {
            follower.update();
            telemetry.addData("position", "x: %.2f y %.2f", follower.pose().x(), follower.pose().y());
            telemetry.update();
        }
        if (isStopRequested()) {
            return;
        }

        //START
        telemetry.addData("Status", "Running");
        telemetry.update();
        schedule(run());
        while (opModeIsActive()) {
            //ITERATIVE
            follower.update();
            Scheduler.execute();
            telemetry.addData("position", "x: %.2f y %.2f", follower.pose().x(), follower.pose().y());
            telemetry.update();
        }
        //DEINIT
    }
}
