package org.firstinspires.ftc.teamcode;

import com.pedropathing.api.Paths;
import com.pedropathing.api.PoseFactory;
import com.pedropathing.follower.Follower;
import com.pedropathing.paths.Path;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.pedro.Constants;

@Autonomous
public class Onefootrightauto extends LinearOpMode {
    Follower follower;
    PoseFactory p = PoseFactory.degrees();
    public Path path3_alt() {
        return  Paths.line(p.of(0, 0, 90), p.of(12, 0, 90)).constant(p.of(0, 0, 90));
    }
    @Override
    public void runOpMode() {
        // INIT
        follower = Constants.create(hardwareMap);
        follower.setPose(p.of(0, 0, 90));
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();
        //START
        telemetry.addData("Status", "Running");
        telemetry.update();
        follower.follow(path3_alt());
        while (opModeIsActive()) {
            //ITERATIVE
            follower.update();
        }
        //DEINIT
    }
}
