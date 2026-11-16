/*
Copyright 2026 FIRST Tech Challenge Team 16424

Permission is hereby granted, free of charge, to any person obtaining a copy of this software and
associated documentation files (the "Software"), to deal in the Software without restriction,
including without limitation the rights to use, copy, modify, merge, publish, distribute,
sublicense, and/or sell copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all copies or substantial
portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT
NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND
NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM,
DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
*/
package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.Robot;

/**
 * This file contains a minimal example of a Linear "OpMode". An OpMode is a 'program' that runs
 * in either the autonomous or the TeleOp period of an FTC match. The names of OpModes appear on
 * the menu of the FTC Driver Station. When an selection is made from the menu, the corresponding
 * OpMode class is instantiated on the Robot Controller and executed.
 *
 * Remove the @Disabled annotation on the next line or two (if present) to add this OpMode to the
 * Driver Station OpMode list, or add a @Disabled annotation to prevent this OpMode from being
 * added to the Driver Station.
 */
@TeleOp

public class TeleOperation extends LinearOpMode {
    Robot robot;

    static IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(RevHubOrientationOnRobot.LogoFacingDirection.FORWARD, RevHubOrientationOnRobot.UsbFacingDirection.RIGHT));

    @Override
    public void runOpMode() {
        robot = new Robot(hardwareMap);

        IMU imu;
        GoBildaPinpointDriver odo;
        imu = hardwareMap.get(IMU.class, "imu");
        imu.initialize(parameters);
        odo = hardwareMap.get(GoBildaPinpointDriver.class,"pinpoint");
        odo.resetPosAndIMU();

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        double forwardPower = 0;
        double strafePower = 0;
        double turnPower = 0;
        double forwardPowerTemp = 0;
        double strafePowerTemp = 0;
        boolean nectarShooterOn = false;
        double nectarShooterPower = 0.4;
        boolean pollenShooterOn = false;
        double pollenShooterPower = 0.4;
        boolean intakeOverride = false;
        boolean indexerOn = false;

        boolean intakeOn = false;
        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        double backLeftPower = 0.0;
        double backRightPower = 0.0;
        double frontLeftPower = 0.0;
        double frontRightPower = 0.0;
        Pose2D currentPos;

        //IMU
        imu = hardwareMap.get(IMU.class, "imu");
        imu.initialize(parameters);

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            odo.update();;
            currentPos = odo.getPosition();
            telemetry.addData("Status", "Running");

            strafePowerTemp = gamepad1.left_stick_x;
            forwardPowerTemp = gamepad1.left_stick_y;
            turnPower = gamepad1.right_stick_x;

            double currentHeading = odo.getHeading(AngleUnit.RADIANS);

            strafePower = strafePowerTemp * Math.cos(currentHeading)
                    - forwardPowerTemp * Math.sin(currentHeading);
            forwardPower = forwardPowerTemp * Math.cos(currentHeading)
                    + strafePowerTemp * Math.sin(currentHeading);


            robot.frontLeft.setPower(forwardPower + turnPower - strafePower);
            robot.frontRight.setPower(forwardPower - turnPower + strafePower);
            robot.backLeft.setPower(forwardPower + turnPower + strafePower);
            robot.backRight.setPower(forwardPower - turnPower - strafePower);

            if (gamepad1.dpadLeftWasPressed()) {
                if (nectarShooterPower > 0) {
                    nectarShooterPower -= 0.05;
                }
            }
            if (gamepad1.dpadRightWasPressed()) {
                if (nectarShooterPower < 1) {
                    nectarShooterPower += 0.05;
                }
            }
            if (gamepad1.dpadDownWasPressed()) {
                if (pollenShooterPower > 0) {
                    pollenShooterPower -= 0.05;
                }
            }
            if (gamepad1.dpadUpWasPressed()) {
                if (pollenShooterPower < 1) {
                    pollenShooterPower += 0.05;
                }
            }

            if (gamepad1.leftBumperWasPressed()) {
                nectarShooterOn = !nectarShooterOn;
                pollenShooterOn = !pollenShooterOn;
            }

            if (gamepad1.rightBumperWasPressed()) {
                intakeOn = !intakeOn;
            }

            if (gamepad1.aWasPressed()) {
                indexerOn = !indexerOn;
                robot.setIndexerOpen(indexerOn);
            }

            robot.setPollenPower((pollenShooterOn ? 1 : 0) * pollenShooterPower);
            robot.setNectarPower((nectarShooterOn ? 1 : 0) * nectarShooterPower);
            double intake = 0;
            if (gamepad2.a) {
                intake = -1;
            } else {
                intake = (intakeOn ? 1 : 0);
            }
            robot.setIntakePower(intake);

            telemetry.addData("forwardSpeed", forwardPower);
            telemetry.addData("strafeSpeed", strafePower);
            telemetry.addData("heading", imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS));
            telemetry.addData("pinpointHeading",odo.getHeading(AngleUnit.RADIANS));
            telemetry.addData("frontLeftVelocity",robot.frontLeft.getVelocity());
            telemetry.addData("frontRightVelocity",robot.frontRight.getVelocity());
            telemetry.addData("backLeftVelocity",robot.backLeft.getVelocity());
            telemetry.addData("backRightVelocity",robot.backRight.getVelocity());
            telemetry.addData("pollenShooterPower", pollenShooterPower);
            telemetry.addData("pollenShooterVelocity", robot.getPollenVelocity());
            telemetry.addData("nectarShooterPower", nectarShooterPower);
            telemetry.addData("nectarShooterVelocity", robot.getNectarVelocity());
            telemetry.addData("intakeVelocity", robot.getIntakeVelocity());
            telemetry.update();
        }
    }
}
