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

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

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

public class BiobuzzDriveTestCode extends LinearOpMode {
    DcMotorEx frontLeft;
    DcMotorEx frontRight;
    DcMotorEx backLeft;
    DcMotorEx backRight;
    BNO055IMU imu;

    @Override
    public void runOpMode() {

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        frontLeft = hardwareMap.get(DcMotorEx.class,"frontLeft");
        frontLeft.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        frontRight = hardwareMap.get(DcMotorEx.class, "frontRight");
        frontRight.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        backLeft = hardwareMap.get(DcMotorEx.class, "backLeft");
        backLeft.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        backRight = hardwareMap.get(DcMotorEx.class, "backRight");
        backRight.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        imu = hardwareMap.get(BNO055IMU.class, "imu");

        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        imu.initialize(parameters);

        double forwardPower = 0;
        double strafePower = 0;
        double turnPower = 0;
        double forwardPowerTemp = 0;
        double strafePowerTemp = 0;

        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        double backLeftPower = 0.0;
        double backRightPower = 0.0;
        double frontLeftPower = 0.0;
        double frontRightPower = 0.0;

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            telemetry.addData("Status", "Running");

            strafePowerTemp = gamepad1.left_stick_x;
            forwardPowerTemp = gamepad1.left_stick_y;
            turnPower = gamepad1.right_stick_x;

            Orientation angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.RADIANS);
            double currentHeading = angles.firstAngle;

            strafePower = strafePowerTemp * Math.cos(currentHeading)
                    - forwardPowerTemp * Math.sin(currentHeading);
            forwardPower = forwardPowerTemp * Math.cos(currentHeading)
                    + strafePowerTemp * Math.sin(currentHeading);

            backLeftPower = turnPower + forwardPower - strafePower;
            backRightPower = 0.0;
            frontLeftPower = 0.0;
            frontRightPower = 0.0;

            if(gamepad2.a){
                backLeftPower = 0;
            }


            backLeft.setPower(backLeftPower);
            backRight.setPower(turnPower - forwardPower - strafePower);
            frontRight.setPower(turnPower - forwardPower + strafePower);
            frontLeft.setPower(turnPower + forwardPower + strafePower);


            telemetry.addData("forwardSpeed", forwardPower);
            telemetry.addData("strafeSpeed", strafePower);
            telemetry.addData("frontLeftSpeed",frontLeft.getVelocity());
            telemetry.addData("frontRightSpeed",frontRight.getVelocity());
            telemetry.addData("backLeftSpeed",backLeft.getVelocity());
            telemetry.addData("backRightSpeed",backRight.getVelocity());
            telemetry.update();


        }
    }


}
