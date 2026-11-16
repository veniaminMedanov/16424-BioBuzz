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
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

/**
 * This file contains a minimal example of a Linear "OpMode". An OpMode is a 'program' that runs
 * in either the autonomous or the TeleOp period of an FTC match. The names of OpModes appear on
 * the menu of the FTC Driver Station. When a selection is made from the menu, the corresponding
 * OpMode class is instantiated on the Robot Controller and executed.
 *
 * Remove the @Disabled annotation on the next line or two (if present) to add this OpMode to the
 * Driver Station OpMode list, or add a @Disabled annotation to prevent this OpMode from being
 * added to the Driver Station.
 * */
@TeleOp

public class motor extends LinearOpMode {
    Servo indexerLeft;
    double indexerLeftPos = 0;
    double indexerRightPos = 0;
    Servo indexerRight;
    @Override
    public void runOpMode() {
        indexerLeft = hardwareMap.get(Servo.class, "indexerLeft");
        indexerLeft.setDirection(Servo.Direction.FORWARD);
        indexerRight = hardwareMap.get(Servo.class, "indexerRight");
        indexerRight.setDirection(Servo.Direction.REVERSE);
        telemetry.addData("Status", "Initialized");
        telemetry.update();
        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        indexerLeft.setPosition(indexerLeftPos);
        indexerRight.setPosition(indexerRightPos);

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            if (gamepad1.dpadUpWasPressed()) {
                    indexerRightPos += 0.05;
                    indexerRight.setPosition(indexerRightPos);
            } else if (gamepad1.dpadDownWasPressed()) {
                    indexerRightPos -= 0.05;
                    indexerRight.setPosition(indexerRightPos);
            }
            if (gamepad1.dpadRightWasPressed()) {
                indexerLeftPos += 0.05;
                indexerLeft.setPosition(indexerLeftPos);
            } else if (gamepad1.dpadLeftWasPressed()) {
                indexerLeftPos -= 0.05;
                indexerLeft.setPosition(indexerLeftPos);
            }
            telemetry.addData("indexer left pos", indexerLeft.getPosition());
            telemetry.addData("indexerright pos", indexerRight.getPosition());
            telemetry.update();
      }
    }
}
