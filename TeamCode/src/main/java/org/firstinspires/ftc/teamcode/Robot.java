package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.IMU;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

public class Robot {
    DcMotorEx frontLeft;
    DcMotorEx frontRight;
    DcMotorEx backLeft;
    DcMotorEx backRight;
    DcMotorEx pollenShooter;
    DcMotorEx nectarShooter;
    DcMotorEx intake;
    Servo leftIndexer;
    Servo rightIndexer;

    void setPollenPower(double power) {
        pollenShooter.setPower(power);
    }
    double getPollenPower() {
        return pollenShooter.getPower();
    }
    double getPollenVelocity() {
        return pollenShooter.getVelocity();
    }
    void setNectarPower(double power) {
        nectarShooter.setPower(power);
    }
    double getNectarPower() {
        return pollenShooter.getPower();
    }
    double getNectarVelocity() {
        return pollenShooter.getVelocity();
    }
    void setIntakePower(double power) {
        intake.setPower(power);
    }
    double getIntakePower() {
        return pollenShooter.getPower();
    }
    double getIntakeVelocity() {
        return pollenShooter.getVelocity();
    }
    void setIndexerOpen(boolean open) {
        leftIndexer.setPosition(open ? 0.1 : 0.35);
        rightIndexer.setPosition(open ? 0.1 : 0.35);
    }
    double getIndexerPos() {
        return leftIndexer.getPosition();
    }
//    double getHeadingRadians() {
//        return imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);
//    }

    Robot(HardwareMap hardwareMap) {
        //Drivetrain
        frontLeft = hardwareMap.get(DcMotorEx.class, "frontLeft");
        frontRight = hardwareMap.get(DcMotorEx.class, "frontRight");
        backLeft = hardwareMap.get(DcMotorEx.class, "backLeft");
        backRight = hardwareMap.get(DcMotorEx.class, "backRight");

        frontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        frontLeft.setDirection(DcMotorSimple.Direction.FORWARD);
        frontRight.setDirection(DcMotorSimple.Direction.REVERSE);
        backLeft.setDirection(DcMotorSimple.Direction.FORWARD);
        backRight.setDirection(DcMotorSimple.Direction.REVERSE);

        //Shooters
        pollenShooter = hardwareMap.get(DcMotorEx.class, "pollenShooter");
        nectarShooter = hardwareMap.get(DcMotorEx.class, "nectarShooter");

        pollenShooter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        nectarShooter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        pollenShooter.setDirection(DcMotorSimple.Direction.FORWARD);
        nectarShooter.setDirection(DcMotorSimple.Direction.REVERSE);

        //Intake
        intake = hardwareMap.get(DcMotorEx.class, "intake");

        intake.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        intake.setDirection(DcMotorSimple.Direction.FORWARD);

        //Indexers
        leftIndexer = hardwareMap.get(Servo.class, "indexerLeft");
        rightIndexer = hardwareMap.get(Servo.class, "indexerRight");

        leftIndexer.setDirection(Servo.Direction.FORWARD);
        rightIndexer.setDirection(Servo.Direction.REVERSE);

    }
}
