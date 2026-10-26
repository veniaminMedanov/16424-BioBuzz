package org.firstinspires.ftc.teamcode.pedro;

import com.pedropathing.algorithm.Foresight;
import com.pedropathing.algorithm.ForesightConfig;
import com.pedropathing.controllers.Controller;
import com.pedropathing.follower.Follower;
import com.pedropathing.math.Matrix;
import com.pedropathing.math.Vector2D;
import com.pedropathing.revhub.drivetrains.Mecanum;
import com.pedropathing.revhub.drivetrains.MecanumConfig;
import com.pedropathing.revhub.localizers.PinpointConfig;
import com.pedropathing.revhub.localizers.PinpointLocalizer;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public class Constants {

    public static PinpointConfig localizerConfig = new PinpointConfig(c -> {
        c.name.set("pinpoint");
        c.podType.set(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        c.xPodOffset.set(-1.7404221362016332);
        c.yPodOffset.set(-3.427502789835292);
        c.xPodDirection.set(GoBildaPinpointDriver.EncoderDirection.REVERSED);
        c.yPodDirection.set(GoBildaPinpointDriver.EncoderDirection.FORWARD);
        c.globalDistanceUnit.set(DistanceUnit.INCH);
        c.offsetUnits.set(DistanceUnit.INCH);
    });

    public static MecanumConfig drivetrainConfig = new MecanumConfig(c -> {
        c.frontLeftName.set("frontRight");
        c.frontRightName.set("frontLeft");
        c.backLeftName.set("backRight");
        c.backRightName.set("backLeft");
        c.frontLeftDirection.set(DcMotorSimple.Direction.FORWARD);
        c.frontRightDirection.set(DcMotorSimple.Direction.REVERSE);
        c.backLeftDirection.set(DcMotorSimple.Direction.FORWARD);
        c.backRightDirection.set(DcMotorSimple.Direction.REVERSE);
    });

    public static ForesightConfig foresightConfig = new ForesightConfig(
            c -> {
                Controller primaryTranslationalForward = Controller.proportional(0.2795308034544289);
                Controller secondaryTranslationalForward = Controller.proportional(0.10327916267578093);
                Controller primaryTranslationalLateral = Controller.proportional(0.35539532638330346);
                Controller secondaryTranslationalLateral = Controller.proportional(0.13130907676061312);

                c.forwardTranslational.set(Controller.piecewise(secondaryTranslationalForward).put(2.5, primaryTranslationalForward));
                c.strafeTranslational.set(Controller.piecewise(secondaryTranslationalLateral).put(2.5, primaryTranslationalLateral));

                c.coast.set(Controller.proportionalFeedforward(0.018778702573088158));
                c.brake.set(Controller.proportionalFeedforward(0.015961897187124933));

                c.headingFeedback.set(Controller.proportional(4.9722239847380845));
                c.headingBrakeCoefficients.set(Vector2D.cartesian(0.0639061305071692, 0.008797789130891244));

                c.linearBrakeCoefficients.set(Matrix.diag(-0.01408450788288325, 0.0662609344762654));
                c.quadraticBrakeCoefficients.set(Matrix.diag(0.003756074428562613, 0.002741275657625901));

                c.maxAchievableForwardVelocity.set(56.60146300080997);
                c.maxAchievableStrafeVelocity.set(44.90733216609554);
                c.naturalForwardDeceleration.set(43.6900669324922);
                c.naturalStrafeDeceleration.set(58.73383779635155);
            }
    );

    public static Follower create(HardwareMap h) {
        return new Follower(
                new PinpointLocalizer(h, localizerConfig),
                new Mecanum(h, drivetrainConfig),
                new Foresight(foresightConfig)
        );
    }
}