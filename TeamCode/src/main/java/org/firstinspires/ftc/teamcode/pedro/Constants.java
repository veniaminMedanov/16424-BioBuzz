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
        c.xPodOffset.set(-2.172697022205263);
        c.yPodOffset.set(5.21120266651544);
        c.xPodDirection.set(GoBildaPinpointDriver.EncoderDirection.REVERSED);
        c.yPodDirection.set(GoBildaPinpointDriver.EncoderDirection.REVERSED);
        c.globalDistanceUnit.set(DistanceUnit.INCH);
        c.offsetUnits.set(DistanceUnit.INCH);
    });

    public static MecanumConfig drivetrainConfig = new MecanumConfig(c -> {
        c.frontLeftName.set("backRight");
        c.frontRightName.set("backLeft");
        c.backLeftName.set("frontRight");
        c.backRightName.set("frontLeft");
        c.frontLeftDirection.set(DcMotorSimple.Direction.FORWARD);
        c.frontRightDirection.set(DcMotorSimple.Direction.REVERSE);
        c.backLeftDirection.set(DcMotorSimple.Direction.FORWARD);
        c.backRightDirection.set(DcMotorSimple.Direction.REVERSE);
    });

    public static ForesightConfig foresightConfig = new ForesightConfig(
            c -> {
                Controller primaryTranslationalForward = Controller.proportional(0.37991146863497643);
                Controller secondaryTranslationalForward = Controller.proportional(0.14036713623922045);
                Controller primaryTranslationalLateral = Controller.proportional(0.6229162315574036);
                Controller secondaryTranslationalLateral = Controller.proportional(0.23015090293220494);

                c.forwardTranslational.set(Controller.piecewise(secondaryTranslationalForward).put(2.5, primaryTranslationalForward));
                c.strafeTranslational.set(Controller.piecewise(secondaryTranslationalLateral).put(2.5, primaryTranslationalLateral));

                c.coast.set(Controller.proportionalFeedforward(0.01617131018809822));
                c.brake.set(Controller.proportionalFeedforward(0.013745613659883485));

                c.headingFeedback.set(Controller.proportional(5.841388429706148));
                c.headingBrakeCoefficients.set(Vector2D.cartesian(0.05620050136054935, 0.009732771333871107));

                c.linearBrakeCoefficients.set(Matrix.diag(0.07336691916713935, 0.07285828613171146));
                c.quadraticBrakeCoefficients.set(Matrix.diag(0.0016261432293880492, 0.0014458965096036578));

                c.maxAchievableForwardVelocity.set(59.17070468155606);
                c.maxAchievableStrafeVelocity.set(45.54446617325245);
                c.naturalForwardDeceleration.set(47.00684075199528);
                c.naturalStrafeDeceleration.set(72.78551359423966);
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