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

import kotlin.Unit;

public class Constants {
    public static Follower create(HardwareMap h) {
         return new Follower(
                 new PinpointLocalizer(h, localizerConfig),
                 new Mecanum(h, driveConfig),
                 new Foresight(foresightConfig)
         );
    }
    public static MecanumConfig driveConfig = new MecanumConfig(
            c -> {
                c.frontLeftName.set("lf");
                c.backLeftName.set("lb");
                c.frontRightName.set("rf");
                c.backRightName.set("rb");

                c.frontLeftDirection.set(DcMotorSimple.Direction.REVERSE);
                c.backLeftDirection.set(DcMotorSimple.Direction.REVERSE);
                c.frontRightDirection.set(DcMotorSimple.Direction.FORWARD);
                c.backRightDirection.set(DcMotorSimple.Direction.FORWARD);
            }
    );

    public static ForesightConfig foresightConfig = new ForesightConfig(
            c -> {
                Controller primaryTranslationalForward = Controller.proportional(0.2564263021530841);
                Controller secondaryTranslationalForward = Controller.proportional(0.09474266680857894);
                Controller primaryTranslationalLateral = Controller.proportional(0.6111966437925562);
                Controller secondaryTranslationalLateral = Controller.proportional(0.22582082840624634);

                c.forwardTranslational.set(Controller.piecewise(secondaryTranslationalForward).put(2.5, primaryTranslationalForward));
                c.strafeTranslational.set(Controller.piecewise(secondaryTranslationalLateral).put(2.5, primaryTranslationalLateral));

                c.coast.set(Controller.proportionalFeedforward(0.013279901264223975));
                c.brake.set(Controller.proportionalFeedforward(0.011287916074590378));

                c.headingFeedback.set(Controller.proportional(5.182579308272094));
                c.headingBrakeCoefficients.set(Vector2D.cartesian(0.06366182803510674, 0.005588402256222653));

                c.linearBrakeCoefficients.set(Matrix.diag(0.046893819815319954, 0.07085405556217185));
                c.quadraticBrakeCoefficients.set(Matrix.diag(0.0032903731099422247, 0.0012741743203535049));

                c.maxAchievableForwardVelocity.set(78.64759601029422);
                c.maxAchievableStrafeVelocity.set(62.092696592464755);
                c.naturalForwardDeceleration.set(39.5244221215076);
                c.naturalStrafeDeceleration.set(72.09923596480915);
            }
    );
    public static PinpointConfig localizerConfig = new PinpointConfig(c -> {
        c.name.set("pinpoint");
        c.podType.set(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        c.xPodOffset.set(-3.6678686667615037);
        c.yPodOffset.set(5.818153441421629);
        c.xPodDirection.set(GoBildaPinpointDriver.EncoderDirection.FORWARD);
        c.yPodDirection.set(GoBildaPinpointDriver.EncoderDirection.REVERSED);
        c.globalDistanceUnit.set(DistanceUnit.INCH);
        c.offsetUnits.set(DistanceUnit.INCH);
    });


}