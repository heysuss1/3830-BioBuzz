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
                Controller primaryTranslationalForward = Controller.proportional(0.21490875554106256);
                Controller secondaryTranslationalForward = Controller.proportional(0.07940304270471403);
                Controller primaryTranslationalLateral = Controller.proportional(0.3299161406934299);
                Controller secondaryTranslationalLateral = Controller.proportional(0.12189519846458521);

                c.forwardTranslational.set(Controller.piecewise(secondaryTranslationalForward).put(2.5, primaryTranslationalForward));
                c.strafeTranslational.set(Controller.piecewise(secondaryTranslationalLateral).put(2.5, primaryTranslationalLateral));

                c.coast.set(Controller.proportionalFeedforward(0.010167651048199356));
                c.brake.set(Controller.proportionalFeedforward(0.008642503390969452));

                c.headingFeedback.set(Controller.proportional(3.440129596171333));
                c.headingBrakeCoefficients.set(Vector2D.cartesian(0.06354720115589134, 0.005107500155832511));

                c.linearBrakeCoefficients.set(Matrix.diag(0.10326143016352414, 0.0527241867845811));
                c.quadraticBrakeCoefficients.set(Matrix.diag(0.0012683494652124402, 0.001967949664449313));

                c.maxAchievableForwardVelocity.set(89.87965238188215);
                c.maxAchievableStrafeVelocity.set(72.13300043950584);
                c.naturalForwardDeceleration.set(31.777492592991287);
                c.naturalStrafeDeceleration.set(54.0600926002226);
            }
    );
    public static PinpointConfig localizerConfig = new PinpointConfig(c -> {
        c.name.set("pinpoint");
        c.podType.set(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        c.xPodOffset.set(-2.85959589199757);
        c.yPodOffset.set(-6.114279078686331);
        c.xPodDirection.set(GoBildaPinpointDriver.EncoderDirection.FORWARD);
        c.yPodDirection.set(GoBildaPinpointDriver.EncoderDirection.REVERSED);
        c.globalDistanceUnit.set(DistanceUnit.INCH);
        c.offsetUnits.set(DistanceUnit.INCH);
    });


}