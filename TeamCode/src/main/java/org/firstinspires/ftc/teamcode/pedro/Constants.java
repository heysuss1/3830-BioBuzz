package org.firstinspires.ftc.teamcode.pedro;

import com.pedropathing.algorithm.Foresight;
import com.pedropathing.algorithm.ForesightConfig;
import com.pedropathing.follower.Follower;
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
                 null
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
            foresightConfig -> {

            }
    );
    public static PinpointConfig localizerConfig = new PinpointConfig(
            pinpointConfig -> {
                pinpointConfig.name.set("pinpoint");
                pinpointConfig.xPodDirection.set(GoBildaPinpointDriver.EncoderDirection.FORWARD);
                pinpointConfig.yPodDirection.set(GoBildaPinpointDriver.EncoderDirection.REVERSED);
                pinpointConfig.offsetUnits.set(DistanceUnit.INCH);
                pinpointConfig.podType.set(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
                pinpointConfig.yPodOffset.set(2.25);
//                pinpointConfig.xPodOffset.set(-151.162);
                pinpointConfig.xPodOffset.set(-5.5);


            }

    );


}