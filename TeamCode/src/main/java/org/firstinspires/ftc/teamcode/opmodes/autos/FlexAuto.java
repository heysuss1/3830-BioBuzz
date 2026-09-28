package org.firstinspires.ftc.teamcode.opmodes.autos;

import com.pedropathing.api.PoseFactory;
import com.pedropathing.math.Pose;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Robot;

public class FlexAuto extends LinearOpMode {


    private final PoseFactory poseFactory = PoseFactory.degrees();
    private final Pose start = poseFactory.of(55.9664, 5.9128, 90);
    private final Pose path1Start = poseFactory.of(55.9664, 5.9128, 180);
    private final Pose path1 = poseFactory.of(6.0478, 6.2968, 180);
    private final Pose point2Start = poseFactory.of(6.0478, 6.2968, 45);
    private final Pose point2 = poseFactory.of(48.2171, 115.7354, 90);
    private final Pose point2Control1 = poseFactory.of(7.3223, 93.4389, 0);
    private final Pose point3 = poseFactory.of(47.4077, 130.9586, 90);
    private final Pose point4 = poseFactory.of(3.9359, 117.8457, 270);
    Robot robot;
    public void runOpMode(){
        robot = new Robot(hardwareMap, telemetry);
        waitForStart();
        while (opModeIsActive()){

        }
    }
}
