package org.firstinspires.ftc.teamcode;

import com.pedropathing.follower.Follower;
import com.pedropathing.math.Pose;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Robot {
    DriveTrain driveTrain;
    Shooter shooter;
    Transfer transfer;
    Follower follower;
    public static Pose teleOpStartPose;


    public Robot(HardwareMap hwMap){
//        follower = new Follower();
    }

    public static void setTeleOpStartPose(Pose startPose) {
        teleOpStartPose = startPose;
    }

}
