package org.firstinspires.ftc.teamcode.subsystems;

import com.pedropathing.follower.Follower;
import com.pedropathing.math.Pose;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.Shooter;

public class Robot {
    Drive driveTrain;
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
