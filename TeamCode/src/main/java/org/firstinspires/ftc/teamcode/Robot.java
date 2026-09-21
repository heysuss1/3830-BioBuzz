package org.firstinspires.ftc.teamcode;

import com.pedropathing.follower.Follower;
import com.pedropathing.math.Pose;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.subsystems.Drive;
import org.firstinspires.ftc.teamcode.subsystems.Flywheel;
import org.firstinspires.ftc.teamcode.subsystems.Hood;
import org.firstinspires.ftc.teamcode.subsystems.Transfer;
import org.firstinspires.ftc.teamcode.subsystems.Turret;

public class Robot {
    Drive driveTrain;
    Hood hood;
    Flywheel flywheel;
    Turret turret;
    Transfer transfer;

    Follower follower;
    public static Pose teleOpStartPose;

    public Robot(HardwareMap hwMap){
        driveTrain = new Drive(hwMap);
        hood = new Hood(hwMap);
        flywheel = new Flywheel(hwMap);
        turret = new Turret(hwMap);
        transfer = new Transfer(hwMap);
    }
}
