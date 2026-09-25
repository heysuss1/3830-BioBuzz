package org.firstinspires.ftc.teamcode;

import com.pedropathing.drivetrain.DrivePowers;
import com.pedropathing.follower.Follower;
import com.pedropathing.follower.ManualDrive;
import com.pedropathing.math.Pose;
import com.qualcomm.robotcore.hardware.Gamepad;
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
    public void setRobotCentricDriving(Gamepad  gamepad1){
        follower.manual(
        -gamepad1.left_stick_y,
        gamepad1.left_stick_x,
        gamepad1.right_stick_x
        );

    }
    public void setFieldCentricDriving(Gamepad gamepad1){
        DrivePowers powers = ManualDrive.fieldCentric(
                -gamepad1.left_stick_y,
                gamepad1.left_stick_x,
                gamepad1.right_stick_x,
                follower.pose().heading()
        );
        follower.manual(powers);

    }
}
