package org.firstinspires.ftc.teamcode;

import com.pedropathing.drivetrain.DrivePowers;
import com.pedropathing.follower.Follower;
import com.pedropathing.follower.ManualDrive;
import com.pedropathing.math.Pose;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.math.ShotCalculator;
import org.firstinspires.ftc.teamcode.pedro.Constants;
import org.firstinspires.ftc.teamcode.subsystems.Drive;
import org.firstinspires.ftc.teamcode.subsystems.Flywheel;
import org.firstinspires.ftc.teamcode.subsystems.Hood;
import org.firstinspires.ftc.teamcode.subsystems.Transfer;
import org.firstinspires.ftc.teamcode.subsystems.Turret;

public class Robot {
    public final Drive driveTrain;
    public final Hood hood;
    public final Flywheel flywheel;
    public final Turret turret;
    public final Transfer transfer;
    public final Follower follower;
    public enum Team {RED, BLUE}
    Team team = Team.RED;
    public ShotCalculator shotCalc; //will be made final once cade shenanigans are figured and sorted out

    public static Pose teleOpStartPose;

    public Robot(HardwareMap hwMap, Telemetry telemetry){
        follower = Constants.create(hwMap);
        driveTrain = new Drive(hwMap);
        hood = new Hood(hwMap, telemetry);
        flywheel = new Flywheel(hwMap);
        turret = new Turret(hwMap, telemetry);
        transfer = new Transfer(hwMap);
//        shotCalc = new ShotCalculator();
    }

    public Team getTeam(){
        return team;
    }
    public void setTeam(Team team){
        this.team = team;
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
    public boolean allSystemsReady(){
        return (flywheel.isAtTarget() && hood.isAtTarget() && turret.isAtTarget());
    }
    public static void setTeleOpStartPose(Pose pose){
        teleOpStartPose = pose;
    }

    public void startFromTeleop(){
        follower.setPose(teleOpStartPose);
    }

    public void update(){
        follower.update();
        turret.update();
        flywheel.updateFly();
        transfer.transferUpdate();
        hood.update();
    }
}
