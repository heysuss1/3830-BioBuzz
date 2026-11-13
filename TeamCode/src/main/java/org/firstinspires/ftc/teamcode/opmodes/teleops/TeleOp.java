package org.firstinspires.ftc.teamcode.opmodes.teleops;


import com.acmerobotics.dashboard.config.Config;
import com.pedropathing.math.Pose;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.FieldConstants;
import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.commands.Fire;
import org.firstinspires.ftc.teamcode.math.AimCalculator;
import org.firstinspires.ftc.teamcode.math.ShotCalculator;
import org.firstinspires.ftc.teamcode.math.Vector;

import java.lang.reflect.Field;
import java.util.Timer; //idk if this is the right one



@Config
@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name = "Main Tele")
public class TeleOp extends LinearOpMode {
    private Robot robot;
    ElapsedTime loopTimer = new ElapsedTime();
    //Placeholder before runner is created:
    Fire fire;
    public static double kP, kI, kD, kF;
    public static double hive_x = 57.6;
    public static double hive_y = 51.8;
    public void runOpMode() {
        robot = new Robot(hardwareMap, telemetry);
        fire = new Fire(robot);
        kP = 0.5;

        Robot.Team team = Robot.Team.RED;

        robot.implementManualBulkReads();
        if (Robot.autoHasRan){
            robot.startFromTeleop();
        } else {
            robot.follower.setPose(new Pose(52.8, 8.2, Math.toRadians(90))); //blue starting pose
        }
//        robot.follower.setPose(new Pose(53, 8.2, Math.toRadians(90)));
        waitForStart();
        while (opModeIsActive()) {
            FieldConstants.BOTTOM_RED_HIVE_POS = new Pose(hive_x, hive_y);
            robot.turret.setPIDFCoefficients(kP, kI, kD);
            double loopMiliseconds = loopTimer.milliseconds();
            loopTimer.reset();
            robot.turret.calcTurretTarget(
                    robot.aimCalc.calcRawTargetYaw(robot.follower.pose()),
                    robot.follower.pose().heading()
            );

            robot.setRobotCentricDriving(gamepad1); //robot centric for now, can be field


            if (gamepad2.dpadUpWasPressed()){
                robot.turret.lockedTurretTarget += Math.toRadians(5);
            }
            if (gamepad2.dpadUpWasPressed()){
                robot.turret.lockedTurretTarget -= Math.toRadians(5);
            }
            if (gamepad2.rightBumperWasPressed()){
                robot.turret.lockTurret = !robot.turret.lockTurret;
            }
            if (gamepad1.rightBumperWasPressed()){
                robot.transfer.setIntakeMode();
            } else if (gamepad1.circleWasPressed()){
                robot.transfer.setEmptyMode();
            } else if (gamepad1.xWasPressed()){
                robot.transfer.setHoldMode();
            }

            if (gamepad1.rightTriggerWasPressed()){
                fire.start();
            }

            //WHen back button is pressed, swap the team ur on
            if (gamepad1.backWasPressed()){
                team = (team == Robot.Team.BLUE) ? Robot.Team.RED : Robot.Team.BLUE;
            }

            //When dpad up is pressed, turns always have shooter on/off
            if (gamepad1.dpadUpWasPressed()){
                robot.flywheel.setAlwaysShoot(!robot.flywheel.getAlwaysShoot());
            }

            //When dpad down is pressed, reset the target pos of the right goal
            //to where the robot is currently sitting
//            if (gamepad1.dpadDownWasPressed()) {
//                robot.aimCalc.updateTargetPosition(
//                        robot.follower.pose().x(),
//                        robot.follower.pose().y()
//                );
//            }


            //ShotCalc update adjusts the RPM necessary to get into the goal
            robot.shotCalc.update(robot.follower.pose(), robot.getTeam(),
                    Vector.pedroVelocityToVector(robot.follower.velocity()));
//            robot.flywheel.setTargetRPM(robot.shotCalc.getRPM());
            robot.flywheel.setTargetRPM(robot.shotCalc.getRPM());
//            robot.flywheel.setTargetRPM(0.0);

            robot.hood.setPitchTarget(robot.shotCalc.getPitch());
            fire.update();
            robot.update();
            robot.setTeam(team);
            telemetry.addData("Time", "%.2f", loopMiliseconds);
            telemetry.addData("pose", robot.follower.pose());
            telemetry.addData("distance to goal", robot.shotCalc.getDistance());
            telemetry.addData("target rpm", robot.shotCalc.getRPM());
            telemetry.addData("current rpm of the robot", robot.flywheel.getRPM());
            if (robot.turret.getTurretTarget() != null){
                telemetry.addData("turret target", robot.turret.getTurretTarget() * (180.0/Math.PI));
            }
            telemetry.addData("turret pos", robot.turret.getTurretRadians() * (180/Math.PI));
            telemetry.addData("pitch", robot.shotCalc.getPitch());
            telemetry.update();
            robot.clearCache();
        }
    }
}
