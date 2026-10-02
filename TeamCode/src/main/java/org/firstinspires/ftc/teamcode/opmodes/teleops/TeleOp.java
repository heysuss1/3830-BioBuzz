package org.firstinspires.ftc.teamcode.opmodes.teleops;


import com.pedropathing.math.Vector;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.FieldConstants;
import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.commands.Fire;
import org.firstinspires.ftc.teamcode.math.AimCalculator;
import org.firstinspires.ftc.teamcode.math.ShotCalculator;

import java.util.Timer; //idk if this is the right one


@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name = "Main Tele")
public class TeleOp extends LinearOpMode {
    private Robot robot;
    Timer loopTimer;
    //Place holder before runner is created:
    Fire fire;
    public void runOpMode() {
        robot = new Robot(hardwareMap, telemetry);
        fire = new Fire(robot);

        Robot.Team team = Robot.Team.RED;
        robot.startFromTeleop();

        waitForStart();
        while (opModeIsActive()) {

            robot.turret.calcTurretTarget(
                    AimCalculator.calcRawTargetYaw(robot.follower.pose()),
                    robot.follower.pose().heading()
            );

            robot.setRobotCentricDriving(gamepad1); //robot centric for now, can be field

            if (gamepad1.rightBumperWasPressed()){
                robot.transfer.setIntakeMode();
            } else if (gamepad1.circleWasPressed()){
                robot.transfer.setEmptyMode();
            } else if (gamepad1.xWasPressed()){
                robot.transfer.setHoldMode();
            }

            if (gamepad1.right_trigger_pressed){
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


            //ShotCalc update adjusts the RPM necessary to get into the goal
            robot.shotCalc.update(robot.follower.pose(), robot.getTeam(), robot.follower.velocity());
            robot.flywheel.setTargetRPM(robot.shotCalc.getRPM());
            fire.update();
            robot.update();
            robot.setTeam(team);
            telemetry.update();
        }
    }
}
