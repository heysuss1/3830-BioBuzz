package org.firstinspires.ftc.teamcode.opmodes.teleops;


import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.math.AimCalculator;

import java.util.Timer; //idk if this is the right one


@Config
@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name = "TeleOp")
public class TeleOp extends LinearOpMode {


    Gamepad currentGamepad1 = new Gamepad();
    Gamepad previousGamepad1 = new Gamepad();
    private Robot robot;
    Timer loopTimer;

    public void runOpMode() {





        while (opModeInInit()) {
            //idk
            break;
        }



        while (opModeIsActive()) {




            robot.turret.calcTurretTarget(
                    AimCalculator.calcRawTargetYaw(robot.follower.pose()),
                    robot.follower.pose().heading()
            ); //calcTurretTarget sets it too btw
            robot.hood.setPitchTarget();

            robot.follower.update();
            //telemetry stuff idk

            telemetry.update();
        }
    }
}
