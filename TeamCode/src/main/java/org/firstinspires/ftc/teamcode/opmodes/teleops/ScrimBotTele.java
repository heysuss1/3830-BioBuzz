package org.firstinspires.ftc.teamcode.opmodes.teleops;

import com.pedropathing.drivetrain.Drivetrain;
import com.pedropathing.follower.Follower;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.opmodes.testers.TransferTest;
import org.firstinspires.ftc.teamcode.pedro.Constants;
import org.firstinspires.ftc.teamcode.subsystems.Transfer;


@TeleOp (name = "Scrimmage TeleOp")
public class ScrimBotTele extends LinearOpMode {

    public void runOpMode(){
        Follower follower = Constants.create(hardwareMap);
        Transfer transfer = new Transfer(hardwareMap);

        waitForStart();
        while (opModeIsActive()){

            follower.manual(
                    -gamepad1.left_stick_y,
                    gamepad1.left_stick_x,
                    gamepad1.right_stick_x
            );

            if (gamepad1.rightBumperWasPressed()){
                transfer.setIntakeMode();
            } else if (gamepad1.circleWasPressed()){
                transfer.setEmptyMode();
            } else if (gamepad1.xWasPressed()){
                transfer.setHoldMode();
            }


            follower.update();
            telemetry.update();
            transfer.transferUpdate();

        }
    }

}
