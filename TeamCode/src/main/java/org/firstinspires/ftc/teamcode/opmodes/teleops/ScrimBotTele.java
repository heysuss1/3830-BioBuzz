package org.firstinspires.ftc.teamcode.opmodes.teleops;

import com.acmerobotics.dashboard.config.Config;
import com.pedropathing.drivetrain.Drivetrain;
import com.pedropathing.follower.Follower;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.opmodes.testers.TransferTest;
import org.firstinspires.ftc.teamcode.pedro.Constants;
import org.firstinspires.ftc.teamcode.subsystems.Flywheel;
import org.firstinspires.ftc.teamcode.subsystems.Transfer;



@Config
@TeleOp (name = "Scrimmage TeleOp")
public class ScrimBotTele extends LinearOpMode {

    public static double power = 0.7;

    public void runOpMode(){
        Follower follower = Constants.create(hardwareMap);
        Transfer transfer = new Transfer(hardwareMap);
        Flywheel flywheel = new Flywheel(hardwareMap);

        waitForStart();
        while (opModeIsActive()){

            follower.manual(
                    -gamepad1.left_stick_y,
                    -gamepad1.left_stick_x,
                    -gamepad1.right_stick_x
            );
            flywheel.setPower(power);
            if (gamepad1.rightBumperWasPressed()){
                transfer.setIntakeMode();
            } else if (gamepad1.circleWasPressed()){
                transfer.setEmptyMode();
            } else if (gamepad1.xWasPressed()){
                transfer.setHoldMode();
            } else if (gamepad1.rightTriggerWasPressed()){
                transfer.setUptakeMode();
            }

            follower.update();
            telemetry.update();
            transfer.transferUpdate();

        }
    }

}
