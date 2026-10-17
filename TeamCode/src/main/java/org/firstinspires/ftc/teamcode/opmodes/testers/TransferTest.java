package org.firstinspires.ftc.teamcode.opmodes.testers;


import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@TeleOp(name = "Tester for transfer")
public class TransferTest extends LinearOpMode {
    static double intakePower = 0.75;
    static double transferPower = 1;
    private DcMotor intake;
    private DcMotor transfer;


    public void runOpMode(){


        intake = hardwareMap.get(DcMotor.class, "intake");
        intake.setDirection(DcMotorSimple.Direction.REVERSE);
        intake.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        intake.setPower(0);

        transfer = hardwareMap.get(DcMotor.class, "transfer");
        transfer.setDirection(DcMotorSimple.Direction.FORWARD);
        transfer.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        transfer.setPower(0);
        waitForStart();
        while (opModeIsActive()){
            if (gamepad1.a){
                intake.setPower(intakePower);
                transfer.setPower(transferPower);
            }
//            if (gamepad1.bWasPressed()){
//                intakePower *= -1;
//            }
////            if (gamepad1.bWasPressed()){
////                intakePower *= -1;
////            }
////            if (gamepad1.yWasPressed()){
////                transferPower *= -1;
////            }
            if (gamepad1.dpadUpWasPressed()){
                intakePower += 0.1;
            }
            if (gamepad1.dpadDownWasPressed()){
                intakePower -= 0.1;
            }
            telemetry.addData("power", intakePower);
            telemetry.update();
        }

    }
}
