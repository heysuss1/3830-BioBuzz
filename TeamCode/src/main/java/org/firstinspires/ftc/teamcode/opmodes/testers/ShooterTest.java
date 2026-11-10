package org.firstinspires.ftc.teamcode.opmodes.testers;


import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.teamcode.Robot;


import org.firstinspires.ftc.teamcode.controllers.PIDFF;
import org.firstinspires.ftc.teamcode.subsystems.Flywheel;
import org.firstinspires.ftc.teamcode.subsystems.Transfer;


@Config
@TeleOp(name = "Shooter Test")
public class ShooterTest extends LinearOpMode {


    public static double kP, kI, kD, kS, kV, kA, integralMax;
    public static boolean usePower = false;
    public static double targetRPM = 0;
    public static boolean stopShooter = false;
    public static double power;

    public Robot robot;
    @Override
    public void runOpMode() throws InterruptedException {
        robot = new Robot(hardwareMap, telemetry);
        Flywheel flywheel = robot.flywheel;
        Transfer transfer = robot.transfer;
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        waitForStart();
        while (opModeIsActive()) {
            robot.flywheel.setPIDFF(kP, kI, kD, kS, kV, kA, integralMax);
            flywheel.setTargetRPM(targetRPM);
            if (stopShooter){
                flywheel.setPower(0);
            } else if (usePower) {
                flywheel.setPower(power);
            } else {
                flywheel.updateFly();
            }

            if (gamepad1.rightBumperWasPressed()){
                transfer.setIntakeMode();
            } else if (gamepad1.circleWasPressed()){
                transfer.setEmptyMode();
            } else if (gamepad1.xWasPressed()){
                transfer.setHoldMode();
            } else if (gamepad1.rightTriggerWasPressed()){
                transfer.setUptakeMode();
            }

            telemetry.addData("RPM", flywheel.getRPM());
            telemetry.addData("Target RPM", flywheel.getTargetRPM());
            telemetry.addData("At Target", flywheel.isAtTarget());
            telemetry.addData("Power", flywheel.getPower());
            transfer.transferUpdate();
            telemetry.update();
        }
    }

}
