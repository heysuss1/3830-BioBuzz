package org.firstinspires.ftc.teamcode.controllers;

import android.util.Log;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Robot;

@Config
@TeleOp(name = "Turret PIDF tuner")
public class TuneTurretPidf extends LinearOpMode {
    Robot robot;
    private Telemetry telemetryA;
    public static double kP, kI, kD, kF, iZone;
    public static double targetPos;

    FtcDashboard dashboard = FtcDashboard.getInstance();
    public void runOpMode(){

        telemetryA = new MultipleTelemetry(this.telemetry, dashboard.getTelemetry());
        robot = new Robot(hardwareMap, telemetryA);
        robot.shooter.getTurretController().setPidCoefficients(kP, kI, kD, kF, iZone);

        waitForStart();

        while (opModeIsActive()){
            robot.shooter.getTurretController().setPidCoefficients(kP, kI, kD, kF, iZone);
            robot.shooter.setTurretDegrees(targetPos);
            robot.shooter.turretTask();
            telemetryA.addData("Target Position (degrees)", targetPos);
            telemetryA.addData("Current Position (degrees)", robot.shooter.getTurretDegrees());
            telemetryA.addData("Current Power", robot.shooter.getTurretServo().getPower());
            telemetryA.addData("Current Error", robot.shooter.getTurretController().getError());
            telemetryA.addData("Raw turret position", robot.shooter.getTurretRawPose());
            telemetryA.update();
            Log.i("ROBOT","rawCurrPos: " + robot.shooter.getTurretRawPose());
        }
    }
}