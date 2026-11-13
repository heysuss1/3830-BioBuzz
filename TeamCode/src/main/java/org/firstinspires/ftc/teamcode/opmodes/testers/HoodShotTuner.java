package org.firstinspires.ftc.teamcode.opmodes.testers;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.pedropathing.api.PoseFactory;
import com.pedropathing.follower.Follower;
import com.pedropathing.math.Pose;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.pedro.Constants;
import org.firstinspires.ftc.teamcode.subsystems.Flywheel;
import org.firstinspires.ftc.teamcode.subsystems.Transfer;


@Config
@TeleOp(name = "Hood / Shot Tuner", group = "Tuning")
public class HoodShotTuner extends LinearOpMode {
    // hood
    public static double hoodPos = 0.5;
    public static double HOOD_MIN = 0.0;   // find the real mechanical limits, then copy into Hood
    public static double HOOD_MAX = 1.0;
    public static double HOOD_STEP = 0.02;

    // flywheel (paste from Shooter Test)
    public static double kP = 0.003, kS = 0.068, kV = 0.0001745;
    public static double targetRPM = 3000;
    public static double RPM_STEP = 100;

    // field (Pedro coords, inches). Defaults = top red hive in FieldConstants
    public static double startX = 52.8, startY = 8.2, startHeadingDeg = 90;
    public static double hiveX = 53.35, hiveY = 85;
    public static double driveScale = 0.4;

    @Override
    public void runOpMode() {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        Servo hood = hardwareMap.get(Servo.class, "pitchServo");
//        AnalogInput hoodEncoder = hardwareMap.get(AnalogInput.class, "pitchEncoder");
        Flywheel flywheel = new Flywheel(hardwareMap);
        Transfer transfer = new Transfer(hardwareMap);
        Follower follower = Constants.create(hardwareMap);
        PoseFactory pf = PoseFactory.degrees();
        follower.setPose(pf.of(startX, startY, startHeadingDeg));

        boolean spin = false;

        telemetry.addLine("Set startX/startY/startHeadingDeg to where the robot is, then press Y after start.");
        telemetry.update();
        waitForStart();

        while (opModeIsActive()) {
            // ---- inputs
            if (gamepad1.dpadUpWasPressed()) hoodPos += HOOD_STEP;
            if (gamepad1.dpadDownWasPressed()) hoodPos -= HOOD_STEP;
            if (gamepad1.dpadRightWasPressed()) targetRPM += RPM_STEP;
            if (gamepad1.dpadLeftWasPressed()) targetRPM -= RPM_STEP;
            if (gamepad1.aWasPressed()) spin = !spin;
            if (gamepad1.yWasPressed()) follower.setPose(pf.of(startX, startY, startHeadingDeg));

            if (gamepad1.rightBumperWasPressed()) transfer.setIntakeMode();
            else if (gamepad1.xWasPressed()) transfer.setHoldMode();
            else if (gamepad1.rightTriggerWasPressed()) transfer.setUptakeMode();

            // ---- drive (same signs as Scrimmage TeleOp — fix if Main Tele turns out different)
            follower.manual(
                    -gamepad1.left_stick_y * driveScale,
                    -gamepad1.left_stick_x * driveScale,
                    -gamepad1.right_stick_x * driveScale
            );
            follower.update();

            // ---- hood
            hoodPos = Range.clip(hoodPos, HOOD_MIN, HOOD_MAX);
            hood.setPosition(hoodPos);

            // ---- flywheel
            flywheel.setPIDFF(kP, 0, 0, kS, kV, 0, 0);
            if (spin) flywheel.setTargetRPM(targetRPM);
            else flywheel.setTargetRPM(null);
            flywheel.updateFly();

            transfer.transferUpdate();

            // ---- distance to hive
            Pose p = follower.pose();
            flywheel.setTargetRPM(targetRPM);
            double distance = Math.hypot(hiveX - p.x(), hiveY - p.y());
            double rpm = flywheel.getRPM();

            telemetry.addData("LOG", String.format("dist=%.1f  rpm=%.0f  hood=%.3f", distance, targetRPM, hoodPos));
            telemetry.addData("distance (in)", distance);
            telemetry.addData("pose", String.format("x=%.1f y=%.1f h=%.0f", p.x(), p.y(), Math.toDegrees(p.heading())));
            telemetry.addData("targetRPM", spin ? targetRPM : 0);
            telemetry.addData("rpm", rpm);
            telemetry.addData("rpm ready", spin && Math.abs(rpm - targetRPM) < 50);
            telemetry.addData("hood cmd", hoodPos);
//            telemetry.addData("hood encoder (0-1)", hoodEncoder.getVoltage() / hoodEncoder.getMaxVoltage());
            telemetry.update();
            flywheel.updateFly();
        }

    }
}