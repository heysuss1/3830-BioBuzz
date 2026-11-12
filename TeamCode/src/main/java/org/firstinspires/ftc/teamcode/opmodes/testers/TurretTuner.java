package org.firstinspires.ftc.teamcode.opmodes.testers;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.controllers.PidfController;

@Config
@TeleOp(name = "Turret Tuner", group = "Tuning")
public class TurretTuner extends LinearOpMode {
    // gains — copy into Turret.TurretParams when done
    public static double kP = 0, kI = 0, kD = 0, kF = 0, iZone = 0;

    // geometry — must match Turret.TurretParams
    public static double GEAR_RATIO = 32.0 / 120.0;
    public static double TICKS_PER_REV = 28;   // motor encoder ticks per rev of the motor OUTPUT shaft
    public static double MAX_DEG = 45;        // soft limit each way (10pi/9 rad)
    public static boolean REVERSE_MOTOR = false;

    // test settings
    public static boolean manualMode = true;
    public static double manualPower = 0.2;
    public static double targetA_Deg = 0;
    public static double targetB_Deg = 90;
    public static double maxPower = 0.6;       // power cap while tuning

    @Override
    public void runOpMode() {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        DcMotorEx motor = hardwareMap.get(DcMotorEx.class, "turretMotor");
        motor.setDirection(REVERSE_MOTOR ? DcMotor.Direction.REVERSE : DcMotor.Direction.FORWARD);
        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER); // our PID, not the hub's

        PidfController controller = new PidfController(kP, kI, kD, kF, iZone);
        boolean useB = false;

        telemetry.addLine("Turret pointing forward? Encoder zeroes now.");
        telemetry.update();
        waitForStart();

        while (opModeIsActive()) {
            if (gamepad1.aWasPressed()) useB = !useB;

            double ticks = motor.getCurrentPosition();
            double turretDeg = ticks / TICKS_PER_REV * GEAR_RATIO * 360.0;
            double targetDeg = Range.clip(useB ? targetB_Deg : targetA_Deg, -MAX_DEG, MAX_DEG);

            double power;
            if (manualMode) {
                if (gamepad1.dpad_right) power = manualPower;
                else if (gamepad1.dpad_left) power = -manualPower;
                else power = 0;
                controller.reset();
            } else {
                controller.setPidCoefficients(kP, kI, kD, kF, iZone);
                power = controller.calculate(Math.toRadians(targetDeg), Math.toRadians(turretDeg));
            }

            // soft limits: never push further past the limit (protects wires)
            if (turretDeg > MAX_DEG && power > 0) power = 0;
            if (turretDeg < -MAX_DEG && power < 0) power = 0;
            power = Range.clip(power, -maxPower, maxPower);
            motor.setPower(power);

            telemetry.addData("mode", manualMode ? "MANUAL (dpad L/R)" : "PID (A = swap target)");
            telemetry.addData("targetDeg", targetDeg);
            telemetry.addData("turretDeg", turretDeg);
            telemetry.addData("errorDeg", targetDeg - turretDeg);
            telemetry.addData("power", power);
            telemetry.addData("ticks", ticks);
            telemetry.update();
        }
        motor.setPower(0);
    }
}