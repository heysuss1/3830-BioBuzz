package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.controllers.PidfController;
import org.firstinspires.ftc.teamcode.math.AimCalculator;

public class Turret {
    private HardwareMap hwMap;
    private Telemetry telemetry;
    private DcMotorEx turretMotor;
    private PidfController turretController;
    private Double turretTarget = null;
    private static final class TurretParams {
        private static final double KP = 0.0, KI = 0.0, KD = 0.0, KF = 0.0, I_ZONE = 0.0;
        private static final double GEAR_RATIO = 32.0/120.0, TICKS_PER_REVOLUTION = 28;
        //private static final double
        private static final double MAX_TURRET_ROTATION = 10.0 * Math.PI / 9.0; //in each direction, in radians

    }

    public Turret(HardwareMap hwMap, Telemetry telemetry) {
        this.hwMap = hwMap;
        this.telemetry = telemetry;

        turretMotor = hwMap.get(DcMotorEx.class, "turretMotor");

        turretMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        turretMotor.setPower(0);

        turretController = new PidfController(TurretParams.KP, TurretParams.KI, TurretParams.KD, TurretParams.KF, TurretParams.I_ZONE);

    }


    public void calcTurretTarget(Double targetYaw, double currentYaw) {
        double turretTargetRaw = targetYaw - currentYaw;
        double turretTargetModulo = modularConversion(turretTargetRaw);

        telemetry.addData("Turret Target:", turretTargetRaw);
        turretTarget = turretTargetRaw;
    }

    public void goToTurretTarget() {
        turretMotor.setPower(turretController.calculate(turretTarget, getTurretRadians()));
    }

    public double getTurretRadians() {
        double ticks = turretMotor.getCurrentPosition();
        double motorRotations = ticks/TurretParams.TICKS_PER_REVOLUTION;
        double turretRotations = motorRotations * TurretParams.GEAR_RATIO;
        return 2 * Math.PI * turretRotations;
    }

    private static double modularConversion(double input) {
        double a = TurretParams.MAX_TURRET_ROTATION;
        double tau = 2 * Math.PI;
        if (-a-tau < input && input < -a) {
            return input - tau;
        } else if (-a < input && input < a) {
            return input;
        } else if (a < input && input < a + tau) {
            return input + tau;
        } else {

        }
    }




}
