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
        private static final double GEAR_RATIO = 32.0/120.0;
        //private static final double

    }

    public Turret(HardwareMap hwMap, Telemetry telemetry) {
        this.hwMap = hwMap;
        this.telemetry = telemetry;

        turretMotor = hwMap.get(DcMotorEx.class, "turretMotor");

        turretMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        turretMotor.setPower(0);

        turretController = new PidfController(TurretParams.KP, TurretParams.KI, TurretParams.KD, TurretParams.KF, TurretParams.I_ZONE);

    }


    public void calcTurretDegrees(Double target) {
        double turretTargetRaw = target - AimCalculator.getYaw();
        double turretTargetModulo = modularConversion(turretTargetRaw);

        //turretTarget = Range.clip(turretTargetModulo, -90, 90);
        telemetry.addData("Turret Target:", turretTargetRaw);
    }

    private static double modularConversion(double input) {
        double x = input + Math.PI;
        double floormod = x - 2 * Math.PI * Math.floor(x / (2 * Math.PI));
        return floormod - Math.PI;
    }




}
