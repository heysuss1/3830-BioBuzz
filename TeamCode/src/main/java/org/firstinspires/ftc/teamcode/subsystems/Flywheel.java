package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.teamcode.controllers.PIDFF;

public class Flywheel {
    private final DcMotorEx flyMotor;
    private final ElapsedTime timer;
    private final PIDFF pidff = new PIDFF(0,0,0,0,0,0,0);
    private Double targetRPM = null;
    private double lastTime;
    private boolean alwaysShoot = true;
    private VoltageSensor voltageSensor;
    private double cachedVoltage = 12.0;
    private final ElapsedTime voltageTimer = new ElapsedTime();
    private static final double TOLERANCE = 50;
    private static final double TICKS_PER_REV = 28.0;

    public Flywheel(HardwareMap hwMap){
        this.flyMotor = hwMap.get(DcMotorEx.class, "flywheel");
        this.flyMotor.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        this.flyMotor.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);

        this.timer = new ElapsedTime();
        this.timer.reset();

        voltageSensor = hwMap.get(VoltageSensor.class, "Control Hub");
        this.lastTime = this.timer.seconds();
    }

    public void setTargetRPM(Double targetRPM) {
        this.targetRPM = targetRPM;
    }
    public Double getTargetRPM() {
        if (targetRPM != null) return targetRPM;
        else return 0.0;
    }

    public boolean getAlwaysShoot() {
        return alwaysShoot;
    }
    public double getPower(){
        return flyMotor.getPower();
    }

    public void setPIDFF(double kp, double ki, double kd, double ks, double kv, double ka, double integralMax) {
        pidff.setCoefficients(kp, ki, kd, ks, kv, ka, integralMax);
    }

    public void setPower(double power){
        flyMotor.setPower(power);
    }
    public void setAlwaysShoot(boolean alwaysShoot) {
        this.alwaysShoot = alwaysShoot;
    }
    public double getRPM() {
        double ticksPerSecond = flyMotor.getVelocity();
        return ticksPerSecond*60.0/TICKS_PER_REV;
    }
    public boolean isAtTarget(){
        return Math.abs(getRPM() - targetRPM) < TOLERANCE;
    }

    private double getVoltage() {
        if (voltageTimer.milliseconds() > 500) {   // refresh twice a second
            cachedVoltage = voltageSensor.getVoltage();
            voltageTimer.reset();
        }
        return cachedVoltage;
    }

    public void updateFly(){
        double currentTime = timer.seconds();
        double dt = currentTime - lastTime;
        lastTime = timer.seconds();

        if ((this.targetRPM != null) && alwaysShoot) {
            double totalPower = pidff.calcPIDFF(getRPM(), targetRPM, dt, targetRPM, 0);
            totalPower *= 12.0 / getVoltage();
            totalPower = Math.min(Math.max(totalPower, 0), 1);
            flyMotor.setPower(totalPower);

        } else {
            flyMotor.setPower(0);
        }
    }
}
