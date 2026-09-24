package org.firstinspires.ftc.teamcode.controllers;

public class PIDFF {
    // PID: Proportional, Integral, and Derivative
    private final double kp;
    private final double ki;
    private final double kd;

    // Feedforward: constant, velocity, and acceleration
    private final double ks; // predicts static friction
    private final double kv; // predicts kinetic friction
    private final double ka; // predicts inertia

    // Utility vars
    private double integral = 0;
    private final double integralMax; // maximum integral size
    private double lastError = 0;
    private boolean firstRun = true;

    public PIDFF(double kp, double ki, double kd, double ks, double kv, double ka, double integralMax) {
        this.kp = kp;
        this.ki = ki;
        this.kd = kd;
        this.ks = ks;
        this.kv = kv;
        this.ka = ka;
        this.integralMax = integralMax;
    }

    public double calcPID(double measured, double target, double dt) {
        // Proportional
        double error = target - measured;

        // Derivative
        double derivative = 0;
        if (firstRun) {
            firstRun = false;
        } else if (dt > 1e-6) {
            derivative = (error - lastError) / dt;
        }
        lastError = error;

        if (dt > 1e-6) {
            integral += error * dt;
            integral = Math.max(-integralMax, Math.min(integralMax, integral));
        }

        return (kp * error) + (ki * integral) + (kd * derivative);
    }

    public double calcFF(double targetVelo, double targetAccel) {
        double staticFriction = 0.0;
        if (Math.abs(targetVelo) > 1e-6) {
            staticFriction = Math.copySign(ks, targetVelo);
        }

        return staticFriction + kv*targetVelo + ka*targetAccel;
    }

    public double calcPIDFF(double measured, double target, double dt, double targetVelo, double targetAccel) {
        return calcPID(measured, target, dt) + calcFF(targetVelo, targetAccel);
    }

    public void reset() {
        this.integral = 0.0;
        this.lastError = 0.0;
        this.firstRun = true;
    }
}
