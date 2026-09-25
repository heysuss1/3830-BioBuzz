package org.firstinspires.ftc.teamcode.math;

public class Projectile{
    // UNITS: SI refers to (Meter, Kilogram, Second, Degree)
    // UNITS: Custom refers to (Inch, Gram, Second, Degree)

    // Conversion and Physical (SI) constants
    private static final double INCHES_TO_METERS = 0.0254;
    private static final double METERS_TO_INCHES = 1.0 / INCHES_TO_METERS;
    private static final double GRAMS_TO_KG = 0.001;
    private static final double KG_TO_GRAMS = 1.0 / GRAMS_TO_KG;
    private static final double G = 9.80665;
    private static final double RHO = 1.225;
    private static final double DT = 0.001;

    // Inputs (SI)
    private double cd;
    private double massKg;
    private double areaM2;
    private double targetX;
    private double targetY;
    private double targetImpactAngle;

    // Outputs (Custom)
    private double launchVelocityInchesPerSec;
    private double launchAngleRad;

    // Constructor, uses projectile's phsyical qualities (Custom)
    public Projectile(double cd, double massGrams, double areaSqInches) {
        this.cd = cd;
        this.massKg = massGrams * GRAMS_TO_KG;
        this.areaM2 = areaSqInches * INCHES_TO_METERS * INCHES_TO_METERS;
    }

    // Target setter, will change for every shot (Custom)
    public void setTarget(double targetXInches, double targetYInches, double angleDegrees){
        this.targetX = targetXInches * INCHES_TO_METERS;
        this.targetY = targetYInches * INCHES_TO_METERS;
        this.targetImpactAngle = Math.toRadians(angleDegrees);
    }

    // Output Getters (Custom)
    public double getLaunchVelocityInchesPerSec() {
        return launchVelocityInchesPerSec;
    }
    public double getLaunchAngleDegrees() {
        return Math.toDegrees(launchAngleRad);
    }

    // RK4 Simulation
    // Simulates the full flight of a projectile given a starting velocity and angle.
    // Returns an array containing the tracking errors: [distance_error, impact_angle_error]
    private SimulationResult simulate(double v0, double theta0) {
        // Initial state (SI)
        double t = 0.0;
        double x = 0.0;
        double y = 0.0;
        double vx = v0 * Math.cos(theta0);
        double vy = v0 * Math.sin(theta0);

        // Kahan Summation variables
        double xComp = 0.0, yComp = 0.0;
        double vxComp = 0.0, vyComp = 0.0;

        // previous steps
        double prevX = 0.0, prevY = 0.0;
        double prevVx = vx, prevVy = vy;

        // continue simulation until ball reaches target
        while (x < targetX && t < 100.0) {
            prevX = x; prevY = y;
            prevVx = vx; prevVy = vy;

            // Runge-Kunnam phsyics simlation
            // Step 1: find current acceleration (k1)
            double[] k1 = getAccelerations(vx, vy);

            // Step 2: Predict velocity at timestep's midpoint using k1
            // Use this to get a more accurate acceleration (k2)
            double v_k2x = vx + 0.5 * DT * k1[0];
            double v_k2y = vy + 0.5 * DT * k1[1];
            double[] k2 = getAccelerations(v_k2x, v_k2y);

            //Step 3: Predict midpoint acceleration again (k3)
            double v_k3x = vx + 0.5 * DT * k2[0];
            double v_k3y = vy + 0.5 * DT * k2[1];
            double[] k3 = getAccelerations(v_k3x, v_k3y);

            //Step 4 Use k3 to get velocity at timestep end
            double v_k4x = vx + DT * k3[0];
            double v_k4y = vy + DT * k3[1];
            double[] k4 = getAccelerations(v_k4x, v_k4y);

            // get a weighted average of everything you've computed
            // somehow this is better than just using the last computation
            double dx = (vx + (DT / 6.0) * (k1[0] + 2.0 * k2[0] + 2.0 * k3[0])) * DT;
            double dy = (vy + (DT / 6.0) * (k1[1] + 2.0 * k2[1] + 2.0 * k3[1])) * DT;
            double dvx = (1.0 / 6.0) * (k1[0] + 2.0 * k2[0] + 2.0 * k3[0] + k4[0]) * DT;
            double dvy = (1.0 / 6.0) * (k1[1] + 2.0 * k2[1] + 2.0 * k3[1] + k4[1]) * DT;

            // Kahan summation predicts floating point error, somehow, and componsates for it
            // reduces error, because we're doing a lot of FP arithmatic
            double yX = dx - xComp; double tX = x + yX; xComp = (tX - x) - yX; x = tX;
            double yY = dy - yComp; double tY = y + yY; yComp = (tY - y) - yY; y = tY;
            double yVx = dvx - vxComp; double tVx = vx + yVx; vxComp = (tVx - vx) - yVx; vx = tVx;
            double yVy = dvy - vyComp; double tVy = vy + yVy; vyComp = (tVy - vy) - yVy; vy = tVy;

            t += DT;
        }

        // Because of how timesteps work, we allways overshoot just a little, this componstates fot that somehoiw
        double fraction = (targetX - prevX) / (x - prevX);
        double finalY = prevY + fraction * (y - prevY);
        double finalVx = prevVx + fraction * (vx - prevVx);
        double finalVy = prevVy + fraction * (vy - prevVy);
        double finalImpactAngle = Math.atan2(finalVy, finalVx);

        return new SimulationResult(finalY - targetY, finalImpactAngle - targetImpactAngle);
    }

    //Calculates derivitives for RK4
    private double[] getAccelerations(double vx, double vy) {
        double v = Math.sqrt(vx * vx + vy * vy);
        double dragForce = 0.5 * RHO * v * v * cd * areaM2;
        double dragAcc = dragForce / massKg;

        double ax = -(dragAcc * (vx / v));
        double ay = -G - (dragAcc * (vy / v));

        return new double[]{ax, ay};
    }

    // Object that holds simulation results
    private static class SimulationResult {
        final double yError;
        final double angleError;
        SimulationResult(double yError, double angleError) {
            this.yError = yError;
            this.angleError = angleError;
        }
    }

    //Executes a 2D Newton-Raphson boundary value solver
    //Computes launch velocity and angle instance variables required to satisfy constraints.
    public void calculateLaunch() {
        //Initial guess ignores air resistance
        double v0 = Math.sqrt((G * targetX * targetX) / (2 * Math.cos(0.78) * Math.cos(0.78) * (targetX * Math.tan(0.78) - targetY)));
        //If physics fail, we need a fallback: 20 m/s and 45 degrees
        if (Double.isNaN(v0) || v0 <= 0) v0 = 20.0;
        double theta0 = Math.toRadians(45.0);

        double epsilon = 1e-7;
        int maxIterations = 100;
        double Delta = 1e-4;

        for (int i = 0; i < maxIterations; i++) {
            SimulationResult res = simulate(v0, theta0);

            // check if error is within tolerance
            if (Math.abs(res.yError) < epsilon && Math.abs(res.angleError) < epsilon) {
                break;
            }

            // Jacobian matrix of partial derivitives
            // matrix of "dError/dInput" errors are xError and yError, input is theta and Velocity
            SimulationResult resV = simulate(v0 + Delta, theta0);
            SimulationResult resTheta = simulate(v0, theta0 + Delta);

            double dY_dV = (resV.yError - res.yError) / Delta;
            double dY_dTheta = (resTheta.yError - res.yError) / Delta;
            double dAngle_dV = (resV.angleError - res.angleError) / Delta;
            double dAngle_dTheta = (resTheta.angleError - res.angleError) / Delta;

            // Jacobian determinant
            double det = dY_dV * dAngle_dTheta - dY_dTheta * dAngle_dV;
            //singular/uninvertible check
            if (Math.abs(det) < 1e-12) {
                break;
            }

            //Invert Jacobian matrix to find the correction steps
            double deltaV = (-dAngle_dTheta * res.yError + dY_dTheta * res.angleError) / det;
            double deltaTheta = (dAngle_dV * res.yError - dY_dV * res.angleError) / det;

            // Dampening factor to protect against large gradients
            double alpha = 0.5;
            v0 += alpha * deltaV;
            theta0 += alpha * deltaTheta;

            // Physical boundary clamping constraints
            if (v0 < 0.1) v0 = 0.1;
            theta0 = Math.max(-Math.PI / 2, Math.min(Math.PI / 2, theta0));
        }

        // optimized configurations converted back to target output units (Inches/Sec, Radians)
        this.launchVelocityInchesPerSec = (v0 * METERS_TO_INCHES);
        this.launchAngleRad = theta0;
    }
}
