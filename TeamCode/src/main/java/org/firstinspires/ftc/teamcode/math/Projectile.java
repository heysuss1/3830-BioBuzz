package org.firstinspires.ftc.teamcode.math;

public class Projectile {
    // UNITS: Inch, gram, second, radian (ISGR)
    // unless it's in degrees

    // constants
    private static final double G = 386.08858;
    private static final double RHO = 2.00742e-5;
    private static final double DT = 0.001;
    private static final double MAX_FLIGHT_TIME = 8.0;   // plenty for any FTC shot

    // inputs
    private final double cd;
    private final double mass;
    private final double area;
    private double targetX;
    private double targetY;
    private double targetImpact;

    // outputs
    private double launchVelocity;
    private double launchAngle;

    public Projectile(double cd, double mass, double area) {
        this.cd = cd;
        this.mass = mass;
        this.area = area;
    }

    public void setTarget(double targetX, double targetY, double angleDegrees) {
        if (targetX <= 0.0) {
            throw new IllegalArgumentException("targetX must be > 0");
        }
        this.targetX = targetX;
        this.targetY = targetY;
        this.targetImpact = Math.toRadians(angleDegrees);
    }

    public double getLaunchVelocityInchesPerSec() {
        return launchVelocity;
    }

    public double getLaunchAngleDegrees() {
        return Math.toDegrees(launchAngle);
    }

    private SimulationResult simulate(double v0, double theta0) {
        double t = 0.0;
        double x = 0.0;
        double y = 0.0;
        double vx = v0 * Math.cos(theta0);
        double vy = v0 * Math.sin(theta0);

        double prevX = 0.0, prevY = 0.0;
        double prevVx = vx, prevVy = vy;

        while (x < targetX && t < MAX_FLIGHT_TIME) {
            prevX = x;
            prevY = y;
            prevVx = vx;
            prevVy = vy;

            // Proper RK4 on state (x, y, vx, vy)
            double[] a1 = getAccelerations(vx, vy);
            double k1x = vx;
            double k1y = vy;
            double k1vx = a1[0];
            double k1vy = a1[1];

            double vx2 = vx + 0.5 * DT * k1vx;
            double vy2 = vy + 0.5 * DT * k1vy;
            double[] a2 = getAccelerations(vx2, vy2);
            double k2x = vx2;
            double k2y = vy2;
            double k2vx = a2[0];
            double k2vy = a2[1];

            double vx3 = vx + 0.5 * DT * k2vx;
            double vy3 = vy + 0.5 * DT * k2vy;
            double[] a3 = getAccelerations(vx3, vy3);
            double k3x = vx3;
            double k3y = vy3;
            double k3vx = a3[0];
            double k3vy = a3[1];

            double vx4 = vx + DT * k3vx;
            double vy4 = vy + DT * k3vy;
            double[] a4 = getAccelerations(vx4, vy4);
            double k4x = vx4;
            double k4y = vy4;
            double k4vx = a4[0];
            double k4vy = a4[1];

            x  += (DT / 6.0) * (k1x  + 2.0 * k2x  + 2.0 * k3x  + k4x);
            y  += (DT / 6.0) * (k1y  + 2.0 * k2y  + 2.0 * k3y  + k4y);
            vx += (DT / 6.0) * (k1vx + 2.0 * k2vx + 2.0 * k3vx + k4vx);
            vy += (DT / 6.0) * (k1vy + 2.0 * k2vy + 2.0 * k3vy + k4vy);

            t += DT;
        }

        // Reject runs that never reached the target plane
        if (x < targetX || x <= prevX + 1e-12) {
            return new SimulationResult(Double.MAX_VALUE, Double.MAX_VALUE);
        }

        double fraction = (targetX - prevX) / (x - prevX);
        fraction = Math.max(0.0, Math.min(1.0, fraction));   // safety clamp

        double finalY  = prevY  + fraction * (y  - prevY);
        double finalVx = prevVx + fraction * (vx - prevVx);
        double finalVy = prevVy + fraction * (vy - prevVy);
        double finalImpactAngle = Math.atan2(finalVy, finalVx);

        return new SimulationResult(finalY - targetY, finalImpactAngle - targetImpact);
    }

    private double[] getAccelerations(double vx, double vy) {
        double v = Math.sqrt(vx * vx + vy * vy);

        if (v < 1e-8) {
            return new double[]{0.0, -G};
        }

        double dragForce = 0.5 * RHO * v * v * cd * area;
        double dragAcc = dragForce / mass;

        double ax = -(dragAcc * (vx / v));
        double ay = -G - (dragAcc * (vy / v));

        return new double[]{ax, ay};
    }

    private static class SimulationResult {
        final double yError;
        final double angleError;

        SimulationResult(double yError, double angleError) {
            this.yError = yError;
            this.angleError = angleError;
        }
    }

    public void calculateLaunch() {
        if (targetX <= 0.0) {
            launchVelocity = 0.0;
            launchAngle = 0.0;
            return;
        }

        // Improved initial guess: line-of-sight plus a bias toward the desired impact angle
        double los = Math.atan2(targetY, targetX);
        double theta0 = 0.6 * los + 0.4 * targetImpact;   // weighted blend
        // keep it in a sensible open interval
        theta0 = Math.max(-Math.PI / 2.0 + 0.05, Math.min(theta0, Math.PI / 2.0 - 0.05));

        // Vacuum estimate for velocity (same formula as before, now with better angle)
        double cosTh = Math.cos(theta0);
        double denominator = 2.0 * cosTh * cosTh * (targetX * Math.tan(theta0) - targetY);
        double v0;
        if (denominator > 0.0) {
            v0 = Math.sqrt((G * targetX * targetX) / denominator);
        } else {
            v0 = 250.0;
        }

        if (!Double.isFinite(v0) || v0 <= 0.0) {
            v0 = 250.0;
        }

        final double epsilon = 1e-6;
        final int maxIterations = 80;

        for (int i = 0; i < maxIterations; i++) {
            SimulationResult res = simulate(v0, theta0);

            if (Math.abs(res.yError) < epsilon && Math.abs(res.angleError) < epsilon) {
                break;
            }

            // Finite-difference Jacobian
            double deltaV = Math.max(0.05, v0 * 1e-5);
            double deltaTheta = 1e-5;

            SimulationResult resV = simulate(v0 + deltaV, theta0);
            SimulationResult resTheta = simulate(v0, theta0 + deltaTheta);

            double dY_dV     = (resV.yError     - res.yError)     / deltaV;
            double dY_dTheta = (resTheta.yError - res.yError)     / deltaTheta;
            double dA_dV     = (resV.angleError - res.angleError) / deltaV;
            double dA_dTheta = (resTheta.angleError - res.angleError) / deltaTheta;

            double det = dY_dV * dA_dTheta - dY_dTheta * dA_dV;

            if (Math.abs(det) < 1e-12 || !Double.isFinite(det)) {
                break;   // singular or invalid Jacobian
            }

            double correctionV     = (-dA_dTheta * res.yError + dY_dTheta * res.angleError) / det;
            double correctionTheta = ( dA_dV     * res.yError - dY_dV     * res.angleError) / det;

            // Limit step size
            correctionV     = Math.max(-40.0, Math.min(40.0, correctionV));
            correctionTheta = Math.max(-Math.toRadians(4.0), Math.min(Math.toRadians(4.0), correctionTheta));

            final double alpha = 0.6;   // mild damping
            v0     += alpha * correctionV;
            theta0 += alpha * correctionTheta;

            // Hard physical bounds
            v0     = Math.max(1.0, Math.min(v0, 4000.0));
            theta0 = Math.max(-Math.PI / 2.0 + 0.01, Math.min(theta0, Math.PI / 2.0 - 0.01));
        }

        this.launchVelocity = v0;
        this.launchAngle = theta0;
    }

    public void printBallData() {
        System.out.println("==================================================");
        System.out.println("BALL DATA");
        System.out.println("==================================================");
        System.out.printf("Projectile Mass : %.3f grams%n", mass);
        System.out.printf("Drag Coefficient : %.3f%n", cd);
        System.out.printf("Cross Section Area : %.4f sq. inches%n", area);
        System.out.println("==================================================");
    }

    public void printResults() {
        calculateLaunch();

        System.out.println("==================================================");
        System.out.println("TRAJECTORY CALCULATION SYSTEM RESULTS");
        System.out.println("==================================================");
        System.out.printf("Target X : %.2f in%n", targetX);
        System.out.printf("Target Y : %.2f in%n", targetY);
        System.out.printf("Target Impact Angle: %.2f degrees%n", Math.toDegrees(targetImpact));
        System.out.println("--------------------------------------------------");
        System.out.printf("Launch Velocity : %.2f inches/second%n", launchVelocity);
        System.out.printf("Launch Angle : %.2f degrees%n", Math.toDegrees(launchAngle));
        System.out.println("==================================================\n\n");
    }

    public static void main(String[] args) {
        Projectile pollen = new Projectile(0.55, 24.95, 1.0);
        pollen.setTarget(72, 55, -10);
        pollen.printBallData();
        pollen.printResults();
    }
}