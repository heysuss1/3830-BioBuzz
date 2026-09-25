package org.firstinspires.ftc.teamcode.math;

//This has taken way too much time
//Also i dont like how many if statements there are, but theres so many ways this can break and i dont
//know how to fix that other than if statements anyways, this probably wont work anyways, whatever
public class Projectile {
    //UNITS: Inch, gram, second, radian (ISGR)
    //unless it's in degrees

    //constants
    private static final double G = 386.08858;
    private static final double RHO = 2.00742e-5;
    private static final double DT = 0.001;

    //inputs
    private final double cd;
    private final double mass;
    private final double area;
    private double targetX;
    private double targetY;
    private double targetImpact;

    //outputs
    private double launchVelocity;
    private double launchAngle;

    public Projectile(double cd, double mass, double area) {
        this.cd = cd;
        this.mass = mass;
        this.area = area;
    }

    public void setTarget(double targetX, double targetY, double angleDegrees) {
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
        double xComp = 0.0, yComp = 0.0;
        double vxComp = 0.0, vyComp = 0.0;

        double prevX = 0.0, prevY = 0.0;
        double prevVx = vx, prevVy = vy;

        while (x < targetX && t < 100.0) {
            prevX = x;
            prevY = y;
            prevVx = vx;
            prevVy = vy;

            //Runge-Kutta degree four
            double[] k1 = getAccelerations(vx, vy);

            double v_k2x = vx + 0.5 * DT * k1[0];
            double v_k2y = vy + 0.5 * DT * k1[1];
            double[] k2 = getAccelerations(v_k2x, v_k2y);

            double v_k3x = vx + 0.5 * DT * k2[0];
            double v_k3y = vy + 0.5 * DT * k2[1];
            double[] k3 = getAccelerations(v_k3x, v_k3y);

            double v_k4x = vx + DT * k3[0];
            double v_k4y = vy + DT * k3[1];
            double[] k4 = getAccelerations(v_k4x, v_k4y);

            double dx = (DT / 6.0) * (vx + 2.0 * v_k2x + 2.0 * v_k3x + v_k4x);
            double dy = (DT / 6.0) * (vy + 2.0 * v_k2y + 2.0 * v_k3y + v_k4y);
            double dvx = (DT / 6.0) * (k1[0] + 2.0 * k2[0] + 2.0 * k3[0] + k4[0]);
            double dvy = (DT / 6.0) * (k1[1] + 2.0 * k2[1] + 2.0 * k3[1] + k4[1]);

            // Kahan summation
            double yX = dx - xComp;
            double tX = x + yX;
            xComp = (tX - x) - yX;
            x = tX;

            double yY = dy - yComp;
            double tY = y + yY;
            yComp = (tY - y) - yY;
            y = tY;

            double yVx = dvx - vxComp;
            double tVx = vx + yVx;
            vxComp = (tVx - vx) - yVx;
            vx = tVx;

            double yVy = dvy - vyComp;
            double tVy = vy + yVy;
            vyComp = (tVy - vy) - yVy;
            vy = tVy;

            t += DT;
        }

        if (x <= prevX) {
            return new SimulationResult(Double.MAX_VALUE, Double.MAX_VALUE);

        }

        double fraction = (targetX - prevX) / (x - prevX);
        double finalY = prevY + fraction * (y - prevY);
        double finalVx = prevVx + fraction * (vx - prevVx);
        double finalVy = prevVy + fraction * (vy - prevVy);
        double finalImpactAngle = Math.atan2(finalVy, finalVx);
        return new SimulationResult( finalY - targetY, finalImpactAngle - targetImpact );
    }

    private double[] getAccelerations(double vx, double vy) {
        double v = Math.sqrt(vx * vx + vy * vy);

        if (v < 1e-8) { return new double[]{0.0, -G}; }

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
        // Start with a normal ballistic estimate
        double theta0 = Math.toRadians(45.0);
        double denominator = 2.0 * Math.cos(theta0) * Math.cos(theta0) * (targetX * Math.tan(theta0) - targetY);
        double v0;
        if (denominator > 0.0) {
            v0 = Math.sqrt((G * targetX * targetX) / denominator);
        } else {
            v0 = 200.0;
        }

        if (Double.isNaN(v0) || Double.isInfinite(v0) || v0 <= 0.0) {
            v0 = 200.0;
        }

        double epsilon = 1e-6;
        int maxIterations = 100;

        for (int i = 0; i < maxIterations; i++) {
            SimulationResult res = simulate(v0, theta0);

            if (Math.abs(res.yError) < epsilon && Math.abs(res.angleError) < epsilon) {
                break;
            }

            double deltaV = Math.max(0.01, v0 * 1e-5);
            double deltaTheta = 1e-5;

            SimulationResult resV = simulate(v0 + deltaV, theta0);
            SimulationResult resTheta = simulate(v0, theta0 + deltaTheta);

            double dY_dV = (resV.yError - res.yError) / deltaV;
            double dY_dTheta = (resTheta.yError - res.yError) / deltaTheta;
            double dAngle_dV = (resV.angleError - res.angleError) / deltaV;
            double dAngle_dTheta = (resTheta.angleError - res.angleError) / deltaTheta;
            double det = dY_dV * dAngle_dTheta - dY_dTheta * dAngle_dV;

            if (Math.abs(det) < 1e-12 || Double.isNaN(det) || Double.isInfinite(det)) {
                break;
            }

            double correctionV = (-dAngle_dTheta * res.yError + dY_dTheta * res.angleError) / det;
            double correctionTheta = (dAngle_dV * res.yError - dY_dV * res.angleError) / det;

            // Limit Newton corrections for reasonable values
            correctionV = Math.max(-50.0, Math.min(50.0, correctionV));
            correctionTheta = Math.max(-Math.toRadians(5.0), Math.min(Math.toRadians(5.0), correctionTheta));

            double alpha = 0.5;
            v0 += alpha * correctionV;
            theta0 += alpha * correctionTheta;
            v0 = Math.max(0.1, Math.min(v0, 5000.0));
            theta0 = Math.max(-Math.PI / 2.0 + 0.001, Math.min(theta0, Math.PI / 2.0 - 0.001) );
        }

        this.launchVelocity = v0;
        this.launchAngle = theta0;
    }

    public void printBallData() {
        System.out.println("==================================================");
        System.out.println("BALL DATA");
        System.out.println("==================================================");
        System.out.printf("Projectile Mass : %.3f grams\n", mass);
        System.out.printf("Drag Coefficient : %.3f\n", cd);
        System.out.printf("Cross Section Area : %.4f sq. inches\n", area);
        System.out.println("==================================================");
    }

    public void printResults() {
        calculateLaunch();

        System.out.println("==================================================");
        System.out.println("TRAJECTORY CALCULATION SYSTEM RESULTS");
        System.out.println("==================================================");
        System.out.printf("Target X : %.2f in\n", targetX);
        System.out.printf("Target Y : %.2f in\n", targetY);
        System.out.printf("Target Impact Angle: %.2f degrees\n",Math.toDegrees(targetImpact));
        System.out.println("--------------------------------------------------");
        System.out.printf("Launch Velocity : %.2f inches/second\n",launchVelocity);
        System.out.printf("Launch Angle : %.2f degrees\n",Math.toDegrees(launchAngle));
        System.out.println("==================================================\n\n");
    }

    public static void main(String[] args) {
        // Ball data directly fed using native components
        Projectile pollen = new Projectile(0.55, 24.95, 1);
        pollen.setTarget(72, 55, -10);
        pollen.printBallData();
        pollen.printResults();
    }
}