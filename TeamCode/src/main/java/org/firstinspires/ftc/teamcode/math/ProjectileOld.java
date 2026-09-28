package org.firstinspires.ftc.teamcode.math;
//Okay so i just realised i forgot to account for robot movement :(
//I'll keep this as it is, and make another in 3 dimentions
//Kinda pmo... but thats just how this stuff works
public class ProjectileOld {
    // UNITS: Inch, gram, second, radian (ISGR)
    // unless it's in degrees, in which case it will have "Deg" in the name


    // constants
    private static final double G = 386.08858;
    private static final double RHO = 2.00742e-5;
    private static final double DT = 0.001;
    private static final double MAX_FLIGHT_TIME = 10.0;  //could extend if needed


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


    //Projectile physical qualitys
    public ProjectileOld(double cd, double mass, double area) {
        this.cd = cd;
        this.mass = mass;
        this.area = area;
    }


    //Target location and ideal impact angle
    public void setTarget(double targetX, double targetY, double angleDegrees) {
        // targetX is the XY distance from robot to goal, and distances are positive
        // targetY must be positive, as the robot can only shoot up. Also, if the hive is below the robot, we have bigger problems
        if (targetX <= 0.0 || targetY <= 0.0) {
            throw new IllegalArgumentException("targetX and targetY must be > 0");
        }
        this.targetX = targetX;
        this.targetY = targetY;
        this.targetImpact = Math.toRadians(angleDegrees);
    }


    //getters
    public double getLaunchVelocityInchesPerSec() {
        return launchVelocity;
    }
    public double getLaunchAngleDegrees() {
        return Math.toDegrees(launchAngle);
    }


    /* Runge-Kutta Fourth Degree (RK4) simulation of the ball
           Takes in initial velocity and angle of ball, simulates until ball reaches targetX
           finds error between ball's angle, y-position and targetAngle, targetY
           returns SimulationResult object containing errors
    */
    private SimulationResult simulate(double v0, double theta0) {
        //initial conditions
        double t = 0.0;
        double x = 0.0;
        double y = 0.0;
        double vx = v0 * Math.cos(theta0);
        double vy = v0 * Math.sin(theta0);
        double prevX = 0.0, prevY = 0.0;
        double prevVx = vx, prevVy = vy;


        //Simulation loop, 1 loop = 1 timestep
        while (x < targetX && t < MAX_FLIGHT_TIME) {
            prevX = x;
            prevY = y;
            prevVx = vx;
            prevVy = vy;


            // RK4 simulation on system state = (x, y, vx, vy)
            // RK4 is actually really cool, you should look into it
            // but if you're just trying to fix the code, please leave this alone
            // I'm like, 98% sure this isn't the issue
            double[] a1 = getAccelerations(vx, vy);
            //degree 1
            double k1x = vx;
            double k1y = vy;
            double k1vx = a1[0];
            double k1vy = a1[1];
            //degree 2
            double vx2 = vx + 0.5 * DT * k1vx;
            double vy2 = vy + 0.5 * DT * k1vy;
            double[] a2 = getAccelerations(vx2, vy2);
            double k2x = vx2;
            double k2y = vy2;
            double k2vx = a2[0];
            double k2vy = a2[1];
            //degree 3
            double vx3 = vx + 0.5 * DT * k2vx;
            double vy3 = vy + 0.5 * DT * k2vy;
            double[] a3 = getAccelerations(vx3, vy3);
            double k3x = vx3;
            double k3y = vy3;
            double k3vx = a3[0];
            double k3vy = a3[1];
            //degree 4
            double vx4 = vx + DT * k3vx;
            double vy4 = vy + DT * k3vy;
            double[] a4 = getAccelerations(vx4, vy4);
            double k4x = vx4;
            double k4y = vy4;
            double k4vx = a4[0];
            double k4vy = a4[1];


            //weighted sum
            x  += (DT / 6.0) * (k1x  + 2.0 * k2x  + 2.0 * k3x  + k4x);
            y  += (DT / 6.0) * (k1y  + 2.0 * k2y  + 2.0 * k3y  + k4y);
            vx += (DT / 6.0) * (k1vx + 2.0 * k2vx + 2.0 * k3vx + k4vx);
            vy += (DT / 6.0) * (k1vy + 2.0 * k2vy + 2.0 * k3vy + k4vy);


            t += DT;
        }


        // Reject runs that never reach targetX
        if (x < targetX || x <= prevX + 1e-12) {
            //maximum error means run was rejected
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


    //does derivitives for the RK4 in simulate method, also applies drag
    private double[] getAccelerations(double vx, double vy) {
        double v = Math.sqrt(vx * vx + vy * vy);


        if (v < 1e-8) {
            return new double[]{0.0, -G};
        }


        double dragAcc = 0.5 * RHO * v * v * cd * area / mass;


        double ax = -(dragAcc * (vx / v));
        double ay = -G - (dragAcc * (vy / v));


        return new double[]{ax, ay};
    }
    //SimulationResult object class, holds errors
    private static class SimulationResult {
        final double yError;
        final double angleError;


        SimulationResult(double yError, double angleError) {
            this.yError = yError;
            this.angleError = angleError;
        }
    }
    //calculates launch value
    public void calculateLaunch() {
        if (targetX <= 0.0) {
            launchVelocity = 0.0;
            launchAngle = 0.0;
            return;
        }


        //initial guess
        double v0 = 250;
        double theta0 = 45;


        final double epsilon = 1e-6;
        final int maxIterations = 80;


        // assumes vector-valued function f(<v0, theta0>) = <errorY, errorAngle>
        // then uses newton-rhapson to find the zeros of that function
        for (int i = 0; i < maxIterations; i++) {
            SimulationResult res = simulate(v0, theta0);


            if (Math.abs(res.yError) < epsilon && Math.abs(res.angleError) < epsilon) {
                break;
            }


            // Jacobian matrix approximation
            double deltaV = Math.max(0.05, v0 * 1e-5);
            double deltaTheta = 1e-5;
            SimulationResult resV = simulate(v0 + deltaV, theta0);
            SimulationResult resTheta = simulate(v0, theta0 + deltaTheta);
            double dY_dV     = (resV.yError     - res.yError)     / deltaV;
            double dY_dTheta = (resTheta.yError - res.yError)     / deltaTheta;
            double dA_dV     = (resV.angleError - res.angleError) / deltaV;
            double dA_dTheta = (resTheta.angleError - res.angleError) / deltaTheta;
            //determinant
            double det = dY_dV * dA_dTheta - dY_dTheta * dA_dV;
            if (Math.abs(det) < 1e-12 || !Double.isFinite(det)) {
                break;   // singular or invalid
            }
            //Uses cramer's rule to find 0s
            double correctionV     = (-dA_dTheta * res.yError + dY_dTheta * res.angleError) / det;
            double correctionTheta = ( dA_dV     * res.yError - dY_dV     * res.angleError) / det;


            //I had to put a lot of limits and stuff here to avoid it shitting itself
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
        ProjectileOld pollen = new ProjectileOld(0.55, 24.95, 1.0);
        pollen.setTarget(72, 55, -10);
        pollen.printBallData();
        pollen.printResults();
    }
}



