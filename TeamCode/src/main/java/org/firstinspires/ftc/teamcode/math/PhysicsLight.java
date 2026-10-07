package org.firstinspires.ftc.teamcode.math;

// UNITS: Inch, Gram, Second, Radian
// Projectile trajectory solver using Euler integration and Newton-Raphson iteration.
// Accounts for gravity, drag, and Magnus effect.
// This is a slightly simplified version: we have larger tolerances, larger time steps, euler instead of RK4 simulation, and foreward instead of centeral jacobians
// this probably speeds it up by around 6 to 8 times
// but we also lose accuracy; this one probably has an ac accuacy of plus/minus 2 inches (if tuned perfect)
// the origional probably has plus/minus 0.5 inches (if tuned perfect)
// If we did gravity with some fudge factors... that has potential, actually. would possibly be better than liner regression?

public class PhysicsLight {
    // Physical constants
    private static final double G = 386.08858;
    private static final double RHO = 0.00118;
    private static final double DT = 0.005;
    private static final double MAX_FLIGHT_TIME = 10.0;

    // Numerical solver settings
    private static final double DERIVATIVE_EPS = 0.01;
    private static final double ERROR_TOLERANCE = 1e-3;
    private static final double ANGLE_TOLERANCE = 1e-4;
    private static final int MAX_ITERATIONS = 30;
    private static final double DAMPING = 0.8;
    private static boolean[] included; // {gravity, drag, magnus}

    // Ball measurements
    private static double cd;
    private static double cl;
    private static double mass;
    private static double area;

    // Robot and target state
    private static Vector target;
    private static double targetAngle;
    private static Vector robotVel;
    private static Vector ballSpin;

    // Solution relative to shooter on robot
    private static Vector launchVel = new Vector(0, 0, 0);

    public static void setBallCoefficients(double cd, double cl, double mass, double area, boolean[] included) {
        PhysicsLight.cd = cd;
        PhysicsLight.cl = cl;
        PhysicsLight.mass = mass;
        PhysicsLight.area = area;
        PhysicsLight.included = included;
    }

    public static void setTarget(Vector targetPos, double targetAngleDeg, Vector robotVelocity, Vector ballSpinRPM) {
        target = targetPos;
        targetAngle = Math.toRadians(targetAngleDeg);
        robotVel = robotVelocity;
        ballSpin = ballSpinRPM.scale(Math.PI / 30.0);
    }

    public static Vector getLaunchVel() {
        calculateLaunch();
        return launchVel;
    }

    //Euler integreation instead of RK4, makes this method about 4 times faster
    //Larger time step makes it 5 times faster
    //In total: each simulation is 1/20th the time, btw we make about 24 simulations per shot calculation
    private static SimulationResult simulate(Vector relativeRobot, Vector spin) {
        Vector vel = relativeRobot.plus(robotVel);
        Vector pos = new Vector(0, 0, 0);
        Vector prevPos = pos;
        Vector prevVel = vel;
        double t = 0.0;

        while (pos.x() < target.x() && t < MAX_FLIGHT_TIME) {
            prevPos = pos;
            prevVel = vel;

            Vector acc = getAcceleration(vel, spin);
            pos = pos.plus(vel.scale(DT));
            vel = vel.plus(acc.scale(DT));
            t += DT;
        }

        double dx = pos.x() - prevPos.x();
        if (pos.x() < target.x() || dx <= 1e-12) {
            return SimulationResult.rejected();
        }

        double fraction = (target.x() - prevPos.x()) / dx;
        if (fraction < 0.0 || fraction > 1.0) {
            return SimulationResult.rejected();
        }

        Vector finalPos = prevPos.plus(pos.minus(prevPos).scale(fraction));
        Vector finalVel = prevVel.plus(vel.minus(prevVel).scale(fraction));
        double finalAngle = finalVel.pitch();

        double yError = finalPos.y() - target.y();
        double zError = finalPos.z() - target.z();
        double angleError = normalizeAngle(finalAngle - targetAngle);
        return new SimulationResult(yError, zError, angleError, true);
    }

    // Acceleration due to gravity, quadratic drag, and Magnus effect
    private static Vector getAcceleration(Vector vel, Vector spin) {
        double v = vel.abs();
        Vector drag = new Vector(0, 0, 0);
        Vector magnus = new Vector(0, 0, 0);
        double gravityContribution = 0.0;

        if (included[0]) {
            gravityContribution = -G;
        }

        if (included[1] && v > 1e-8) {
            double dragAcc = 0.5 * RHO * v * v * cd * area / mass;
            drag = vel.scale(-dragAcc / v);
        }

        if (included[2] && v > 1e-8 && spin.abs() > 1e-8) {
            Vector crossProduct = spin.cross(vel);
            if (crossProduct.abs() > 1e-8) {
                magnus = crossProduct.normal().scale(0.5 * RHO * v * v * area * cl / mass);
            }
        }

        return new Vector(
                drag.x() + magnus.x(),
                drag.y() + magnus.y(),
                drag.z() + magnus.z() + gravityContribution
        );
    }

    // Gravity-only closed-form initial guess
    public static Vector calcLaunchGravOnly(Vector target, double theta) {
        double x = target.x();
        double y = target.y();
        double z = target.z();

        double R = Math.sqrt(x * x + y * y);
        double vh = R * Math.sqrt(G / (2.0 * (z - R * Math.tan(theta))));

        double vx = x * vh / R;
        double vy = y * vh / R;
        double vz = vh * (Math.tan(theta) + (G * R) / (vh * vh));
        return new Vector(vx, vy, vz);
    }

    // Newton-Raphson with forward (one-sided) differences — only 3 extra simulations per iteration
    public static void calculateLaunch() {
        if (target == null || robotVel == null || ballSpin == null || target.x() <= 0.0) {
            launchVel = new Vector(0, 0, 0);
            return;
        }

        Vector vel = calcLaunchGravOnly(target, targetAngle);
        boolean converged = false;

        for (int iter = 0; iter < MAX_ITERATIONS; iter++) {
            SimulationResult r = simulate(vel, ballSpin);
            if (!r.valid) {
                break;
            }

            double eY = r.yError;
            double eZ = r.zError;
            double eA = r.angleError;

            if (Math.abs(eY) < ERROR_TOLERANCE &&
                    Math.abs(eZ) < ERROR_TOLERANCE &&
                    Math.abs(eA) < ANGLE_TOLERANCE) {
                converged = true;
                break;
            }

            // Simpler jacobian formation, half the time
            // we make about 6 jacobians per shot
            Vector xPlus = vel.plus(new Vector(DERIVATIVE_EPS, 0, 0));
            Vector yPlus = vel.plus(new Vector(0, DERIVATIVE_EPS, 0));
            Vector zPlus = vel.plus(new Vector(0, 0, DERIVATIVE_EPS));

            SimulationResult rxPlus = simulate(xPlus, ballSpin);
            SimulationResult ryPlus = simulate(yPlus, ballSpin);
            SimulationResult rzPlus = simulate(zPlus, ballSpin);

            if (!rxPlus.valid || !ryPlus.valid || !rzPlus.valid) {
                break;
            }

            double dyDx = (rxPlus.yError - eY) / DERIVATIVE_EPS;
            double dzDx = (rxPlus.zError - eZ) / DERIVATIVE_EPS;
            double daDx = (rxPlus.angleError - eA) / DERIVATIVE_EPS;

            double dyDy = (ryPlus.yError - eY) / DERIVATIVE_EPS;
            double dzDy = (ryPlus.zError - eZ) / DERIVATIVE_EPS;
            double daDy = (ryPlus.angleError - eA) / DERIVATIVE_EPS;

            double dyDz = (rzPlus.yError - eY) / DERIVATIVE_EPS;
            double dzDz = (rzPlus.zError - eZ) / DERIVATIVE_EPS;
            double daDz = (rzPlus.angleError - eA) / DERIVATIVE_EPS;

            Matrix jacobian = new Matrix(
                    dyDx, dyDy, dyDz,
                    dzDx, dzDy, dzDz,
                    daDx, daDy, daDz
            );

            double determinant = jacobian.det();
            if (!Double.isFinite(determinant) || Math.abs(determinant) < 1e-12) {
                break;
            }

            Vector negError = new Vector(-eY, -eZ, -eA);
            Vector dV;
            try {
                dV = jacobian.inverse().multiply(negError);
            } catch (Exception ignored) {
                break;
            }
            if (!dV.isFinite()) {
                break;
            }

            vel = vel.plus(dV.scale(DAMPING));
            if (vel.x() <= 0.0) {
                break;
            }
        }

        if (converged) {
            launchVel = vel;
        } else {
            launchVel = new Vector(0, 0, 0);
        }
    }

    private static double normalizeAngle(double angle) {
        while (angle > Math.PI) {
            angle -= 2.0 * Math.PI;
        }
        while (angle < -Math.PI) {
            angle += 2.0 * Math.PI;
        }
        return angle;
    }

    private static class SimulationResult {
        final double yError;
        final double zError;
        final double angleError;
        final boolean valid;

        SimulationResult(double yError, double zError, double angleError, boolean valid) {
            this.yError = yError;
            this.zError = zError;
            this.angleError = angleError;
            this.valid = valid;
        }

        static SimulationResult rejected() {
            return new SimulationResult(0.0, 0.0, 0.0, false);
        }
    }
}