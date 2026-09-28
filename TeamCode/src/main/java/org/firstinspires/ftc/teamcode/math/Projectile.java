package org.firstinspires.ftc.teamcode.math;

// UNITS: Inch, Gram, Second, Radian
// Projectile trajectory solver using RK4 integration and Newton-Raphson iteration.
// Accounts for gravity, drag, and Magnus effect.
// I swear to god if i have to edit this again i will go back in time to personally assinate Runge, Newton, Kutta, Raphson, and Magnus

public class Projectile {
    private static final double G = 386.08858;
    private static final double RHO = 0.00118;
    private static final double DT = 0.001;
    private static final double MAX_FLIGHT_TIME = 10.0;

    // Numerical solver settings
    private static final double DERIVATIVE_EPS = 0.01;
    private static final double ERROR_TOLERANCE = 1e-4;
    private static final double ANGLE_TOLERANCE = 1e-5;
    private static final int MAX_ITERATIONS = 80;
    private static final double DAMPING = 0.7;

    // Ball measurements
    private final double cd; // drag coefficent
    private final double cl; // lift coefficent, this isn't actually a constant, but for our purposes it's okay
    private final double mass;
    private final double area;

    // Robot and target state
    private Vector target;
    private double targetAngle;
    private Vector robotVel;
    private Vector ballSpin;

    // Solution relative to shooter on robot
    private Vector launchVel = new Vector(0, 0, 0);

    //Ball phsyical constants
    public Projectile(double cd, double cl, double mass, double area) {
        this.cd = cd;
        this.cl = cl;
        this.mass = mass;
        this.area = area;
    }

    // Values that vary between shots
    public void setTarget(Vector targetPos, double targetAngleDeg, Vector robotVelocity, Vector ballSpinRPM) {
        this.target = targetPos;
        this.targetAngle = Math.toRadians(targetAngleDeg);
        this.robotVel = robotVelocity;
        this.ballSpin = ballSpinRPM.scale(Math.PI / 30.0);
    }

    public Vector getLaunchVel() {
        return launchVel;
    }

    // RK4 simulation
    private SimulationResult simulate(Vector relativeRobot, Vector spin) {
        Vector vel = relativeRobot.plus(robotVel);
        Vector pos = new Vector(0, 0, 0);
        Vector prevPos = pos;
        Vector prevVel = vel;
        double t = 0.0;

        //rk4 loop
        while (pos.x() < target.x() && t < MAX_FLIGHT_TIME) {
            prevPos = pos;
            prevVel = vel;

            //rk4 degree 1
            Vector a1 = getAcceleration(vel, spin);
            Vector k1Pos = vel;
            Vector k1Vel = a1;
            //degree 2
            Vector vel2 = vel.plus(k1Vel.scale(0.5 * DT));
            Vector a2 = getAcceleration(vel2, spin);
            Vector k2Pos = vel2;
            Vector k2Vel = a2;
            //degree 3
            Vector vel3 = vel.plus(k2Vel.scale(0.5 * DT));
            Vector a3 = getAcceleration(vel3, spin);
            Vector k3Pos = vel3;
            Vector k3Vel = a3;
            //degree 4
            Vector vel4 = vel.plus(k3Vel.scale(DT));
            Vector a4 = getAcceleration(vel4, spin);
            Vector k4Pos = vel4;
            Vector k4Vel = a4;

            // Position, Velocity, time update
            pos = pos.plus(k1Pos.plus(k2Pos.scale(2.0)).plus(k3Pos.scale(2.0)).plus(k4Pos).scale(DT / 6.0));
            vel = vel.plus(k1Vel.plus(k2Vel.scale(2.0)).plus(k3Vel.scale(2.0)).plus(k4Vel).scale(DT / 6.0));
            t += DT;
        }

        // check if things are still valid
        double dx = pos.x() - prevPos.x();
        if (pos.x() < target.x()) {return SimulationResult.rejected();}
        if (pos.x() - prevPos.x() <= 1e-12) {return SimulationResult.rejected();}

        //find final values, errors, return result
        double fraction = (target.x() - prevPos.x()) / dx;
        if (fraction < 0.0 || fraction > 1.0) {return SimulationResult.rejected();}
        Vector finalPos = prevPos.plus(pos.minus(prevPos).scale(fraction));
        Vector finalVel = prevVel.plus(vel.minus(prevVel).scale(fraction));
        double finalAngle = finalVel.pitch();

        //find and return errrors
        double yError = finalPos.y() - target.y();
        double zError = finalPos.z() - target.z();
        double angleError = normalizeAngle(finalAngle - targetAngle);
        return new SimulationResult(yError, zError, angleError, true);
    }

    // Acceleration due to gravity, aerodynamic drag, and the Magnus effect.
    private Vector getAcceleration(Vector vel, Vector spin) {
        double v = vel.abs();
        Vector magnus;
        double dragAcc = 0.5 * RHO * v * v * cd * area / mass;
        Vector drag = vel.scale(-dragAcc / v);

        //check if anything is weird and broken
        if (v < 1e-8) { return new Vector(0, 0, -G); }
        if (spin.abs() < 1e-8 || spin.cross(vel).abs() < 1e-8) {
            magnus = new Vector(0, 0, 0);
        } else {
            magnus = spin.cross(vel).normal().scale(0.5 * RHO * v * v * area * cl / mass);
        }

        return new Vector(drag.x() + magnus.x(), drag.y() + magnus.y(), drag.z() + magnus.z() - G);
    }

    // Newton-Raphson solver
    public void calculateLaunch() {
        //check broekn
        if (target == null || robotVel == null || ballSpin == null) {
            launchVel = new Vector(0, 0, 0);
            return;
        }
        if (target.x() <= 0.0) {
            launchVel = new Vector(0, 0, 0);
            return;
        }

        // Initial guess
        Vector vel = new Vector(50, 50, 50);
        boolean converged = false;

        // new-ton-rap-son loop
        for (int iter = 0; iter < MAX_ITERATIONS; iter++) {
            SimulationResult r = simulate(vel, ballSpin);

            if (!r.valid) { break; }

            double eY = r.yError;
            double eZ = r.zError;
            double eA = r.angleError;

            // Check convergence
            if (Math.abs(eY) < ERROR_TOLERANCE && Math.abs(eZ) < ERROR_TOLERANCE && Math.abs(eA) < ANGLE_TOLERANCE) {
                converged = true;
                break;
            }

            //Jacobian midpoint thingy
            Vector xPlus = vel.plus(new Vector(DERIVATIVE_EPS, 0, 0));
            Vector xMinus = vel.plus(new Vector(-DERIVATIVE_EPS, 0, 0));
            Vector yPlus = vel.plus(new Vector(0, DERIVATIVE_EPS, 0));
            Vector yMinus = vel.plus(new Vector(0, -DERIVATIVE_EPS, 0));
            Vector zPlus = vel.plus(new Vector(0, 0, DERIVATIVE_EPS));
            Vector zMinus = vel.plus(new Vector(0, 0, -DERIVATIVE_EPS));
            SimulationResult rxPlus = simulate(xPlus, ballSpin);
            SimulationResult rxMinus = simulate(xMinus, ballSpin);
            SimulationResult ryPlus = simulate(yPlus, ballSpin);
            SimulationResult ryMinus = simulate(yMinus, ballSpin);
            SimulationResult rzPlus = simulate(zPlus, ballSpin);
            SimulationResult rzMinus = simulate(zMinus, ballSpin);

            // check if simulations broke
            if (!rxPlus.valid || !rxMinus.valid || !ryPlus.valid || !ryMinus.valid || !rzPlus.valid || !rzMinus.valid) {break;}
            double twoEps = 2.0 * DERIVATIVE_EPS;

            // jacobian time!
            double dyDx = (rxPlus.yError - rxMinus.yError) / twoEps;
            double dzDx = (rxPlus.zError - rxMinus.zError) / twoEps;
            double daDx = (rxPlus.angleError - rxMinus.angleError) / twoEps;
            double dyDy = (ryPlus.yError - ryMinus.yError) / twoEps;
            double dzDy = (ryPlus.zError - ryMinus.zError) / twoEps;
            double daDy = (ryPlus.angleError - ryMinus.angleError) / twoEps;
            double dyDz = (rzPlus.yError - rzMinus.yError) / twoEps;
            double dzDz = (rzPlus.zError - rzMinus.zError) / twoEps;
            double daDz = (rzPlus.angleError - rzMinus.angleError) / twoEps;
            Matrix jacobian = new Matrix(dyDx, dyDy, dyDz, dzDx, dzDy, dzDz, daDx, daDy, daDz);

            //use jacobian to figure out
            double determinant = jacobian.det();
            if (!Double.isFinite(determinant) || Math.abs(determinant) < 1e-12) {break;}

            //multiply jacobian by a vector gives you your change for the newton thingy
            Vector negError = new Vector(-eY, -eZ, -eA);
            Vector dV;
            try { dV = jacobian.inverse().multiply(negError); }
            catch (Exception ignored) { break; }
            if (!dV.isFinite()) { break; }

            //change velocity according to newton
            vel = vel.plus(dV.scale(DAMPING));
            if (vel.x() <= 0.0) { break;}
        }

        // Only accept the solution if the solver actually converged.
        if (converged) { launchVel = vel; }
        else {launchVel = new Vector(0, 0, 0); }
    }

    // Normalize angle to [-PI, PI].
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