package org.firstinspires.ftc.teamcode.math;

// UNITS: Inch, Gram, Second, Radian
// Projectile trajectory solver using RK4 integration and Newton-Raphson iteration.
// Accounts for gravity, drag, and Magnus effect.
// I swear to god if I have to edit this again i will go back in time to personally assinate Runge, Newton, Kutta, Raphson, and Magnus

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
            //update values
            prevPos = pos;
            prevVel = vel;
            //step 1
            Vector k1Vel = getAcceleration(vel, spin);
            //step 2
            Vector vel2 = vel.plus(k1Vel.scale(0.5 * DT));
            Vector k2Vel = getAcceleration(vel2, spin);
            //step 3
            Vector vel3 = vel.plus(k2Vel.scale(0.5 * DT));
            Vector k3Vel = getAcceleration(vel3, spin);
            //step 4
            Vector vel4 = vel.plus(k3Vel.scale(DT));
            Vector k4Vel = getAcceleration(vel4, spin);
            //final updates
            pos = pos.plus(vel.plus(vel2.scale(2.0)).plus(vel3.scale(2.0)).plus(vel4).scale(DT / 6.0));
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

        //check if anything is weird and broken, then calculate magnus
        if (v < 1e-8) { return new Vector(0, 0, -G); }
        if (spin.abs() < 1e-8 || spin.cross(vel).abs() < 1e-8) {
            magnus = new Vector(0, 0, 0);
        } else {
            //work, you piece of shit, i hate you
            magnus = spin.cross(vel).normal().scale(0.5 * RHO * v * v * area * cl / mass);
        }

        return new Vector(drag.x() + magnus.x(), drag.y() + magnus.y(), drag.z() + magnus.z() - G);
    }

    // Newton-Raphson solver
    public void calculateLaunch() {
        //check if we messed up somewhere, abort if so
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
            //get errors and check if something broke
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

            //Jacobian, except I'm using midpoints niow because I'm just so cool
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

            // check if simulations broke, then continue jacobian formation
            if (!rxPlus.valid || !rxMinus.valid || !ryPlus.valid || !ryMinus.valid || !rzPlus.valid || !rzMinus.valid) {break;}
            double twoEps = 2.0 * DERIVATIVE_EPS;
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

            //multiply jacobian and error vector to find the change for Novel, Two kilo-pounds, knock, child
            double determinant = jacobian.det();
            if (!Double.isFinite(determinant) || Math.abs(determinant) < 1e-12) {break;}
            Vector negError = new Vector(-eY, -eZ, -eA);
            Vector dV;
            try { dV = jacobian.inverse().multiply(negError); }
            catch (Exception ignored) { break; }
            if (!dV.isFinite()) { break; }

            //change velocity according to newton
            vel = vel.plus(dV.scale(DAMPING));
            if (vel.x() <= 0.0) { break;}
        }

        // Only if the solver actually converge, preety sure this is redundant, but i dont wanna risk removing it
        if (converged) { launchVel = vel; }
        else {launchVel = new Vector(0, 0, 0); }
    }

    // Normalize angle to [-PI, PI].
    // i feel like this should be in a different class...
    private static double normalizeAngle(double angle) {
        while (angle > Math.PI) { angle -= 2.0 * Math.PI; }
        while (angle < -Math.PI) { angle += 2.0 * Math.PI; }
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