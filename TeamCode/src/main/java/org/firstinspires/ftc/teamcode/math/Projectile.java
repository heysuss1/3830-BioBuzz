package org.firstinspires.ftc.teamcode.math;

// UNITS: Inch, Gram, Second, Radian
// Projectile trajectory solver using RK4 integration and Newton-Raphson iteration.
// Accounts for gravity, drag, and Magnus effect.
// I swear to god if I have to edit this again i will go back in time to personally assinate Runge, Newton, Kutta, Raphson, and Magnus

public class Projectile {
    //Physical constants
    private static final double G = 386.08858;
    private static final double RHO = 0.00118;
    private static final double DT = 0.001;
    private static final double MAX_FLIGHT_TIME = 10.0;

    // Numerical solver settings
    private static final double DERIVATIVE_EPS = 0.005;
    private static final double ERROR_TOLERANCE = 1e-4;
    private static final double ANGLE_TOLERANCE = 1e-5;
    private static final int MAX_ITERATIONS = 100;
    private static final double DAMPING = 0.8;
    private static final boolean[] included; //what calculations are included, {gravity, drag, magnus}

    // Ball measurements
    private static final double cd; // drag coefficent
    private static final double cl; // lift coefficent, this isn't actually a constant, but for our purposes it's okay
    private static final double mass;
    private static final double area;

    static {
        // Set your ball qualities here (must be initialized as static finals)
        cd = 0.0; // TODO: set drag coefficient
        cl = 0.0; // TODO: set lift coefficient
        mass = 0.0; // TODO: set mass (grams)
        area = 0.0; // TODO: set cross-sectional area
        included = new boolean[]{true, true, true}; // TODO: set {gravity, drag, magnus}
    }

    // Values that vary between shots
    public static Vector getLaunchVel(Vector targetPos, double targetAngleDeg, Vector robotVelocity, Vector ballSpinRPM) {
        Vector target = targetPos;
        double targetAngle = Math.toRadians(targetAngleDeg);
        Vector robotVel = robotVelocity;
        Vector ballSpin = ballSpinRPM.scale(Math.PI / 30.0);
        // note: ballSpin depends on how fast the ball is launched
        // which depeneds on the physics simulator
        // which depends on ball spin... so... whoops
        // we can use calcLaunchGravOnly to approximate thi, hopefully
        return calculateLaunch(target, targetAngle, robotVel, ballSpin);
    }

    // RK4 simulation
    private static SimulationResult simulate(Vector relativeRobot, Vector spin, Vector target, Vector robotVel, double targetAngle) {
        private static Vector getAcceleration(Vector vel, Vector spin) {
            double v = vel.abs();
            Vector drag = new Vector(0, 0, 0);
            Vector magnus = new Vector(0, 0, 0);
            double gravityContribution = 0;

            //if we want gravity, do that
            if (included[0]) {
                gravityContribution = -G;
            }

            //if we want drag, and the ball is moving, do that
            if (included[1] && v > 1e-8) {
                double dragAcc = 0.5 * RHO * v * v * cd * area / mass;
                drag = vel.scale(-dragAcc / v);
            }

            //if we want magnus, and the ball is moving and spinning, do that
            if (included[2] && v > 1e-8 && spin.abs() > 1e-8) {
                Vector crossProduct = spin.cross(vel);
                if (crossProduct.abs() > 1e-8) {
                    // WORK PLEASE
                    magnus = crossProduct.normal().scale(0.5 * RHO * v * v * area * cl / mass);
                }
            }
            return new Vector(drag.x() + magnus.x(), drag.y() + magnus.y(), drag.z() + magnus.z() + gravityContribution);
        }


        //solver with only gravity
        public static Vector calcLaunchGravOnly(Vector target, double theta) {
            double x = target.x();
            double y = target.y();
            double z = target.z();

            double R = Math.sqrt(x * x + y * y);
            double vh = R * Math.sqrt(G / (2 * (z - R * Math.tan(theta))));

            double vx = x * vh / R;
            double vy = y * vh / R;
            double vz = vh * (Math.tan(theta) + (G * R) / (vh * vh));
            return new Vector(vx, vy, vz);
        }

        // Newton-Raphson solver
        public static Vector calculateLaunch(Vector target, double targetAngle, Vector robotVel, Vector ballSpin) {
            //check if we messed up somewhere, abort if so
            if (target == null || robotVel == null || ballSpin == null) {
                return new Vector(0, 0, 0);
            }
            if (target.x() <= 0.0) {
                return new Vector(0, 0, 0);
            }

            // Initial guess
            Vector vel = calcLaunchGravOnly(target, targetAngle);
            boolean converged = false;

            // new-ton-rap-son loop
            for (int iter = 0; iter < MAX_ITERATIONS; iter++) {
                //get errors and check if something broke
                SimulationResult r = simulate(vel, ballSpin, target, robotVel, targetAngle);
                if (!r.valid) { break; }
                double eY = r.yError;
                double eZ = r.zError;
                double eA = r.angleError;

                // Check convergence
                if (Math.abs(eY) < ERROR_TOLERANCE && Math.abs(eZ) < ERROR_TOLERANCE && Math.abs(eA) < ANGLE_TOLERANCE) {
                    converged = true;
                    break;
                }

                //Jacobian, except I'm using midpoints now because I'm just so cool
                Vector xPlus = vel.plus(new Vector(DERIVATIVE_EPS, 0, 0));
                Vector xMinus = vel.plus(new Vector(-DERIVATIVE_EPS, 0, 0));
                Vector yPlus = vel.plus(new Vector(0, DERIVATIVE_EPS, 0));
                Vector yMinus = vel.plus(new Vector(0, -DERIVATIVE_EPS, 0));
                Vector zPlus = vel.plus(new Vector(0, 0, DERIVATIVE_EPS));
                Vector zMinus = vel.plus(new Vector(0, 0, -DERIVATIVE_EPS));
                SimulationResult rxPlus = simulate(xPlus, ballSpin, target, robotVel, targetAngle);
                SimulationResult rxMinus = simulate(xMinus, ballSpin, target, robotVel, targetAngle);
                SimulationResult ryPlus = simulate(yPlus, ballSpin, target, robotVel, targetAngle);
                SimulationResult ryMinus = simulate(yMinus, ballSpin, target, robotVel, targetAngle);
                SimulationResult rzPlus = simulate(zPlus, ballSpin, target, robotVel, targetAngle);
                SimulationResult rzMinus = simulate(zMinus, ballSpin, target, robotVel, targetAngle);

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
            if (converged) { return vel; }
            else {return new Vector(0, 0, 0); }
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