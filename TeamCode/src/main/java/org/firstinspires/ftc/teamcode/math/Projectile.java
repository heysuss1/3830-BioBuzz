package org.firstinspires.ftc.teamcode.math;

//UNITS: Inch, Gram, Second, Radian
// sometimes degrees actually
// new 3d projectile tradjectory solver
// uses RK4 and newton-rhapson methods
// currently accounts for gravity, drag, and I'm gonna add magnus effect soon
public class Projectile {

    private static final double G = 386.08858;
    private static final double RHO = 2.00742e-5; //air density
    private static final double DT = 0.001;
    private static final double MAX_FLIGHT_TIME = 10.0;

    //ball measurements
    private final double cd; //drag coefficent
    private final double sp; //spin parameter
    private final double mass;
    private final double area;

    //robot and target state
    private Vector target; // position relative to robot
    private double targetAngle;
    private Vector robotVel; // reletive to feild
    private Vector ballSpin; // magnitude = rads per sec

    //soulution RELATIVE TO SHOOTER ON ROBOT
    private Vector launchVel = new Vector(0, 0, 0);

    //things that are constant between all pollen / nectar
    public Projectile(double cd, double sp, double mass, double area) {
        this.cd   = cd;
        this.sp = sp;
        this.mass = mass;
        this.area = area;
    }

    //things that will vary between shots
    public void setTarget(Vector targetPos, double targetAngleDeg, Vector robotVelocity, Vector ballSpinRPM) {
        this.target      = targetPos;
        this.targetAngle = Math.toRadians(targetAngleDeg);
        this.robotVel    = robotVelocity;
        this.ballSpin = ballSpinRPM.scale(Math.PI / 30.0);
    }

    public Vector getLaunchVel() {
        return launchVel;
    }


    // RK4 SIMULATION
    /* integrate (RK4) with intital velocity relative to the feild
       (not the air, hopefully there isn't any wind lol)
       when x = target.x() returns the residuals in Y, Z, and impact angle */
    private SimulationResult simulate(Vector relativeMuzzle, Vector spin) {
        // absolute initial state
        // we're gonna ignore the fact that ball spin is gonna slow down
        Vector vel = relativeMuzzle.plus(robotVel);
        Vector pos = new Vector(0, 0, 0);

        Vector prevPos = pos;
        Vector prevVel = vel;
        double t = 0.0;

        // intirgtae until we hit target x or timeout
        while (pos.x() < target.x() && t < MAX_FLIGHT_TIME) {
            prevPos = pos;
            prevVel = vel;

            //RK4 is a fascinating thing,
            Vector a1 = getAcceleration(vel, spin);
            Vector k1Pos = vel;
            Vector k1Vel = a1;

            Vector vel2 = vel.plus(k1Vel.scale(0.5 * DT));
            Vector a2   = getAcceleration(vel2, spin);
            Vector k2Pos = vel2;
            Vector k2Vel = a2;

            Vector vel3 = vel.plus(k2Vel.scale(0.5 * DT));
            Vector a3   = getAcceleration(vel3, spin);
            Vector k3Pos = vel3;
            Vector k3Vel = a3;

            Vector vel4 = vel.plus(k3Vel.scale(DT));
            Vector a4   = getAcceleration(vel4, spin);
            Vector k4Pos = vel4;
            Vector k4Vel = a4;

            //weighted average, i know notation is bad, sowwy  (pos += DT/6 * (k1 + 2k2 + 2k3 + k4))
            pos = pos.plus(k1Pos.plus(k2Pos.scale(2)).plus(k3Pos.scale(2)).plus(k4Pos).scale(DT / 6.0));
        //  pos += DT/6 * (k1 + 2*k2 + 2*k3 +k4)
            vel = vel.plus(k1Vel.plus(k2Vel.scale(2)).plus(k3Vel.scale(2)).plus(k4Vel).scale(DT / 6.0));
        //  vel += DT/6 * (k1Vel + 2*k2Vel + 2*k3Vel +k4Vel)
            t += DT;
        }

        // Did we ever reach the target plane?
        if (pos.x() < target.x() || pos.x() <= prevPos.x() + 1e-12) {
            return SimulationResult.REJECTED;
        }

        // Linear intoerpolation to finish RK4 and get final system state
        double fraction = (target.x() - prevPos.x()) / (pos.x() - prevPos.x());
        fraction = Math.max(0.0, Math.min(1.0, fraction));
        Vector finalPos = prevPos.plus(pos.minus(prevPos).scale(fraction));
        Vector finalVel = prevVel.plus(vel.minus(prevVel).scale(fraction));
        double finalAngle = finalVel.pitch();

        //and now, error in system states
        return new SimulationResult(finalPos.y() - target.y(), finalPos.z() - target.z(), finalAngle - targetAngle);
    }

    //acceleration due to gravity and drag. you could add magnus stuff here if you wanted.
    private Vector getAcceleration(Vector vel, Vector spin) {
        double v = vel.abs();
        if (v < 1e-8) {
            return new Vector(0, 0, -G);
        }
        double dragAcc = 0.5 * RHO * v * v * cd * area / mass;
        Vector drag = vel.scale(-dragAcc / v);
        Vector magnus = spin.cross(vel).scale(sp);
        return new Vector(drag.x() + magnus.x(), drag.y() + magnus.y(), drag.z() + magnus.z() - G);
    }

    //Newton rhapson
    public void calculateLaunch() {
        if (target.x() <= 0.0) {
            launchVel = new Vector(0, 0, 0);
            return;
        }

        // Initial guess
        Vector vel = new Vector(50, 50, 50);

        final double eps = 1e-5;
        final double tol = 1e-4;
        final int maxIter = 80;
        final double damp = 0.7;

        for (int iter = 0; iter < maxIter; iter++) {
            // Get current errors
            SimulationResult r = simulate(vel, ballSpin);
            double eY = r.yError;
            double eZ = r.zError;
            double eA = r.angleError;

            if (Math.abs(eY) < tol && Math.abs(eZ) < tol && Math.abs(eA) < tol) {
                break; // Errors within tolerance
            }

            // build Jacobian
            SimulationResult rx = simulate(vel.plus(new Vector(eps, 0, 0)), ballSpin);
            SimulationResult ry = simulate(vel.plus(new Vector(0, eps, 0)), ballSpin);
            SimulationResult rz = simulate(vel.plus(new Vector(0, 0, eps)), ballSpin);
            Matrix jacobian = new Matrix((rx.yError - eY) / eps, (ry.yError - eY) / eps, (rz.yError - eY) / eps, (rx.zError - eZ) / eps, (ry.zError - eZ) / eps, (rz.zError - eZ) / eps, (rx.angleError - eA) / eps, (ry.angleError - eA) / eps, (rz.angleError - eA) / eps);

            if (Math.abs(jacobian.det()) < 1e-12) {
                break;
            }

            Vector negError = new Vector(-eY, -eZ, -eA);
            Vector dV = jacobian.inverse().multiply(negError);
            vel = vel.plus(dV.scale(damp));
        }
        this.launchVel = vel;
    }

    //result holder thingy
    private static class SimulationResult {
        static final SimulationResult REJECTED = new SimulationResult(Double.MAX_VALUE, Double.MAX_VALUE, Double.MAX_VALUE);
        final double yError;
        final double zError;
        final double angleError;
        SimulationResult(double yError, double zError, double angleError) {
            this.yError     = yError;
            this.zError     = zError;
            this.angleError = angleError;
        }
    }
}