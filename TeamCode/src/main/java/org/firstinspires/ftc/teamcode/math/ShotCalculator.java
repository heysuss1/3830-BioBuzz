package org.firstinspires.ftc.teamcode.math;

public class ShotCalculator
{
    //UNITS: INCHES, SECONDS, ROTATIONS PER MINUTE, RADIANS
    //RPY is RPM, Pitch, and yaw
    //XYZ is cartesian

    //inputs
    Vector hivePos;
    Vector robotPos;
    Vector robotVel;

    //coefficients
    double k = 1; // ball velocity * k = RPM
    //regression coefficients go here

    //outputs
    double rpm = 0;
    double pitch = 0;
    double yaw = 0;

    //constructor
    public ShotCalculator(Vector hivePos, Vector robotPos, Vector robotVel) {
        this.hivePos = hivePos;
        this.robotPos = robotPos;
        this.robotVel = robotVel;
    }

    //update outputs
    public void update() {
        //pedro stuff goes here to update robotPos, robotVel, hivePos

        //Calculate R,P,Y for a stationary robot
        Vector deltaPos = hivePos.minus(robotPos);
        double distance = deltaPos.absXY();

        double rpmTemp = rpmRegress(distance);
        double pitchTemp = pitchRegress(distance);
        double yawTemp = Math.atan2( deltaPos.y(), deltaPos.x()  );

        //Convert to X,Y,Z so you can subtract robot movement
        double ballSpeed = rpmTemp / k;
        Vector ballVelTemp = Vector.rpyToXYZ(ballSpeed, pitchTemp, yawTemp);
        Vector ballVel = ballVelTemp.minus(robotVel);

        //Convert back to R,P,Y
        double radius = ballVel.radius();
        this.rpm = radius*k;
        this.pitch = ballVel.pitch();
        this.yaw = ballVel.yaw();
    }

    //regressions
    private double rpmRegress(double distance) {
        // using the xy distance to the hive, determine rpm
        // I'd suggest a degree-2
        return 0;
    }
    private double pitchRegress(double distance) {
        // using the xy distance to the hive, determine pitch
        // I'd suggest a degree-2
        return 0 ;
    }

    //getters
    public double getRPM(){
        return this.rpm;
    }
    public double getPitch(){
        return this.pitch;
    }
    public double getYaw(){
        return this.yaw;
    }
}
