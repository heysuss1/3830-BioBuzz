package org.firstinspires.ftc.teamcode.math;

import com.pedropathing.math.Pose;

import org.firstinspires.ftc.teamcode.FieldConstants;
import org.firstinspires.ftc.teamcode.Robot;

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

    //constructor (was yeeted)

    private Pose getHiveTarget(Robot.Team team, Pose robotPos){
        double hivePos_x;
        double hivePos_y;
        if (Robot.Team.RED == team) {
            if (robotPos.y() > 73){
                hivePos_x = FieldConstants.TOP_RED_HIVE_POS.x();
                hivePos_y = FieldConstants.TOP_RED_HIVE_POS.y();
            } else {
                hivePos_x = FieldConstants.BOTTOM_RED_HIVE_POS.x();
                hivePos_y = FieldConstants.BOTTOM_BLUE_HIVE_POS.y();
            }
        } else {
            if (robotPos.y() > 73){
                hivePos_x = FieldConstants.TOP_BLUE_HIVE_POS.x();
                hivePos_y = FieldConstants.TOP_BLUE_HIVE_POS.y();
            } else {
                hivePos_x = FieldConstants.BOTTOM_BLUE_HIVE_POS.x();
                hivePos_y = FieldConstants.BOTTOM_BLUE_HIVE_POS.y();
            }
        }
        return new Pose(hivePos_x, hivePos_y);
    }
    //update outputs
    public void update(Pose robotPos, Robot.Team team, Vector robotVel) {
        //TODO: refactor to use pedro poses (they have their own inbuilt distance functions)
        //TODO: go to pedropathing and see what velocity they return and refactor ur code to use that
        //pedro stuff goes here to update robotPos, robotVel, hivePos
        double hivePos_x = getHiveTarget(team, robotPos).x();
        double hivePos_y = getHiveTarget(team, robotPos).y();;


        Vector robotVector = new Vector(robotPos.x(), robotPos.y(), 6.7); //idk what z is
        Vector hiveVector = new Vector(hivePos_x,hivePos_y, 6.7); // idk what z is


        //Calculate R,P,Y for a stationary robot
        Vector deltaPos = hiveVector.minus(robotVector);
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
