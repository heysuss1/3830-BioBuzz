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

    private Pose getHiveTarget(Pose robotPos){
        double hivePos_x;
        double hivePos_y;
//            if (robotPos.y() > 73){
//                hivePos_x = FieldConstants.TOP_RED_HIVE_POS.x();
//                hivePos_y = FieldConstants.TOP_RED_HIVE_POS.y();
//            } else {
                hivePos_x = FieldConstants.BOTTOM_RED_HIVE_POS.x();
                hivePos_y = FieldConstants.BOTTOM_RED_HIVE_POS.y();
//            }

        return new Pose(hivePos_x, hivePos_y);
    }
    //update outputs
    public void update(Pose robotPos, Robot.Team team, Vector robotVel) {
        //TODO: refactor to use pedro poses (they have their own inbuilt distance functions)
        //TODO: go to pedropathing and see what velocity they return and refactor ur code to use that
        //pedro stuff goes here to update robotPos, robotVel, hivePos
        double hivePos_x = getHiveTarget(robotPos).x();
        double hivePos_y = getHiveTarget(robotPos).y();


        Vector robotVector = new Vector(robotPos.x(), robotPos.y(), 6.7); //idk what z is
        Vector hiveVector = new Vector(hivePos_x,hivePos_y, 6.7); // idk what z is


        //Calculate R,P,Y for a stationary robot
        Vector deltaPos = hiveVector.minus(robotVector);
        double distance = deltaPos.absXY();

        double rpm /*Temp*/ = rpmRegress(distance);
        double pitch /*Temp*/ = pitchRegress(distance);
        double yaw /*Temp*/ = Math.atan2( deltaPos.y(), deltaPos.x()  );

        //Convert to X,Y,Z so you can subtract robot movement
//        double ballSpeed = rpmTemp / k;
//        Vector ballVelTemp = Vector.rpyToXYZ(ballSpeed, pitchTemp, yawTemp);
//        Vector ballVel = ballVelTemp.minus(robotVel);

        //Convert back to R,P,Y
//        double radius = ballVel.radius();
//        this.rpm = radius*k;
//        this.pitch = ballVel.pitch();
//        this.yaw = ballVel.yaw();
        this.rpm = rpmRegress(distance);
        this.pitch = pitchRegress(distance);
        this.yaw =  Math.atan2( deltaPos.y(), deltaPos.x()  );
    }

    //regressions
    //see https://www.desmos.com/calcuator/zz6c359i9y
    private double rpmRegress(double d) {
        if(d<=22) {
            return 16.666667*d + 2683.3333;
        } else if (d<=24) {
            return 0*d + 3050;
        } else if (d<=31) {
            return 7.1428571*d + 2878.5714;
        } else if (d<=38) {
            return 14.285714*d + 2657.1429;
        } else {
            return 25*d + 2250;
        }
        //fallback to a quardratic regression
        return 0.4447 * d * d - 15.16*d + 3147.72;
    }
    private double pitchRegress(double d) { //returns the servo position

        pitch = -0.2182 * Math.atan(1.03464 * d - 22.63423) + 0.32795;
        return Math.round(pitch*100) / 100;
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
