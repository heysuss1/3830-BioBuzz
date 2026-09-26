/**
 * to calculate stuff for the turret
 */

package org.firstinspires.ftc.teamcode.math;

import com.pedropathing.follower.Follower;

//For turret
public class AimCalculator {
    private Follower follower;

    private static final double FIELD_LENGTH = 141.5;
    private static final double HIVE_LEG_X = 47.2;
    private static final double TARGET_X = 57.7;
    private static final double RIGHT_TARGET_Y = 55.0;
    private static final double LEFT_TARGET_Y = FIELD_LENGTH - RIGHT_TARGET_Y;
    private static final double MAX_SHOOTING_ANGLE = 0.3;
    //radians from y=0 to the line where we can start to shoot, should be small as possible

    public static double RIGHT_HIVE_AIM_BOUNDARY(double x) {
        return -Math.tan(MAX_SHOOTING_ANGLE) * Math.abs(x-TARGET_X) + RIGHT_TARGET_Y;
    }

    public static double LEFT_HIVE_AIM_BOUNDARY(double x) {
        return Math.tan(MAX_SHOOTING_ANGLE) * Math.abs(x-TARGET_X) + LEFT_TARGET_Y;
    }

    public AimCalculator(Follower follower) {
        this.follower = follower;
    }

    public double getYaw() {
        return follower.pose().heading();
    }

    /**
    From the driver's POV, the angle that the turret should be pointing at. Not subtracted from
    the heading of the robot yet. The set values are meant to predict roughly where it needs to
    point without wasting computing power pointing at the goal when we can't shoot there.
     Reference here: https://www.desmos.com/calculator/zimodtjzrh
     **/
    public Double calcRawTargetYaw() {
        double x = follower.pose().x();
        double y = follower.pose().y();

        if (y < RIGHT_HIVE_AIM_BOUNDARY(x)) { //in the aim @ right hive zone
            //aim towards right hive target
            return Math.atan2(RIGHT_TARGET_Y - y, TARGET_X - x);
        } else if (y > LEFT_HIVE_AIM_BOUNDARY(x)) { //in the aim @ left hive zone
            //aim towards left hive target
            return Math.atan2(LEFT_TARGET_Y - y, TARGET_X - x);
        } else if (x < HIVE_LEG_X) { //closer to the drivers
            if (y < FIELD_LENGTH / 2.0) {
                //aim at pi/6 ish rads
                return Math.PI / 6.0;
            } else {
                //aim at -pi/6 ish rads
                return -Math.PI / 6.0;
            }
        } else if (HIVE_LEG_X < x && x < FIELD_LENGTH - HIVE_LEG_X) { //under the hives
            if (y < FIELD_LENGTH / 2.0) {
                //aim at pi/2 rads
                return Math.PI / 2.0;
            } else {
                //aim at -pi/2 rads
                return -Math.PI / 2.0;
            }
        } else if (FIELD_LENGTH - HIVE_LEG_X < x) { //in the zone across from drivers
            if (y < FIELD_LENGTH / 2.0) {
                //aim at 5pi/6 ish rads
                return 5.0 * Math.PI / 6.0;
            } else {
                //aim at -5pi/6 ish rads
                return -5.0 * Math.PI / 6.0;
            }
        } else {
            //if somehow none of the above conditions are met? I covered the entire plane tho :sob:
            return null;
        }
    }



}
