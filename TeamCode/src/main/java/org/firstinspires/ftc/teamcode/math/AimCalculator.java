/**
 * to calculate stuff for the turret
 */

package org.firstinspires.ftc.teamcode.math;

import com.pedropathing.follower.Follower;

//For turret
public class AimCalculator {
    private Follower follower;

    public AimCalculator(Follower follower) {
        this.follower = follower;
    }

    public double getYaw() {
        return follower.pose().heading();
    }

}
