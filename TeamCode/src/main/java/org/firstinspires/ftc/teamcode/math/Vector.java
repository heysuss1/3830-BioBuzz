package org.firstinspires.ftc.teamcode.math;
/*
VECTOR CLASS
 - XYZ form is cartesian, RPY is radius pitch and yaw.
 - never make a RPY vector
*/

public class Vector {
    private final double x;
    private final double y;
    private final double z;

    public Vector(double x, double y, double z) {
        this.x = x;
        this.y = y;
        this.z = z;
    }

    // XYZ calculations
    public Vector plus(Vector that) {
        return new Vector(this.x + that.x, this.y + that.y, this.z + that.z);
    }

    public Vector minus(Vector that) {
        return new Vector(this.x - that.x, this.y - that.y, this.z - that.z);
    }

    public Vector scale(double k) {
        return new Vector(k * this.x, k * this.y, k * this.z);
    }

    public double abs() {
        return Math.sqrt(this.x * this.x + this.y * this.y + this.z * this.z);
    }

    public double absXY() {
        return Math.sqrt(this.x * this.x + this.y * this.y);
    }

    //RPY calculations
    public double radius() {
        return this.abs();
    }

    public double pitch() {
        double absolute = this.abs();

        if (absolute < 1e-6) {
            return 0;
        }

        return Math.asin(this.z / absolute);
    }

    public double yaw() {
        return Math.atan2(this.y, this.x);
    }

    // getters
    public double x() {
        return this.x;
    }

    public double y() {
        return this.y;
    }

    public double z() {
        return this.z;
    }

    //convert forms
    public static Vector rpyToXYZ(double radius, double pitch, double yaw) {
        double x = radius * Math.cos(yaw) * Math.cos(pitch);
        double y = radius * Math.sin(yaw) * Math.cos(pitch);
        double z = radius * Math.sin(pitch);
        return new Vector(x, y, z);
    }
}