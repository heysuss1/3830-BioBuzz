package org.firstinspires.ftc.teamcode.math;

// Row-major 3x3 Matrix
public class Matrix {
    private double i11, i12, i13, i21, i22, i23, i31, i32, i33;

    public Matrix(double i11, double i12, double i13, double i21, double i22, double i23, double i31, double i32, double i33){
        this.i11 = i11; this.i12 = i12; this.i13 = i13;
        this.i21 = i21; this.i22 = i22; this.i23 = i23;
        this.i31 = i31; this.i32 = i32; this.i33 = i33;
    }
    public double det(){
        return this.i11*(this.i22*this.i33 - this.i23*this.i32) - this.i12*(this.i21*this.i33 - this.i23*this.i31) + this.i13*(this.i21*this.i32 - this.i22*this.i31);
    }
    public Matrix transpose(){
        return new Matrix(this.i11, this.i21, this.i31, this.i12, this.i22, this.i32, this.i13, this.i23, this.i33);
    }
    public Matrix cofactor() {
        double c11 =  (i22 * i33 - i23 * i32);
        double c12 = -(i21 * i33 - i23 * i31);
        double c13 =  (i21 * i32 - i22 * i31);
        double c21 = -(i12 * i33 - i13 * i32);
        double c22 =  (i11 * i33 - i13 * i31);
        double c23 = -(i11 * i32 - i12 * i31);
        double c31 =  (i12 * i23 - i13 * i22);
        double c32 = -(i11 * i23 - i13 * i21);
        double c33 =  (i11 * i22 - i12 * i21);
        return new Matrix(c11, c12, c13, c21, c22, c23, c31, c32, c33);
    }
    public Matrix inverse(){
        return this.cofactor().transpose().scale(1.0 / this.det());
    }
    public Matrix scale(double k) {
        return new Matrix(
                k * this.i11, k * this.i12, k * this.i13,
                k * this.i21, k * this.i22, k * this.i23,
                k * this.i31, k * this.i32, k * this.i33);
    }
    public Vector multiply(Vector that) {
        return new Vector(
                this.i11 * that.x() + this.i12 * that.y() + this.i13 * that.z(),
                this.i21 * that.x() + this.i22 * that.y() + this.i23 * that.z(),
                this.i31 * that.x() + this.i32 * that.y() + this.i33 * that.z()
        );
    }
}
