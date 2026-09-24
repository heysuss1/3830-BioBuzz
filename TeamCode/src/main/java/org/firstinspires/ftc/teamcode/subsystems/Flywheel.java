package org.firstinspires.ftc.teamcode.subsystems;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

public class Flywheel {
    private final DcMotorEx flyMotor;
    private final ElapsedTime timer;

    private double kP = 0.0; // proporional
    private double kD = 0.0; // derivitive
    private double kK = 0.0; // kinetic friction
    private double kS = 0.0; // static friction

    private double targetRPM = 0.0;
    private double lastError = 0.0;
    private double lastTime = 0.0;
    private static final double TICKS_PER_REV = 28.0;
    public Flywheel(HardwareMap hwMap){
        this.flyMotor = hwMap.get(DcMotorEx.class, "flywheel");
        this.flyMotor.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        this.flyMotor.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        this.timer = new ElapsedTime();
        this.timer.reset();
    }
    public void setCoefficients(double kP, double kD, double kK, double kS) {
        this.kP = kP;
        this.kD = kD;
        this.kK = kK;
        this.kS = kS;
    }
    public void setTargetRPM(double targetRPM) {
        this.targetRPM = targetRPM;
    }
    public double getRPM() {
        double ticksPerSecond = flyMotor.getVelocity();
        return ticksPerSecond*60.0/TICKS_PER_REV;
    }
    public void updateFly(){
        //feedback (proportional and derivitive) and feedforeward (kinetic and static)
        double time = timer.seconds();
        double error = targetRPM - getRPM();
        double errorDerivitive = (error-lastError) / (time-lastTime);
        double totalPower = kP*error + kD*errorDerivitive + kK*targetRPM + kS;
        totalPower = Math.min(Math.max(totalPower, -1), 1); //clip power between -1 and 1
        flyMotor.setPower(totalPower);
        lastError = error;
        lastTime = time;
    }
}
