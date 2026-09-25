package org.firstinspires.ftc.teamcode.subsystems;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.teamcode.controllers.PIDFF;
//uhhh hi this is imp
public class Flywheel {
    private final DcMotorEx flyMotor;
    private final ElapsedTime timer;
    private final PIDFF pidff = new PIDFF(0,0,0,0,0,0,0);
    private double targetRPM = 0.0;
    private double lastTime;
    private static final double TICKS_PER_REV = 28.0;

    public Flywheel(HardwareMap hwMap){
        this.flyMotor = hwMap.get(DcMotorEx.class, "flywheel");
        this.flyMotor.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        this.flyMotor.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        this.timer = new ElapsedTime();
        this.timer.reset();
        this.lastTime = this.timer.seconds();
    }

    public void setTargetRPM(double targetRPM) {
        this.targetRPM = targetRPM;
    }

    public double getRPM() {
        double ticksPerSecond = flyMotor.getVelocity();
        return ticksPerSecond*60.0/TICKS_PER_REV;
    }

    public void updateFly(){
        double currentTime = timer.seconds();
        double dt = currentTime - lastTime;
        lastTime = currentTime;

        double totalPower = pidff.calcPIDFF(getRPM(), targetRPM, dt, 0, 0);
        totalPower = Math.min(Math.max(totalPower, -1), 1);
        flyMotor.setPower(totalPower);
    }
}
