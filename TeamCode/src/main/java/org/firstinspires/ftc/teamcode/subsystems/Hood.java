/**
 * Contains methods to work with the hood.
 * We plan on just using the position of the Servo itself in all of our regressions for the
 * shooter, so no need for any degrees bullshit; that was really annoying last year.
 *
 * @author Carter McKay
 * @version 9/24/26
 */

package org.firstinspires.ftc.teamcode.subsystems;

import com.pedropathing.follower.Follower;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class Hood {
    private HardwareMap hwMap;
    private Follower follower;
    private Telemetry telemetry;
    private Robot robot;
    private final Servo pitchServo;
    private final AnalogInput pitchEncoder;

    public Double pitchTarget = null;

    private static final class HoodParams{

        //ALL TBD!!!
        public static final double MIN_PITCH_POS = 0;
        public static final double MAX_PITCH_POS = 1;
        public static final double PITCH_GEAR_RATIO = 0;
        public static final double PITCH_ENCODER_ZERO_OFFSET = 0;
        public static final double PITCH_POSITION_OFFSET = MIN_PITCH_POS;
        public static final double PITCH_TOLERANCE = 0;
    }



    public Hood(HardwareMap hwMap, Telemetry telemetry, Robot robot) {
        this.robot = robot;
        this.hwMap = hwMap;
        this.telemetry = telemetry;

        pitchServo = hwMap.get(Servo.class, "pitchServo");
        //pitchServo.setDirection(Servo.Direction.REVERSE);
        //^ perhaps

        pitchEncoder = hwMap.get(AnalogInput.class, "pitchEncoder");
        //idk if we're using an analog input again but i think that's what the axons have right?
    }

    public void setPitchTarget(double target) {
        this.pitchTarget = target;
    }

    public void setPose(double pose) {
        this.pitchServo.setPosition(pose);
    }

    public void goToTargetPose() {
        if (pitchTarget != null)
            this.pitchServo.setPosition(pitchTarget);
    }

    public double getPitchPose() {
        return pitchEncoder.getVoltage()/pitchEncoder.getMaxVoltage();
    }


}
