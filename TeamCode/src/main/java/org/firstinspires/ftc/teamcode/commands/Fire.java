package org.firstinspires.ftc.teamcode.commands;

import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.math.AimCalculator;

public class Fire {
    private final Robot robot;
    private final ElapsedTime timer = new ElapsedTime();
    private final double SHOT_TIME = 1.5;
    enum FireStates{START, FIRE, DONE}
    FireStates fireState = FireStates.START;
    public FireStates getFireState(){
        return fireState;
    }
    public void setFireState(FireStates fireState){
        timer.reset();
        this.fireState = fireState;
    }
    public Fire(Robot robot) {
        this.robot = robot;
    }
    public boolean isFinished(){
        return fireState == FireStates.DONE;
    }
    public void start(){
        setFireState(FireStates.START);
    }


    //This current system has you press a button and then shoot and it's done, you look for more (auto shooting more not yet created)
    public void update(){
        switch(fireState) {
            case START:
                if (robot.allSystemsReady() || (timer.seconds() > 2)) {
                    setFireState(FireStates.FIRE);
                }
                break;
            case FIRE:
                robot.transfer.setUptakeMode();
                if (timer.seconds() > SHOT_TIME) {
                    end(false);
                    setFireState(FireStates.DONE);
                }
                break;
        }
    }
    public void end(boolean interrupted) {
            robot.transfer.setHoldMode();
    }

}
