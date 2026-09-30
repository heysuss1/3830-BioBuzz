package org.firstinspires.ftc.teamcode.commands;

import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.math.AimCalculator;

public class Fire {
    private Robot robot;
    private ElapsedTime timer = new ElapsedTime();
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
        setFireState(FireStates.FIRE);
    }


    //This current system has you press a button and then shoot and it's done, you look for more (auto shooting more not yet created)
    public void update(){
        switch(fireState) {
            case START:
                if (robot.allSystemsReady()) {
                    setFireState(FireStates.FIRE);
                }
                break;
            case FIRE:
                robot.transfer.setUptakeMode();
                robot.turret.calcTurretTarget( //hypothetically this works
                        AimCalculator.calcRawTargetYaw(robot.follower.pose()),
                        robot.follower.pose().heading()
                );
                if (timer.seconds() > 2) {
                    setFireState(FireStates.DONE);
                }
                break;
        }
    }
    public void end(boolean interrupted) {
            robot.transfer.setHoldMode();
    }

}
