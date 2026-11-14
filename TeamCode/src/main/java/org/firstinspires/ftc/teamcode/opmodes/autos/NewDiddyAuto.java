package org.firstinspires.ftc.teamcode.opmodes.autos;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;

import com.pedropathing.api.Paths;
import com.pedropathing.api.PoseFactory;
import com.pedropathing.math.Pose;
import com.pedropathing.paths.Path;
import com.pedropathing.utils.Timer;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.commands.Fire;

public class NewDiddyAuto extends LinearOpMode {
    enum PathState{
        START,
        SHOOTING,
        INTAKE,
        PARKING,
        DONE
    }

    Robot robot;
    int shotCount = 0;

    DiddyAuto.PathState state = DiddyAuto.PathState.START;
    Timer fullAutoTimer;
    Fire fire;
    Timer timer;

    private final PoseFactory poseFactory = PoseFactory.degrees();

    private final Pose start = poseFactory.of(6.1747, 114.8531, 90);
    private final Pose point1 = poseFactory.of(6.0763, 133.426, 270);

    public Path toParking() {
        return Paths.line(start, point1).constant(point1);
    }
    public void setState(DiddyAuto.PathState state){
        this.state = state;
        timer.reset();
    }
    public void pathUpdate(){
        switch (state){
            case PARKING:
                robot.follower.follow(toParking());
                setState(DiddyAuto.PathState.DONE);
                break;
            case DONE:
                break;

        }
    }

    public void runOpMode(){
        robot = new Robot(hardwareMap, telemetry);
        fire = new Fire(robot);
        robot.follower.setPose(start);

        Robot.autoHasRan = true;
        timer = new Timer();
        fullAutoTimer = new Timer();
        waitForStart();
        fullAutoTimer.reset();
        while (opModeIsActive()){


            fire.update();
            telemetry.addData("Auto State", state);
            telemetry.addData("Fire State", fire.getFireState());

            telemetry.addData("Position", robot.follower.pose());
            telemetry.addData("Shot Count", shotCount);
            telemetry.update();
//            robot.flywheel.setTargetRPM(3350.0);
//            robot.turret.calcTurretTarget(
//                    robot.aimCalc.calcRawTargetYaw(robot.follower.pose()),
//                    robot.follower.pose().heading()
//            );
            robot.hood.setPitchTarget(0.0);
            Robot.setTeleOpStartPose(robot.follower.pose());
            pathUpdate();
            robot.update();
        }
        Robot.setTeleOpStartPose(robot.follower.pose());
    }
}
