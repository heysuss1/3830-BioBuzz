package org.firstinspires.ftc.teamcode.opmodes.autos;

import com.pedropathing.api.Paths;
import com.pedropathing.api.PoseFactory;
import com.pedropathing.math.Pose;
import com.pedropathing.paths.Path;
import com.pedropathing.utils.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.commands.Fire;


@Autonomous(name = "Diddy Auto 4 Ben")
public class DiddyAuto extends LinearOpMode{

    enum PathState{
        START,
        SHOOTING,
        INTAKE,
        PARKING,
        DONE
    }

    Robot robot;
    int shotCount = 0;

    PathState state = PathState.START;
    Timer fullAutoTimer;
    Fire fire;
    Timer timer;

    private final PoseFactory poseFactory = PoseFactory.degrees();

    private final Pose start = poseFactory.of(64.9398, 133.2527, 90);
    private final Pose point1 = poseFactory.of(6.8215, 118.5364, 270);
    private final Pose point1Control1 = poseFactory.of(43.9214, 109.3698, 0);

    public Path toParking() {
        return Paths.curve(start, point1Control1, point1).constant(point1);
    }
        public void setState(PathState state){
            this.state = state;
            timer.reset();
        }
        public void pathUpdate(){
        switch (state){
            case START:
                //If the robot is done moving, (so it was either never moving or is that the end of its path),
                //then start shooting.
                if (!robot.follower.isBusy()){
                    fire.start();
                    setState(PathState.SHOOTING);
                }
                break;
            case SHOOTING:
                //If its done shooting, and it was the first shot, go intake. If it was the second shot, then go park.
                if (fire.isFinished()){
                    robot.follower.follow(toParking());
                    setState(PathState.DONE);
                }
                break;
            case PARKING:
                if (!robot.follower.isBusy()){
                    setState(PathState.DONE);
                }
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
                robot.flywheel.setTargetRPM(3470.0);
                robot.turret.calcTurretTarget(
                        robot.aimCalc.calcRawTargetYaw(robot.follower.pose()),
                        robot.follower.pose().heading()
                );
                robot.hood.setPitchTarget(0.0);
                Robot.setTeleOpStartPose(robot.follower.pose());
                if (fullAutoTimer.seconds() > 15){
                    pathUpdate();
                }

                robot.update();
            }
            Robot.setTeleOpStartPose(robot.follower.pose());
    }




}
