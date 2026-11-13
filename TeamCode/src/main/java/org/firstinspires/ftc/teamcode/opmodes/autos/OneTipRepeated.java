package org.firstinspires.ftc.teamcode.opmodes.autos;

import com.pedropathing.api.Paths;
import com.pedropathing.api.PoseFactory;
import com.pedropathing.math.Pose;
import com.pedropathing.paths.Path;
import com.pedropathing.utils.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.commands.Fire;



@Autonomous(name = "One tip Repeated")
public class OneTipRepeated extends LinearOpMode{

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
    Fire fire;
    Timer timer;

        private final PoseFactory poseFactory = PoseFactory.degrees();

        private final Pose start = poseFactory.of(56.1732, 7.4964, 180);
        private final Pose loadingZonePoint = poseFactory.of(8.7642, 7.7186, 180);
        private final Pose start_2 = poseFactory.of(56.648, 7.6757, 179.9487);
        private final Pose parkPoint = poseFactory.of(9.0308, 92.9344, -89.652);
        private final Pose parkPointControl = poseFactory.of(9.4716, 20.3571, 0);

        public Path toLoadingZone() {
            return Paths.line(start, loadingZonePoint).linear(start, loadingZonePoint);
        }

        public Path backToShoot() {
            return Paths.line(loadingZonePoint, start).reverseTangent();
        }

        public Path toPark() {
            return Paths.curve(start, parkPointControl, parkPoint).reverseTangent();
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
                    shotCount += 1;
                    if (shotCount == 1){
                        robot.transfer.setIntakeMode();
                        robot.follower.follow(toLoadingZone());
                        setState(PathState.INTAKE);
                    } else if (shotCount == 2) {
                        robot.follower.follow(toPark());
                        setState(PathState.PARKING);
                    }
                }
                break;
            case INTAKE:
                if (!robot.follower.isBusy()){
                    robot.follower.follow(backToShoot());
                    setState(PathState.START);
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
            waitForStart();
            while (opModeIsActive()){


                fire.update();
                pathUpdate();
                telemetry.addData("Auto State", state);
                telemetry.addData("Fire State", fire.getFireState());

                telemetry.addData("Position", robot.follower.pose());
                telemetry.addData("Shot Count", shotCount);
                telemetry.update();
                robot.flywheel.setTargetRPM(3350.0);
                robot.hood.setPitchTarget(0.0);
                robot.turret.lockTurret = true;
                Robot.setTeleOpStartPose(robot.follower.pose());
                robot.update();
            }
            Robot.setTeleOpStartPose(robot.follower.pose());
    }




}
