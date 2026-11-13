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


@Autonomous(name = "1 tip + 4 balls")
public class OneTip4Balls extends LinearOpMode {
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
    private final PoseFactory poseFactory = PoseFactory.degrees();

    Fire fire;
    Timer timer;

    private final Pose start = poseFactory.of(56, 6.6839, 180);
    private final Pose loadingZone = poseFactory.of(0.7255, 5.8765, 180);
    private final Pose topShootingZone = poseFactory.of(61.2589, 128.3761, -81.986);
    private final Pose shootingZone2Control1 = poseFactory.of(78.0166, 9.3502, 0);
    private final Pose parkingZone = poseFactory.of(6.2672, 122.8794, 5.7081);

    public Path toLoadingZone() {
        return Paths.line(start, loadingZone).linear(start, loadingZone);
    }

    public Path toShootingZone() {
        return Paths.curve(loadingZone, shootingZone2Control1, topShootingZone).reverseTangent();
    }

    public Path toParking() {
        return Paths.line(topShootingZone, parkingZone).reverseTangent();
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
                        robot.follower.follow(toParking());
                        setState(PathState.PARKING);
                    }
                }
                break;
            case INTAKE:
                if (!robot.follower.isBusy()){
                    robot.follower.follow(toShootingZone());
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
