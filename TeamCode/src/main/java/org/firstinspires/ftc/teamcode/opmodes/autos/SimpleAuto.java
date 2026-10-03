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


@Autonomous(name = "Simple Auto")
public class SimpleAuto extends LinearOpMode {
    Robot robot;

    enum State {START, SHOOTING, INTAKING, PARKING, DONE};
    State state = State.START;
    Timer timer;
    Fire fire;
    private final PoseFactory poseFactory = PoseFactory.degrees();

    private final Pose start = poseFactory.of(62.1625, 8.5, 90);
    private final Pose loadingZonePose = poseFactory.of(5.5, 8.5, 180);
    private final Pose parkingPose = poseFactory.of(4, 95, -34.7453);
    private final Pose point2Control1 = poseFactory.of(45, 47, 0);

    public void setState(State state){
        this.state = state;

    }

    public Path toLoadingZone() {
        return Paths.line(start, loadingZonePose).linear(start, loadingZonePose);
    }

    public Path toParking() {
        return Paths.curve(loadingZonePose, point2Control1, parkingPose).tangent();
    }

    public void autoUpdate(){
        switch(state){
            case START:
                //code to shoot;
                fire.start();
                setState(State.SHOOTING);
                break;
            case SHOOTING:
                if (fire.isFinished()){
                    setState(State.INTAKING);
                }
                break;
            case INTAKING:
                robot.transfer.setIntakeMode();
                robot.follower.follow(toLoadingZone());
                setState(State.PARKING);
                break;
            case PARKING:
                //code to park;
                if (!robot.follower.isBusy()){
                    robot.transfer.setHoldMode();
                    robot.follower.follow(toParking());
                    setState(State.DONE);
                }
                break;
            case DONE:
                break;
        }
    }
    public void runOpMode() {
        robot = new Robot(hardwareMap, telemetry);
        timer = new Timer();
        fire = new Fire(robot);
        robot.follower.setPose(start);
        waitForStart();
        while (opModeIsActive()) {
            fire.update();
            autoUpdate();
            robot.follower.update();
            telemetry.addData("State", state);
            telemetry.addData("Position", robot.follower.pose());
            telemetry.update();
            robot.update();
        }
        Robot.setTeleOpStartPose(robot.follower.pose());
    }
}
