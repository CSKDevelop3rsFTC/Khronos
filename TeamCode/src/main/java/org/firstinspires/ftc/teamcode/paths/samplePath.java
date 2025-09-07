package org.firstinspires.ftc.teamcode.paths;
import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;
import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@Autonomous(name = "samplePath", group = "Autonomous")
public class samplePath extends OpMode{
    private Follower follower;
    private Timer pathTimer;
    private int pathState;

    //poses
    private final Pose startPose = new Pose(8,51,Math.toDegrees(0));
    private final Pose startPose2 = new Pose(118,21,Math.toDegrees(0));
    private final Pose endPose = new Pose(50,0,Math.toDegrees(90));
    private final Pose corna0 = new Pose(20,98,Math.toDegrees(335));
    private final Pose corna1 = new Pose(118,21,Math.toDegrees(335));

    private PathChain forward, backForward, rotate;

    public void buildPath() {
         forward = follower.pathBuilder()
                .addPath(new BezierLine(startPose, corna1))
                .setLinearHeadingInterpolation(startPose.getHeading(), corna1.getHeading())
                .build();
        backForward = follower.pathBuilder()
                .addPath(new BezierLine(corna1, corna0))
                .setLinearHeadingInterpolation(corna1.getHeading(), corna0.getHeading())
                .build();
    }

    public void setPathState(int pState) {
        pathState = pState;
        pathTimer.resetTimer();
    }

    public void pathUpdate(){
        switch(pathState){
            case 0:
                follower.followPath(forward,true);
                setPathState(1);
                break;
            case 1:
                if(!follower.isBusy()){
                    setPathState(2);
                }
                break;
            case 2:
                if(pathTimer.getElapsedTimeSeconds() > 2.5){
                follower.followPath(backForward, true);
                setPathState(3);
                }
                break;
            case 3:
                if(follower.getCurrentTValue() > 0.98){
                    setPathState(-1);
                }
                break;
        }
    }
    @Override
    public void loop() {
        // Loop robot movement and odometry values
        follower.update();
        // Loop the finite-state machine
        pathUpdate();

        // Feedback to Driver Hub
        telemetry.addData("path state", pathState);
        telemetry.addData("x", follower.getPose().getX());
        telemetry.addData("y", follower.getPose().getY());
        telemetry.addData("heading", follower.getPose().getHeading());
        telemetry.update();
    }

    /** This method is called once at the init of the OpMode. **/
    @Override
    public void init() {
        pathTimer = new Timer();
        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(startPose);
        buildPath();
    }

    /** This method is called once at the start of the OpMode. It runs all the setup actions, including building paths and starting the path system **/
    @Override
    public void start() {
        setPathState(0);
    }

}
