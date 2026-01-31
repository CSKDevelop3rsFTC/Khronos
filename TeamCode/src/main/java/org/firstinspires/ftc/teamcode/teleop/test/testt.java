package org.firstinspires.ftc.teamcode.teleop.test;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.pedropathing.Constants;
import org.firstinspires.ftc.teamcode.teleop.meet3.Meet3TeleOpB;
import org.firstinspires.ftc.teamcode.teleop.ShooterPID;

@Autonomous(name = "testt", group = "Autonomous")
public class testt extends OpMode {

    // Pedro follower
    private Follower follower;
    private Timer pathTimer;
    private int pathState;

    // Paths / PathChains
    private PathChain preload, preintake1;

    private final Pose startPose    = new Pose(123, 60, Math.toRadians(45));
    private final Pose end       = new Pose(100, 60, Math.toRadians(45));
    private void setPathState(int s) {
        pathState = s;
        pathTimer.resetTimer();
    }

    // --------------------------------------------------------------------
    // PATH BUILDING
    // --------------------------------------------------------------------
    private void buildPath() {

        preload = follower.pathBuilder()
                .addPath(new BezierLine(startPose, end))
                .setLinearHeadingInterpolation(startPose.getHeading(), end.getHeading())
                .build();
        preintake1 = follower.pathBuilder()
                .addPath(new BezierLine(end, startPose))
                .setLinearHeadingInterpolation(end.getHeading(), startPose.getHeading())
                .build();
    }
    @Override
    public void init() {
        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(startPose);

        pathTimer = new Timer();

        buildPath();
    }



    public void start(){
        setPathState(0);
    }

    @Override
    public void loop() {
        follower.update();
        pathUpdate();
    }

    private void pathUpdate() {

        switch (pathState) {


            case 0:
                follower.followPath(preload, true);
                setPathState(1);
                break;
            case 1:
                follower.followPath(preintake1, true);
                setPathState(2);
                break;
        }
    }
}