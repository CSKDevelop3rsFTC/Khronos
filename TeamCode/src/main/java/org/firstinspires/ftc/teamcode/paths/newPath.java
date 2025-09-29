package org.firstinspires.ftc.teamcode.paths;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@Autonomous(name = "newPath", group = "Autonomous")
public class newPath extends OpMode {

    // Pathing
    private Follower follower;
    private Timer pathTimer;
    private int pathState;
    private PathChain forward;

    // Shooter hardware
    private CRServo leftFeeder, rightFeeder;
    private DcMotorEx launcher;

    // Open-loop params (editable like TeleOp)
    private double  mshooterSpeed    = 0.7;    // initial launcher power
    private double  feederPower      = -1.0;   // flip sign or servo directions if needed
    private long    shotSpaceTimeMs  = 200;    // feeder ON per shot
    private long    singleShotTimeMs = 850;    // recovery BEFORE 2nd shot

    // Post-first-shot boost + shorter recovery (TeleOp-style)
    private double  postFirstPower = 0.90;               // launcher power after first shot
    private long    singleShotTimeAfterFirst = 200;      // recovery AFTER first shot (ms)
    private boolean firstShotDone = false;

    // Spin-up before first feed
    private long    chargeUpTimeMs   = 1600;

    // Shooter FSM
    private enum ShooterState { IDLE, SPIN_UP, FEED, INTERSHOT, SPIN_DOWN, DONE }
    private ShooterState shooterState = ShooterState.IDLE;
    private int shotsRemaining = 0;
    private final ElapsedTime shooterTimer = new ElapsedTime();

    // Poses
    private final Pose startPose = new Pose(12, 18, Math.toRadians(0));
    private final Pose endPose   = new Pose(132, 48, Math.toRadians(45));

    // One-shot trigger guard
    private boolean burstTriggered = false;

    // Auto stop guards
    private final long AUTO_TIMEOUT_MS = 20000; // safety stop after 20 s
    private final ElapsedTime matchTimer = new ElapsedTime();

    // Build path
    private void buildPath() {
        forward = follower.pathBuilder()
                .addPath(new BezierLine(startPose, endPose))
                .setLinearHeadingInterpolation(startPose.getHeading(), endPose.getHeading())
                .addParametricCallback(0.90, this::requestBurst3)   // fire before end
                .build();
    }

    // Shooter helpers
    private void requestBurst3() {
        if (burstTriggered) return; // only once
        if (shooterState == ShooterState.IDLE || shooterState == ShooterState.DONE) {
            burstTriggered = true;
            shotsRemaining = 3;
            firstShotDone = false;
            launcher.setPower(mshooterSpeed);
            shooterTimer.reset();
            shooterState = ShooterState.SPIN_UP;
        }
    }

    private void updateShooter() {
        switch (shooterState) {
            case IDLE:
            case DONE:
                leftFeeder.setPower(0);
                rightFeeder.setPower(0);
                launcher.setPower(0);
                break;

            case SPIN_UP:
                if (shooterTimer.milliseconds() >= chargeUpTimeMs) {
                    leftFeeder.setPower(feederPower);
                    rightFeeder.setPower(feederPower);
                    shooterTimer.reset();
                    shooterState = ShooterState.FEED;
                }
                break;

            case FEED:
                if (shooterTimer.milliseconds() >= shotSpaceTimeMs) {
                    // one shot completed
                    leftFeeder.setPower(0);
                    rightFeeder.setPower(0);

                    if (!firstShotDone) {
                        firstShotDone = true;
                        launcher.setPower(postFirstPower); // boost after first shot
                    }

                    shotsRemaining = Math.max(0, shotsRemaining - 1);
                    shooterTimer.reset();
                    shooterState = ShooterState.INTERSHOT;
                }
                break;

            case INTERSHOT:
                long needed = firstShotDone ? singleShotTimeAfterFirst : singleShotTimeMs;
                if (shooterTimer.milliseconds() >= needed) {
                    if (shotsRemaining > 0) {
                        leftFeeder.setPower(feederPower);
                        rightFeeder.setPower(feederPower);
                        shooterTimer.reset();
                        shooterState = ShooterState.FEED;
                    } else {
                        shooterState = ShooterState.SPIN_DOWN;
                    }
                }
                break;

            case SPIN_DOWN:
                launcher.setPower(0);
                leftFeeder.setPower(0);
                rightFeeder.setPower(0);
                shooterState = ShooterState.DONE;
                break;
        }

        telemetry.addData("shooter state", shooterState);
        telemetry.addData("shots remaining", shotsRemaining);
        telemetry.addData("first shot done", firstShotDone);
    }

    // Path state
    private void setPathState(int s) { pathState = s; pathTimer.resetTimer(); }

    private void pathUpdate() {
        switch (pathState) {
            case 0:
                follower.followPath(forward, true);
                setPathState(1);
                break;
            case 1:
                if (!burstTriggered && follower.getCurrentTValue() >= 0.90) {
                    requestBurst3();
                }
                if (follower.getCurrentTValue() >= 0.98 || !follower.isBusy()) {
                    setPathState(-1); // path finished
                }
                break;
        }
    }

    // Lifecycle
    @Override
    public void init() {
        pathTimer = new Timer();
        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(startPose);

        launcher    = hardwareMap.get(DcMotorEx.class, "launcher");
        leftFeeder  = hardwareMap.get(CRServo.class,   "left_feeder");
        rightFeeder = hardwareMap.get(CRServo.class,   "right_feeder");

        rightFeeder.setDirection(CRServo.Direction.REVERSE);
        leftFeeder.setPower(0);
        rightFeeder.setPower(0);

        // Open-loop behavior like TeleOp
        launcher.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        launcher.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        buildPath();
    }

    @Override
    public void start() {
        matchTimer.reset();
        burstTriggered = false;
        setPathState(0);
    }

    @Override
    public void loop() {
        follower.update();
        pathUpdate();
        updateShooter();

        // Auto end once path finished and shooter is done
        if (pathState == -1 && (shooterState == ShooterState.DONE || shooterState == ShooterState.IDLE)) {
            requestOpModeStop();
        }

        // Safety timeout
        if (matchTimer.milliseconds() >= AUTO_TIMEOUT_MS) {
            launcher.setPower(0);
            leftFeeder.setPower(0);
            rightFeeder.setPower(0);
            requestOpModeStop();
        }

        telemetry.addData("path state", pathState);
        telemetry.addData("t", follower.getCurrentTValue());
        telemetry.addData("x", follower.getPose().getX());
        telemetry.addData("y", follower.getPose().getY());
        telemetry.addData("heading", Math.toDegrees(follower.getPose().getHeading()));
        telemetry.update();
    }

    @Override
    public void stop() {
        launcher.setPower(0);
        leftFeeder.setPower(0);
        rightFeeder.setPower(0);
    }
}
