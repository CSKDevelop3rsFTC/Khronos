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

    // Three separate push legs
    private PathChain pushLeg1; // endPose -> balls0
    private PathChain pushLeg2; // balls0 -> push0
    private PathChain pushLeg3; // push0  -> push1

    // Shooter hardware
    private CRServo leftFeeder, rightFeeder;
    private DcMotorEx launcher;

    // === Match TeleOp defaults (editable) ===
    private double  mshooterSpeed    = 0.65;   // initial launcher power
    private double  feederPower      = -1.0;   // flip if needed
    private long    shotSpaceTimeMs  = 270;    // feeder ON per shot
    private long    singleShotTimeMs = 200;    // recovery BEFORE 2nd shot

    // Post-first-shot boost + shorter recovery (same as your TeleOp)
    private double  postFirstPower = 0.92;     // launcher power after first shot
    private long    singleShotTimeAfterFirst = 200; // recovery AFTER first shot (ms)
    private boolean firstShotDone = false;

    // Spin-up before first feed
    private long    chargeUpTimeMs   = 1600;

    // Shooter FSM
    private enum ShooterState { IDLE, SPIN_UP, FEED, INTERSHOT, SPIN_DOWN, DONE }
    private ShooterState shooterState = ShooterState.IDLE;
    private int shotsRemaining = 0;
    private final ElapsedTime shooterTimer = new ElapsedTime();

    // Poses (yours)
    private final Pose startPose = new Pose(12, 18, Math.toRadians(0));
    private final Pose endPose   = new Pose(132, 48, Math.toRadians(45));

    // push row closest to zone
    private final Pose balls0 = new Pose (55, -30, Math.toRadians(0));
    private final Pose push0  = new Pose (55, -48, Math.toRadians(0));
    private final Pose push1  = new Pose (24, -48, Math.toRadians(0));

    // Guards
    private boolean burstTriggered = false;

    // Push sequence flags
    private boolean leg1Started = false, leg1Finished = false;
    private boolean leg2Started = false, leg2Finished = false;
    private boolean leg3Started = false, leg3Finished = false;

    // Wait a beat between shooting and pushing
    private final long POST_SHOOT_SETTLE_MS = 300;
    private final ElapsedTime settleTimer = new ElapsedTime();

    // Auto stop guards
    private final long AUTO_TIMEOUT_MS = 20000; // safety stop after 20 s
    private final ElapsedTime matchTimer = new ElapsedTime();

    // Build first path (shooting run)
    private void buildPath() {
        forward = follower.pathBuilder()
                .addPath(new BezierLine(startPose, endPose))
                .setLinearHeadingInterpolation(startPose.getHeading(), endPose.getHeading())
                .addParametricCallback(0.90, this::requestBurst3)
                .build();
    }

    // Build each push leg as its own PathChain (constant heading = 0 each leg)
    private void buildPushLegs() {
        double h0 = Math.toRadians(0);

        // Leg 1: endPose -> balls0 (turn from 45° to 0° then hold 0°)
        pushLeg1 = follower.pathBuilder()
                .addPath(new BezierLine(endPose, balls0))
                .setLinearHeadingInterpolation(endPose.getHeading(), h0)
                .build();

        // Leg 2: balls0 -> push0 (hold 0°)
        pushLeg2 = follower.pathBuilder()
                .addPath(new BezierLine(balls0, push0))
                .setLinearHeadingInterpolation(h0, h0)
                .build();

        // Leg 3: push0 -> push1 (hold 0°)
        pushLeg3 = follower.pathBuilder()
                .addPath(new BezierLine(push0, push1))
                .setLinearHeadingInterpolation(h0, h0)
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
                // Finish first path
                boolean firstPathDone = (follower.getCurrentTValue() >= 0.98 || !follower.isBusy());

                // If we haven't triggered yet (failsafe), trigger at t>=0.90
                if (!burstTriggered && follower.getCurrentTValue() >= 0.90) {
                    requestBurst3();
                }

                // When path is done and shooter finishes, wait briefly then start leg 1
                if (firstPathDone && (shooterState == ShooterState.DONE || shooterState == ShooterState.IDLE)) {
                    settleTimer.reset();
                    setPathState(2); // settle before leg 1
                }
                break;

            case 2: // settle window
                if (settleTimer.milliseconds() >= POST_SHOOT_SETTLE_MS) {
                    if (!leg1Started) {
                        buildPushLegs(); // build once, right before starting the sequence
                        follower.followPath(pushLeg1, true);
                        leg1Started = true;
                    }
                    setPathState(3);
                }
                break;

            case 3: // leg 1 active
                if (follower.getCurrentTValue() >= 0.98 || !follower.isBusy()) {
                    leg1Finished = true;
                    setPathState(4);
                }
                break;

            case 4: // start leg 2
                if (!leg2Started) {
                    follower.followPath(pushLeg2, true);
                    leg2Started = true;
                }
                setPathState(5);
                break;

            case 5: // leg 2 active
                if (follower.getCurrentTValue() >= 0.98 || !follower.isBusy()) {
                    leg2Finished = true;
                    setPathState(6);
                }
                break;

            case 6: // start leg 3
                if (!leg3Started) {
                    follower.followPath(pushLeg3, true);
                    leg3Started = true;
                }
                setPathState(7);
                break;

            case 7: // leg 3 active
                if (follower.getCurrentTValue() >= 0.98 || !follower.isBusy()) {
                    leg3Finished = true;
                    setPathState(8);
                }
                break;

            case 8: // all legs done
                // handled in loop() for stop
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
        launcher.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        buildPath(); // first path only; push legs are built right before the sequence
    }

    @Override
    public void start() {
        matchTimer.reset();
        burstTriggered = false;

        leg1Started = leg1Finished = false;
        leg2Started = leg2Finished = false;
        leg3Started = leg3Finished = false;

        setPathState(0);
    }

    @Override
    public void loop() {
        follower.update();
        pathUpdate();
        updateShooter();

        // Auto end once all legs finished and shooter is done (or idle)
        boolean allLegsDone = leg1Finished && leg2Finished && leg3Finished;
        if (allLegsDone && (shooterState == ShooterState.DONE || shooterState == ShooterState.IDLE)) {
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
        telemetry.addData("leg1", "%b/%b", leg1Started, leg1Finished);
        telemetry.addData("leg2", "%b/%b", leg2Started, leg2Finished);
        telemetry.addData("leg3", "%b/%b", leg3Started, leg3Finished);
        telemetry.update();
    }

    @Override
    public void stop() {
        launcher.setPower(0);
        leftFeeder.setPower(0);
        rightFeeder.setPower(0);
    }
}
