package org.firstinspires.ftc.teamcode.paths;

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

@Autonomous(name = "goalPathR", group = "Autonomous")
public class goalPathR extends OpMode {

    // Pathing
    private Follower follower;
    private Timer pathTimer;
    private int pathState;
    private PathChain forward;
    private PathChain finish;

    // Push legs
    private PathChain pushLeg1; // endPose -> balls0
    // private PathChain pushLeg2; // balls0 -> push0  (DISABLED)
    // private PathChain pushLeg3; // push0  -> push1  (DISABLED)

    // Shooter hardware
    private CRServo leftFeeder, rightFeeder;
    private DcMotorEx launcher;

    // === Match TeleOp defaults (editable) ===
    private double  mshooterSpeed    = 0.63;   // initial launcher power
    private double  feederPower      = -1.0;   // flip if needed
    private long    shotSpaceTimeMs  = 300;    // feeder ON per shot
    private long    singleShotTimeMs = 240;    // recovery BEFORE 2nd shot

    // Post-first-shot boost + shorter recovery (same as your TeleOp)
    private double  postFirstPower = 0.83;
    private long    singleShotTimeAfterFirst = 350;
    private boolean firstShotDone = false;

    // Spin-up before first feed
    private long    chargeUpTimeMs   = 1600;

    // Shooter FSM
    private enum ShooterState { IDLE, SPIN_UP, FEED, INTERSHOT, SPIN_DOWN, DONE }
    private ShooterState shooterState = ShooterState.IDLE;
    private int shotsRemaining = 0;
    private final ElapsedTime shooterTimer = new ElapsedTime();

    // Poses (yours)
    private final Pose pos1   = new Pose(120, -48, Math.toRadians(315));

    // push row closest to zone
    private final Pose pos2 = new Pose (96, -56, Math.toRadians(315));
    // private final Pose push0  = new Pose (24, -57, Math.toRadians(60)); // DISABLED
    // private final Pose push1  = new Pose (33, -48, Math.toRadians(45)); // DISABLED

    // Guards
    private boolean burstTriggered = false; // set true when we requestBurst3()
    private boolean shotRequested  = false; // "finish is done, shots requested"

    // Push sequence flags
    private boolean leg1Started = false, leg1Finished = false;
    // private boolean leg2Started = false, leg2Finished = false; // DISABLED
    // private boolean leg3Started = false, leg3Finished = false; // DISABLED

    // Wait a beat between shooting and pushing
    private final long POST_SHOOT_SETTLE_MS = 300;
    private final ElapsedTime settleTimer = new ElapsedTime();

    // Auto stop guards
    private final long AUTO_TIMEOUT_MS = 20000; // safety stop after 20 s
    private final ElapsedTime matchTimer = new ElapsedTime();

    // Build first path (shooting run): forward then finish. No callbacks here.
    private void buildPath() {
        forward = follower.pathBuilder()
                .addPath(new BezierLine(pos1, pos2))
                .setLinearHeadingInterpolation(pos1.getHeading(), pos2.getHeading())
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
                if (follower.getCurrentTValue() >= 0.98 || !follower.isBusy()) {
                    if (!shotRequested) {
                        requestBurst3();      // fire AFTER finish is fully done
                        shotRequested = true; // guard
                    }
                    // move on once shooter finishes or is idle (e.g., if no shots)
                    if (shooterState == ShooterState.DONE || shooterState == ShooterState.IDLE) {
                        settleTimer.reset();
                        setPathState(1); // settle then push
                    }
                }
                break;
            case 1: // Run forward (start -> mid)
                follower.followPath(forward, true);
                setPathState(2);
                break;

            case 2: // leg 1 active
                if (follower.getCurrentTValue() >= 0.98 || !follower.isBusy()) {
                    leg1Finished = true;
                    setPathState(3); // jump straight to done
                }
                break;

            case 3: // all legs done
                // handled in loop() for stop
                break;
        }
    }

    // Lifecycle
    @Override
    public void init() {
        pathTimer = new Timer();
        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(pos1);

        launcher    = hardwareMap.get(DcMotorEx.class, "launcher");
        leftFeeder  = hardwareMap.get(CRServo.class,   "left_feeder");
        rightFeeder = hardwareMap.get(CRServo.class,   "right_feeder");

        rightFeeder.setDirection(CRServo.Direction.REVERSE);
        leftFeeder.setPower(0);
        rightFeeder.setPower(0);

        launcher.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        buildPath(); // build forward + finish
    }

    @Override
    public void start() {
        matchTimer.reset();
        burstTriggered = false;
        shotRequested  = false;

        leg1Started = leg1Finished = false;

        setPathState(0);
    }

    @Override
    public void loop() {
        follower.update();
        pathUpdate();
        updateShooter();

        // Auto end once all active legs finished and shooter is done (or idle)
        boolean allLegsDone = leg1Finished;

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
        telemetry.addData("shotRequested", shotRequested);
        telemetry.addData("burstTriggered", burstTriggered);
        telemetry.update();
    }

    @Override
    public void stop() {
        launcher.setPower(0);
        leftFeeder.setPower(0);
        rightFeeder.setPower(0);
    }
}
