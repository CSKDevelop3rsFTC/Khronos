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
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@Autonomous(name = "pathBlue", group = "Autonomous")
public class pathBlue extends OpMode {

    // Pathing
    private Follower follower;
    private Timer pathTimer;
    private int pathState;
    private PathChain forward;
    private PathChain finish;
    private PathChain pushLeg1;

    // Shooter hardware
    private CRServo leftFeeder, rightFeeder;
    private DcMotorEx launcher;

    // TeleOp shooter constants
    private static final double SERVO_FEED_POWER    = -1.0;
    private static final long   SHOT_SPACE_MS       = (long)(0.3 * 1000);
    private static final long   INTERSHOT_MS        = (long)(0.5 * 1000);
    private static final long   CHARGE_UP_MS        = (long)(1.6 * 1000);
    private static final long   POST_BURST_BRAKE_MS = 300;
    private static final long   COOLDOWN_MS         = 3000;

    // Launcher baseline powers at 12.3V
    private static final double BASE_POWER_1  = 0.66;
    private static final double BASE_POWER_2  = 0.75;
    private static final double BASE_POWER_34 = 0.73;

    // Voltage scaling
    private static final double NOMINAL_VOLTAGE = 12.3;
    private static final double VOLTAGE_SCALE   = 0.06; // 10% per ±0.5V

    // Shooter FSM
    private enum ShooterState { IDLE, SPIN_UP, FEED, INTERSHOT, REVERSE_BRAKE, SPIN_DOWN, COOLDOWN }
    private ShooterState shooterState = ShooterState.IDLE;

    private final ElapsedTime shooterTimer  = new ElapsedTime();
    private final ElapsedTime cooldownTimer = new ElapsedTime();

    // Burst bookkeeping
    private int  burstShots    = 0;   // remaining to fire
    private int  shotsInCycle  = 0;   // 1,2,3,4...
    private boolean shotRequested = false; // guard to request once

    // Poses
    private final Pose startPose  = new Pose(12, 18, Math.toRadians(0));
    private final Pose midPose   = new Pose(108, 18, Math.toRadians(48));
    private final Pose endPose   = new Pose(131, 48, Math.toRadians(48));

    // Push target
    private final Pose balls0    = new Pose(50, -40, Math.toRadians(48));

    // Push sequence flags
    private boolean leg1Started = false, leg1Finished = false;

    // Settle between shooting and push
    private final long POST_SHOOT_SETTLE_MS = 300;
    private final ElapsedTime settleTimer = new ElapsedTime();

    // Auto stop guards
    private final long AUTO_TIMEOUT_MS = 20000;
    private final ElapsedTime matchTimer = new ElapsedTime();

    // -------- Voltage helpers --------
    private double getVoltage() {
        double min = Double.POSITIVE_INFINITY;
        for (VoltageSensor s : hardwareMap.getAll(VoltageSensor.class)) {
            double v = s.getVoltage();
            if (v > 0) min = Math.min(min, v);
        }
        return (min == Double.POSITIVE_INFINITY) ? NOMINAL_VOLTAGE : min;
    }
    private double scalePower(double base, double voltage) {
        double diff  = voltage - NOMINAL_VOLTAGE;
        double scale = 1.0 - (diff * (VOLTAGE_SCALE / 0.5)); // ±10% per 0.5V
        return Math.max(0, Math.min(1, base * scale));
    }

    // -------- Paths --------
    private void buildPath() {
        forward = follower.pathBuilder()
                .addPath(new BezierLine(startPose, midPose))
                .setLinearHeadingInterpolation(startPose.getHeading(), midPose.getHeading())
                .build();

        finish = follower.pathBuilder()
                .addPath(new BezierLine(midPose, endPose))
                .setLinearHeadingInterpolation(midPose.getHeading(), endPose.getHeading())
                .build();
    }
    private void buildPushLegs() {
        double h = Math.toRadians(48);
        pushLeg1 = follower.pathBuilder()
                .addPath(new BezierLine(endPose, balls0))
                .setLinearHeadingInterpolation(endPose.getHeading(), h)
                .build();
    }

    // -------- Shooter trigger (4-shot burst once) --------
    private void requestBurst4() {
        if (shotRequested) return;
        if (shooterState != ShooterState.IDLE) return;

        shotRequested = true;
        burstShots    = 4;
        shotsInCycle  = 0;

        double p1 = scalePower(BASE_POWER_1, getVoltage());
        launcher.setPower(p1);
        shooterTimer.reset();
        shooterState = ShooterState.SPIN_UP;
    }

    // -------- Shooter update (TeleOp parity) --------
    private void updateShooter() {
        double v   = getVoltage();
        double p1  = scalePower(BASE_POWER_1,  v);
        double p2  = scalePower(BASE_POWER_2,  v);
        double p34 = scalePower(BASE_POWER_34, v);

        switch (shooterState) {
            case IDLE:
                leftFeeder.setPower(0);
                rightFeeder.setPower(0);
                launcher.setPower(0);
                break;

            case SPIN_UP:
                if (shooterTimer.milliseconds() >= CHARGE_UP_MS) {
                    leftFeeder.setPower(SERVO_FEED_POWER);
                    rightFeeder.setPower(SERVO_FEED_POWER);
                    shooterTimer.reset();
                    shooterState = ShooterState.FEED;
                }
                break;

            case FEED:
                if (shooterTimer.milliseconds() >= SHOT_SPACE_MS) {
                    // one shot completed
                    leftFeeder.setPower(0);
                    rightFeeder.setPower(0);

                    shotsInCycle++; // 1,2,3,4

                    // stage powers
                    if (shotsInCycle == 1) {
                        launcher.setPower(p2);
                    } else {
                        launcher.setPower(p34);
                    }

                    if (--burstShots > 0) {
                        shooterTimer.reset();
                        shooterState = ShooterState.INTERSHOT;
                    } else {
                        // Reverse brake after 4th
                        launcher.setPower(-1.0);
                        shooterTimer.reset();
                        shooterState = ShooterState.REVERSE_BRAKE;
                    }
                }
                break;

            case INTERSHOT:
                if (shooterTimer.milliseconds() >= INTERSHOT_MS) {
                    leftFeeder.setPower(SERVO_FEED_POWER);
                    rightFeeder.setPower(SERVO_FEED_POWER);
                    shooterTimer.reset();
                    shooterState = ShooterState.FEED;
                }
                break;

            case REVERSE_BRAKE:
                if (shooterTimer.milliseconds() >= POST_BURST_BRAKE_MS) {
                    launcher.setPower(0);
                    shooterState = ShooterState.SPIN_DOWN;
                }
                break;

            case SPIN_DOWN:
                launcher.setPower(0);
                shooterState = ShooterState.COOLDOWN;
                cooldownTimer.reset();
                break;

            case COOLDOWN:
                if (cooldownTimer.milliseconds() >= COOLDOWN_MS) {
                    shooterState = ShooterState.IDLE;
                }
                break;
        }

        telemetry.addData("Voltage", "%.2f V", v);
        telemetry.addData("Launcher Pwr (p1,p2,p34)", "%.2f, %.2f, %.2f", p1, p2, p34);
        telemetry.addData("ShotsInCycle", shotsInCycle);
        telemetry.addData("BurstRemaining", burstShots);
        telemetry.addData("ShooterState", shooterState);
    }

    // -------- Path state machine (shoot after finish, then push) --------
    private void setPathState(int s) { pathState = s; pathTimer.resetTimer(); }

    private void pathUpdate() {
        switch (pathState) {
            case 0: // start -> mid
                follower.followPath(forward, true);
                setPathState(1);
                break;

            case 1: // mid -> end
                if (follower.getCurrentTValue() >= 0.98 || !follower.isBusy()) {
                    follower.followPath(finish, true);
                    setPathState(2);
                }
                break;

            case 2: // at goal: shoot once, wait for shooter to finish
                if (follower.getCurrentTValue() >= 0.98 || !follower.isBusy()) {
                    requestBurst4();
                    if (shooterState == ShooterState.IDLE) {
                        settleTimer.reset();
                        setPathState(3);
                    }
                }
                break;

            case 3: // settle, then push
                if (settleTimer.milliseconds() >= POST_SHOOT_SETTLE_MS) {
                    if (!leg1Started) {
                        buildPushLegs();
                        follower.followPath(pushLeg1, true);
                        leg1Started = true;
                    }
                    setPathState(4);
                }
                break;

            case 4: // push leg active
                if (follower.getCurrentTValue() >= 0.98 || !follower.isBusy()) {
                    leg1Finished = true;
                    setPathState(5);
                }
                break;

            case 5: // done
                break;
        }
    }

    // -------- Lifecycle --------
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
        launcher.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        buildPath();
    }

    @Override
    public void start() {
        matchTimer.reset();
        shotRequested = false;
        burstShots    = 0;
        shotsInCycle  = 0;
        shooterState  = ShooterState.IDLE;

        leg1Started = leg1Finished = false;

        setPathState(0);
    }

    @Override
    public void loop() {
        follower.update();
        pathUpdate();
        updateShooter();

        boolean allLegsDone = leg1Finished;
        if (allLegsDone && shooterState == ShooterState.IDLE) {
            requestOpModeStop();
        }

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
        telemetry.update();
    }

    @Override
    public void stop() {
        launcher.setPower(0);
        leftFeeder.setPower(0);
        rightFeeder.setPower(0);
    }
}
