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

@Autonomous(name = "goalPathR", group = "Autonomous")
public class goalPathR extends OpMode {

    // Pathing
    private Follower follower;
    private Timer pathTimer;
    private int pathState;
    private PathChain forward;
    private PathChain park;

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

    private int  burstShots     = 0;  // remaining in current burst
    private int  shotsInCycle   = 0;  // 1,2,3,4...
    private boolean burstRequested = false;

    // Poses (right side)
    private final Pose pos1     = new Pose(120, -48, Math.toRadians(315)); // start
    //private final Pose pos2     = new Pose(48,   22, Math.toRadians(315)); // goal correct
    private final Pose pos2    = new Pose(96, -48, Math.toRadians(315)); // start
    //  private final Pose parkPose = new Pose(24,   48, Math.toRadians(315)); // ending tile

    // Match timeout
    private final long AUTO_TIMEOUT_MS = 20000;
    private final ElapsedTime matchTimer = new ElapsedTime();

    // Voltage helpers
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

    // Trigger a 4-shot burst once (TeleOp behavior)
    private void requestBurst4() {
        if (burstRequested) return;
        if (shooterState != ShooterState.IDLE) return;

        burstRequested = true;
        burstShots     = 4;
        shotsInCycle   = 0;

        double p1 = scalePower(BASE_POWER_1, getVoltage());
        launcher.setPower(p1);
        shooterTimer.reset();
        shooterState = ShooterState.SPIN_UP;
    }

    // Shooter update
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

                    // continue or finish burst
                    if (--burstShots > 0) {
                        shooterTimer.reset();
                        shooterState = ShooterState.INTERSHOT;
                    } else {
                        launcher.setPower(-1.0); // reverse brake
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

    // Paths
    private void buildPath() {
        forward = follower.pathBuilder()
                .addPath(new BezierLine(pos1, pos2))
                .setLinearHeadingInterpolation(pos1.getHeading(), pos2.getHeading())
                .build();

       /* park = follower.pathBuilder()
                .addPath(new BezierLine(pos2, parkPose))
                .setLinearHeadingInterpolation(pos2.getHeading(), parkPose.getHeading())
                .build();*/
    }

    private void setPathState(int s) { pathState = s; pathTimer.resetTimer(); }

    // ***********************
    // SHOOT FIRST, THEN DRIVE
    // ***********************
    private void pathUpdate() {
        switch (pathState) {
            case 0:
                // Shoot immediately at start pose
                requestBurst4();                 // guarded by burstRequested + IDLE
                setPathState(1);
                break;

            case 1:
                // Wait until shooter fully completes (back to IDLE)
                if (shooterState == ShooterState.IDLE) {
                    follower.followPath(forward, true); // now drive to goal
                    setPathState(3);
                }
                break;

         /*   case 2:
                // Once at goal, go park
                if (follower.getCurrentTValue() >= 0.98 || !follower.isBusy()) {
                    follower.followPath(park, true);
                    setPathState(3);
                }
                break; */

            case 3:
                // Done parking
                if (follower.getCurrentTValue() >= 0.98 || !follower.isBusy()) {
                    setPathState(4);
                }
                break;

            case 4:
                // finished
                break;
        }
    }

    // Lifecycle
    @Override
    public void init() {
        pathTimer = new Timer();
        follower  = Constants.createFollower(hardwareMap);
        follower.setStartingPose(pos1);

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
        burstRequested = false;
        burstShots     = 0;
        shotsInCycle   = 0;
        shooterState   = ShooterState.IDLE;

        setPathState(0);
    }

    @Override
    public void loop() {
        follower.update();
        pathUpdate();
        updateShooter();

        boolean done = (pathState >= 4);
        if (done || matchTimer.milliseconds() >= AUTO_TIMEOUT_MS) {
            launcher.setPower(0);
            leftFeeder.setPower(0);
            rightFeeder.setPower(0);
            requestOpModeStop();
        }

        telemetry.addData("pathState", pathState);
        telemetry.addData("t", follower.getCurrentTValue());
        telemetry.addData("x", follower.getPose().getX());
        telemetry.addData("y", follower.getPose().getY());
        telemetry.addData("heading", Math.toDegrees(follower.getPose().getHeading()));
        telemetry.addData("burstRequested", burstRequested);
        telemetry.update();
    }

    @Override
    public void stop() {
        launcher.setPower(0);
        leftFeeder.setPower(0);
        rightFeeder.setPower(0);
    }
}
