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
import org.firstinspires.ftc.teamcode.teleop.MeetZeroTeleOp;

@Autonomous(name = "goalPathB", group = "Autonomous")
public class goalPathB extends OpMode {

    // Pathing
    private Follower follower;
    private Timer pathTimer;
    private int pathState;
    private PathChain forward, park;
    private PathChain finish;

    // Push legs
    private PathChain pushLeg1; // endPose -> balls0
    // private PathChain pushLeg2; // balls0 -> push0  (DISABLED)
    // private PathChain pushLeg3; // push0  -> push1  (DISABLED)

    // Shooter hardware
    private CRServo leftFeeder, rightFeeder;
    private DcMotorEx launcher;

    // === Match TeleOp defaults (editable) ===
    // Servo constants (unchanged)
    private static final double SERVO_FEED_POWER = -1.0;
    private static final long SHOT_SPACE_MS = (long)(0.3 * 1000);
    private static final long INTERSHOT_MS  = (long)(0.5 * 1000);

    // Launcher baseline values (at 12.3V)
    private static final double BASE_POWER_1  = 0.68; // 66 first shot
    private static final double BASE_POWER_2  = 0.77; // 75 second shot (tunable midpoint)
    private static final double BASE_POWER_34 = 0.74; // 73 shots 3 and 4

    // Charge-up timing
    private static final long CHARGE_UP_MS = (long)(1.6 * 1000);

    // NEW: reverse brake after 4th shot
    private static final long POST_BURST_BRAKE_MS = 300;

    private int burstShots = 0;

    private boolean firstShotDone = false;
    private int shotsInCycle = 0;
    private boolean continuousRequested = false;
    private boolean burstRequested = false;

    private static final double NOMINAL_VOLTAGE = 12.6;
    private static final double VOLTAGE_SCALE = 0.04;

    // Spin-up before first feed
    private long    chargeUpTimeMs   = 1600;

    // Shooter FSM
    private enum ShooterState { IDLE, SPIN_UP, FEED, INTERSHOT, SPIN_DOWN, DONE, COOLDOWN, REVERSE_BRAKE }
    private ShooterState shooterState = ShooterState.IDLE;
    private int shotsRemaining = 0;
    private com.qualcomm.robotcore.util.ElapsedTime shooterTimer =
            new com.qualcomm.robotcore.util.ElapsedTime();
    private com.qualcomm.robotcore.util.ElapsedTime cooldownTimer =
            new com.qualcomm.robotcore.util.ElapsedTime();
    // Poses (yours)
    private final Pose pos1   = new Pose(123, 51, Math.toRadians(45));
    private final Pose pos2 = new Pose(108,36,Math.toRadians(45));
    // private final Pose pos2 = new Pose (72, -34, Math.toRadians(0));
    // push row closest to zone
    //private final Pose pos2 = new Pose (48, -22, Math.toRadians(45)); //CORRECT
    private final Pose pos3 = new Pose(96, 48, Math.toRadians(45));
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

    private double getVoltage() {
        double min = Double.POSITIVE_INFINITY;
        for (VoltageSensor s : hardwareMap.getAll(VoltageSensor.class)) {
            double v = s.getVoltage();
            if (v > 0) min = Math.min(min, v);
        }
        return (min == Double.POSITIVE_INFINITY) ? NOMINAL_VOLTAGE : min;
    }

    private double scalePower(double base, double voltage) {
        double diff = voltage - NOMINAL_VOLTAGE;
        double scale = 1.0 - (diff * (VOLTAGE_SCALE / 0.5)); // ±10% per 0.5V diff
        return Math.max(0, Math.min(1, base * scale));
    }

    // Build first path (shooting run): forward then finish. No callbacks here.
    private void buildPath() {
        forward = follower.pathBuilder()
                .addPath(new BezierLine(pos1, pos2))
                .setLinearHeadingInterpolation(pos1.getHeading(), pos2.getHeading())
                .build();
        park = follower.pathBuilder()
                .addPath(new BezierLine(pos2,pos3))
                .setLinearHeadingInterpolation(pos2.getHeading(), pos3.getHeading())
                .build();

    }

    // Shooter helpers
    private void requestBurst4() {
        if (burstRequested) return;
        if (shooterState != ShooterState.IDLE) return;

        burstRequested = true;
        burstShots = 4;
        shotsInCycle = 0;
        double voltage = getVoltage();

        double p1 = scalePower(BASE_POWER_1, getVoltage());
        double p2  = scalePower(BASE_POWER_2,  voltage);
        double p34 = scalePower(BASE_POWER_34, voltage);
        launcher.setPower(p1);
        shooterTimer.reset();
        shooterState = ShooterState.SPIN_UP;
    }



    private void updateShooter() {
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
                    // one shot just completed
                    double voltage = getVoltage();
                    double p1  = scalePower(BASE_POWER_1,  voltage);
                    double p2  = scalePower(BASE_POWER_2,  voltage);
                    double p34 = scalePower(BASE_POWER_34, voltage);
                    leftFeeder.setPower(0);
                    rightFeeder.setPower(0);

                    shotsInCycle++;                 // 1,2,3,...

                    // choose next launcher setpoint based on shot index
                    if (shotsInCycle == 1) {
                        firstShotDone = true;
                        launcher.setPower(p2);       // after first shot
                    } else if (shotsInCycle >= 2) {
                        launcher.setPower(p34);      // from second onward
                    }

                    if (continuousRequested) {
                        shooterState = ShooterState.INTERSHOT;
                        shooterTimer.reset();
                    } else {
                        if (--burstShots > 0) {
                            shooterState = ShooterState.INTERSHOT;
                            shooterTimer.reset();
                        } else {
                            // Immediately reverse for 300ms after the 4th shot
                            launcher.setPower(-1.0);
                            shooterTimer.reset();
                            shooterState = ShooterState.REVERSE_BRAKE;
                        }
                    }
                }
                break;

            case INTERSHOT:
                if (shooterTimer.milliseconds() >= INTERSHOT_MS) {
                    leftFeeder.setPower(SERVO_FEED_POWER);
                    rightFeeder.setPower(SERVO_FEED_POWER);
                    shooterState = ShooterState.FEED;
                    shooterTimer.reset();
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
                if (cooldownTimer.milliseconds() >= 3000) {
                    shooterState = ShooterState.IDLE;
                }
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
                setPathState(1); // settle then push
                break;
            case 1: // Run forward (start -> mid)

                if (follower.getCurrentTValue() >= 0.98 || !follower.isBusy()) {
                    if (!shotRequested) {
                        requestBurst4();      // fire AFTER finish is fully done
                        shotRequested = true; // guard
                    }
                    // move on once shooter finishes or is idle (e.g., if no shots)
                    if (shooterState == ShooterState.DONE
                            || shooterState == ShooterState.IDLE
                            || shooterState == ShooterState.COOLDOWN) {
                        setPathState(2);
                    }
                }

                break;
            case 2:
                follower.followPath(park,true);
                setPathState(3);
            case 3: // leg 1 active
                if (follower.getCurrentTValue() >= 0.98 || !follower.isBusy()) {
                    //leg1Finished = true;
                    setPathState(3); // jump straight to done
                }
                break;

            case 4: // all legs done
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
