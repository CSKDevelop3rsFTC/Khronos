package org.firstinspires.ftc.teamcode.paths;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;
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
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@Autonomous(name = "samplePath", group = "Autonomous")
public class samplePath extends OpMode {

    // Pedro core
    private Follower follower;
    private Timer pathTimer;
    private int pathState;

    // Shooter hardware (names match your TeleOp)
    private CRServo leftFeeder, rightFeeder;
    private DcMotorEx launcher;

    // Shooter timings from your teleop
    private final double  mshooterSpeed   = 0.73;     // launcher power
    private final double  sshooterSpeed   = -1.0;     // feeder power
    private final long    shotSpaceTimeMs = 200;      // feeder ON
    private final long    singleShotTimeMs= 850;      // feeder OFF
    private final long    chargeUpTimeMs  = 1600;     // spin-up

    // Shooter FSM
    private enum ShooterState { IDLE, SPIN_UP, FEED, INTERSHOT, SPIN_DOWN, DONE }
    private ShooterState shooterState = ShooterState.IDLE;
    private int shotsRemaining = 0;
    private final ElapsedTime shooterTimer = new ElapsedTime();

    // Field poses (positions only; heading will be computed from the segment)
    private final Pose startPose = new Pose(88, 8, 0);         // heading ignored here
    private final Pose endPose   = new Pose(110, 127, 0);      // heading ignored here

    private PathChain path;

    // === Shooter helpers ===
    private void requestBurst3() {
        if (shooterState == ShooterState.IDLE || shooterState == ShooterState.DONE) {
            shotsRemaining = 3;
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
                    leftFeeder.setPower(sshooterSpeed);
                    rightFeeder.setPower(sshooterSpeed);
                    shooterTimer.reset();
                    shooterState = ShooterState.FEED;
                }
                break;

            case FEED:
                if (shooterTimer.milliseconds() >= shotSpaceTimeMs) {
                    leftFeeder.setPower(0);
                    rightFeeder.setPower(0);
                    shotsRemaining = Math.max(0, shotsRemaining - 1);
                    shooterTimer.reset();
                    shooterState = ShooterState.INTERSHOT;
                }
                break;

            case INTERSHOT:
                if (shooterTimer.milliseconds() >= singleShotTimeMs) {
                    if (shotsRemaining > 0) {
                        leftFeeder.setPower(sshooterSpeed);
                        rightFeeder.setPower(sshooterSpeed);
                        shooterTimer.reset();
                        shooterState = ShooterState.FEED;
                    } else {
                        shooterState = ShooterState.SPIN_DOWN;
                    }
                }
                break;

            case SPIN_DOWN:
                launcher.setPower(0);
                shooterState = ShooterState.DONE;
                break;
        }
    }

    // Compute the straight-line heading from start to end (radians)
    private double segmentHeading() {
        double dx = endPose.getX() - startPose.getX();
        double dy = endPose.getY() - startPose.getY();
        return Math.atan2(dy, dx);
    }

    // === Pedro path ===
    public void buildPath() {
        double h = segmentHeading();

        // Build a straight path where heading points along the line the whole time
        Pose startWithH = new Pose(startPose.getX(), startPose.getY(), h);
        Pose endWithH   = new Pose(endPose.getX(),   endPose.getY(),   h);

        path = follower.pathBuilder()
                .addPath(new BezierLine(startWithH, endWithH))
                .setLinearHeadingInterpolation(h, h)   // constant along-line heading
                .addParametricCallback(1.0, this::requestBurst3)
                .build();
    }

    public void setPathState(int pState) {
        pathState = pState;
        pathTimer.resetTimer();
    }

    public void pathUpdate() {
        switch (pathState) {
            case 0:
                follower.followPath(path, true);
                setPathState(1);
                break;

            case 1:
                if (follower.getCurrentTValue() >= 0.98 || !follower.isBusy()) {
                    setPathState(-1);
                }
                break;

            default:
                break;
        }
    }

    // === OpMode lifecycle ===
    @Override
    public void init() {
        pathTimer = new Timer();

        follower = Constants.createFollower(hardwareMap);
        // Start pose uses computed heading so odometry and path agree
        follower.setStartingPose(new Pose(startPose.getX(), startPose.getY(), segmentHeading()));

        // Shooter HW
        launcher    = hardwareMap.get(DcMotorEx.class, "launcher");
        leftFeeder  = hardwareMap.get(CRServo.class, "left_feeder");
        rightFeeder = hardwareMap.get(CRServo.class, "right_feeder");

        rightFeeder.setDirection(DcMotorSimple.Direction.REVERSE);
        leftFeeder.setPower(0);
        rightFeeder.setPower(0);

        launcher.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        launcher.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        launcher.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER,
                new PIDFCoefficients(300, 0, 0, 10));

        buildPath();
    }

    @Override
    public void start() {
        setPathState(0);
    }

    @Override
    public void loop() {
        follower.update();
        pathUpdate();
        updateShooter();

        telemetry.addData("path state", pathState);
        telemetry.addData("t", "%.2f", follower.getCurrentTValue());
        telemetry.addData("pose", "(%.1f, %.1f)", follower.getPose().getX(), follower.getPose().getY());
        telemetry.addData("heading rad", "%.3f", follower.getPose().getHeading());
        telemetry.addData("heading deg", "%.1f", Math.toDegrees(follower.getPose().getHeading()));
        telemetry.addData("shooter", shooterState);
        telemetry.update();
    }
}
