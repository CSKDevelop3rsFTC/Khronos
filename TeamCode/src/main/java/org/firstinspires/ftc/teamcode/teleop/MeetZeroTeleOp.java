package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoImplEx;

@TeleOp(name="MeetZeroTeleOp", group="Robot")
public class MeetZeroTeleOp extends LinearOpMode {

    //drive train motors
    public DcMotor frontLeftDrive = null;
    public DcMotor frontRightDrive = null;
    public DcMotor backLeftDrive = null;
    public DcMotor backRightDrive = null;
    public CRServo leftFeeder = null;
    public CRServo rightFeeder = null;
    public DcMotorEx launcher = null;

    public double  mshooterSpeed = 0.73;
    public double  sshooterSpeed = -1;
    public long    shotSpaceTime = (long) (0.20 * 1000);
    public long    singleShotTime = (long) (0.85 * 1000);
    public long    chargeUpTime = (long) (1.6 * 1000);
    public boolean shootTimer = false;

    private enum ShooterState { IDLE, SPIN_UP, FEED, INTERSHOT, SPIN_DOWN, COOLDOWN }
    private ShooterState shooterState = ShooterState.IDLE;
    private com.qualcomm.robotcore.util.ElapsedTime shooterTimer =
            new com.qualcomm.robotcore.util.ElapsedTime();
    private com.qualcomm.robotcore.util.ElapsedTime cooldownTimer =
            new com.qualcomm.robotcore.util.ElapsedTime();

    // Edge detection and mode
    private boolean prevTriggerHeld = false;
    private boolean prevBumper = false;
    private boolean continuousRequested = false;  // left trigger hold
    private int burstShots = 0;                   // for 3-shot burst


    public void initialize() {

        frontLeftDrive = hardwareMap.get(DcMotor.class, "frontLeftDrive");
        frontRightDrive = hardwareMap.get(DcMotor.class, "frontRightDrive");
        backLeftDrive = hardwareMap.get(DcMotor.class, "backLeftDrive");
        backRightDrive = hardwareMap.get(DcMotor.class, "backRightDrive");


        frontLeftDrive.setDirection(DcMotorSimple.Direction.FORWARD);
        backLeftDrive.setDirection(DcMotorSimple.Direction.FORWARD);
        frontRightDrive.setDirection(DcMotorSimple.Direction.REVERSE);
        backRightDrive.setDirection(DcMotorSimple.Direction.REVERSE);

        // launcher + feeders
        launcher = hardwareMap.get(DcMotorEx.class, "launcher");
        leftFeeder = hardwareMap.get(CRServo.class, "left_feeder");
        rightFeeder = hardwareMap.get(CRServo.class, "right_feeder");

        rightFeeder.setDirection(DcMotorSimple.Direction.REVERSE);
        leftFeeder.setPower(0);
        rightFeeder.setPower(0);

    }

    public void driveMovement(double driveSpeed) {
        double vertical;
        double horizontal;
        double pivot;

        horizontal = gamepad1.left_stick_x;
        vertical = gamepad1.left_stick_y;
        pivot = -gamepad1.right_stick_x * 0.5;

        frontRightDrive.setPower(((vertical + horizontal) - pivot) * driveSpeed);
        frontLeftDrive.setPower(((vertical - horizontal) + pivot) * driveSpeed);
        backRightDrive.setPower(((vertical - horizontal) - pivot) * driveSpeed);
        backLeftDrive.setPower(((vertical + horizontal) + pivot) * driveSpeed);
    }

    public void driveController() {
        if (gamepad1.right_trigger > 0) {
            driveMovement(0.5);
        }
        else {
            driveMovement(1);
        }
    }

    public void shooter() {
        boolean triggerHeld = gamepad1.left_trigger > 0.15;
        boolean bumperPressed = gamepad1.left_bumper;

        if (triggerHeld && !prevTriggerHeld) {
            continuousRequested = true;
            if (shooterState == ShooterState.IDLE || shooterState == ShooterState.SPIN_DOWN) {
                launcher.setPower(mshooterSpeed);
                shooterState = ShooterState.SPIN_UP;
                shooterTimer.reset();
            }
        }
        if (!triggerHeld && prevTriggerHeld) {
            continuousRequested = false;
            if (shooterState == ShooterState.SPIN_UP ||
                    shooterState == ShooterState.FEED ||
                    shooterState == ShooterState.INTERSHOT) {
                shooterState = ShooterState.SPIN_DOWN;
                shooterTimer.reset();
            }
        }
        prevTriggerHeld = triggerHeld;

        if (bumperPressed && !prevBumper) {
            if (shooterState != ShooterState.COOLDOWN && shootTimer == false) {
                continuousRequested = false;
                burstShots = 3;
                launcher.setPower(mshooterSpeed);
                shooterState = ShooterState.SPIN_UP;
                shooterTimer.reset();
            }
        }
        prevBumper = bumperPressed;

        switch (shooterState) {
            case IDLE:
                leftFeeder.setPower(0);
                rightFeeder.setPower(0);
                launcher.setPower(0);
                break;

            case SPIN_UP:
                if (shooterTimer.milliseconds() >= chargeUpTime) {
                    leftFeeder.setPower(sshooterSpeed);
                    rightFeeder.setPower(sshooterSpeed);
                    shooterTimer.reset();
                    shooterState = ShooterState.FEED;
                }
                break;

            case FEED:
                if (shooterTimer.milliseconds() >= shotSpaceTime) {
                    leftFeeder.setPower(0);
                    rightFeeder.setPower(0);
                    shooterTimer.reset();
                    shooterState = ShooterState.INTERSHOT;

                    if (!continuousRequested && burstShots > 0) {
                        burstShots--;
                    }
                }
                break;

            case INTERSHOT:
                if (shooterTimer.milliseconds() >= singleShotTime) {
                    boolean doMoreContinuous = continuousRequested;
                    boolean doMoreBurst = (!continuousRequested && burstShots > 0);

                    if (doMoreContinuous || doMoreBurst) {
                        leftFeeder.setPower(sshooterSpeed);
                        rightFeeder.setPower(sshooterSpeed);
                        shooterTimer.reset();
                        shooterState = ShooterState.FEED;
                    } else {
                        shooterState = ShooterState.SPIN_DOWN;
                        shooterTimer.reset();
                    }
                }
                break;

            case SPIN_DOWN:
                launcher.setPower(0);
                leftFeeder.setPower(0);
                rightFeeder.setPower(0);

                if (continuousRequested) {
                    shooterState = ShooterState.IDLE;
                } else {
                    shootTimer = true;
                    cooldownTimer.reset();
                    shooterState = ShooterState.COOLDOWN;
                }
                break;

            case COOLDOWN:
                if (cooldownTimer.milliseconds() >= 3500) {
                    shootTimer = false;
                    shooterState = ShooterState.IDLE;
                }
                break;
        }

        if (gamepad1.y) {
            continuousRequested = false;
            burstShots = 0;
            launcher.setPower(0);
            leftFeeder.setPower(0);
            rightFeeder.setPower(0);
            shootTimer = false;
            shooterState = ShooterState.IDLE;
        }

    }

    public void shotCooldown() {
        shootTimer = true;
        cooldownTimer.reset();
        shooterState = ShooterState.COOLDOWN;
    }

    @Override
    public void runOpMode() {

        initialize();

        waitForStart();


        while (opModeIsActive()) {

            driveController();
            shooter();
            sleep(50);
        }

    }
}