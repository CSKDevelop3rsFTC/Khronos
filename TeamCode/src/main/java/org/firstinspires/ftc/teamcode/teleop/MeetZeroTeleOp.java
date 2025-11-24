package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.CRServo;

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

    // --- Shooter params (same behavior as before) ---
    public double  mshooterSpeed = 0.63;                 // initial launcher power
    public double  sshooterSpeed = -1;
    public long    shotSpaceTime = (long) (0.23 * 1000); // feeder ON per shot
    public long    singleShotTime = (long) (0.24 * 1000);// recovery BEFORE 2nd shot

    // --- NEW: post-first-shot boost and shorter recovery for subsequent shots ---
    public double  postFirstPower = 0.86;                // boost power after 1st shot
    public long    singleShotTimeAfterFirst = (long)(0.2 * 1000); // shorter recovery after 1st
    private boolean firstShotDone = false;

    public long    chargeUpTime = (long) (1.6 * 1000);
    public boolean shootTimer = false;

    // NEW: extra-shot wait time and flag
    private static final long FOURTH_DELAY_MS = 200;     // 1 second pause before 4th
    private static final long BRAKE_MS = 400;             // 100 ms reverse brake
    private boolean delayedFourthEnabled = false;         // only for 3-shot burst

    private enum ShooterState {
        IDLE, SPIN_UP, FEED, INTERSHOT, SPIN_DOWN, COOLDOWN,
        // NEW states
        DELAY_BEFORE_FOURTH,
        POST_FOURTH_BRAKE
    }
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
        frontLeftDrive  = hardwareMap.get(DcMotor.class, "frontLeftDrive");
        frontRightDrive = hardwareMap.get(DcMotor.class, "frontRightDrive");
        backLeftDrive   = hardwareMap.get(DcMotor.class, "backLeftDrive");
        backRightDrive  = hardwareMap.get(DcMotor.class, "backRightDrive");

        frontLeftDrive.setDirection(DcMotorSimple.Direction.FORWARD);
        backLeftDrive.setDirection(DcMotorSimple.Direction.FORWARD);
        frontRightDrive.setDirection(DcMotorSimple.Direction.REVERSE);
        backRightDrive.setDirection(DcMotorSimple.Direction.REVERSE);

        launcher    = hardwareMap.get(DcMotorEx.class, "launcher");
        leftFeeder  = hardwareMap.get(CRServo.class, "left_feeder");
        rightFeeder = hardwareMap.get(CRServo.class, "right_feeder");

        rightFeeder.setDirection(DcMotorSimple.Direction.REVERSE);
        leftFeeder.setPower(0);
        rightFeeder.setPower(0);
    }

    public void driveMovement(double driveSpeed) {
        double horizontal = gamepad1.left_stick_x;
        double vertical   = gamepad1.left_stick_y;
        double pivot      = -gamepad1.right_stick_x * 0.5;

        frontRightDrive.setPower(((vertical + horizontal) - pivot) * driveSpeed);
        frontLeftDrive.setPower(((vertical - horizontal) + pivot) * driveSpeed);
        backRightDrive.setPower(((vertical - horizontal) - pivot) * driveSpeed);
        backLeftDrive.setPower(((vertical + horizontal) + pivot) * driveSpeed);
    }

    public void driveController() {
        if (gamepad1.right_trigger > 0) driveMovement(0.5);
        else                             driveMovement(1.0);
    }

    public void shooter() {
        boolean triggerHeld   = gamepad1.left_trigger > 0.15;
        boolean bumperPressed = gamepad1.left_bumper;

        // Trigger: start/stop continuous
        if (triggerHeld && !prevTriggerHeld) {
            continuousRequested = true;
            if (shooterState == ShooterState.IDLE || shooterState == ShooterState.SPIN_DOWN) {
                firstShotDone = false;                      // reset boost flag
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

        // Bumper: 3-shot burst (+ delayed 4th after 1s)
        if (bumperPressed && !prevBumper) {
            if (shooterState != ShooterState.COOLDOWN && !shootTimer) {
                continuousRequested = false;
                burstShots = 3;
                delayedFourthEnabled = true;               // arm the delayed 4th
                firstShotDone = false;                      // reset boost flag
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
                    // Shot just completed
                    leftFeeder.setPower(0);
                    rightFeeder.setPower(0);

                    // After the FIRST completed shot, rapidly boost to higher power
                    if (!firstShotDone) {
                        firstShotDone = true;
                        launcher.setPower(postFirstPower);   // immediate ramp
                    }

                    // If we're still in a burst (first three), decrement and proceed
                    if (!continuousRequested && burstShots > 0) {
                        burstShots--;
                        shooterTimer.reset();
                        shooterState = ShooterState.INTERSHOT;
                        break;
                    }

                    // If this was the delayed 4th shot: apply a brief reverse brake
                    if (!continuousRequested && !delayedFourthEnabled && burstShots == 0 && firstShotDone) {
                        launcher.setPower(-0.01);            // tiny reverse to decelerate
                        shooterTimer.reset();
                        shooterState = ShooterState.POST_FOURTH_BRAKE;
                        break;
                    }

                    // Otherwise continue normal intershot handling
                    shooterTimer.reset();
                    shooterState = ShooterState.INTERSHOT;
                }
                break;

            case INTERSHOT:
                // Use shorter recovery AFTER first shot
                long needed = firstShotDone ? singleShotTimeAfterFirst : singleShotTime;

                if (shooterTimer.milliseconds() >= needed) {
                    boolean doMoreContinuous = continuousRequested;
                    boolean doMoreBurst      = (!continuousRequested && burstShots > 0);

                    if (doMoreContinuous || doMoreBurst) {
                        leftFeeder.setPower(sshooterSpeed);
                        rightFeeder.setPower(sshooterSpeed);
                        shooterTimer.reset();
                        shooterState = ShooterState.FEED;
                    } else {
                        // No more burst shots, but we owe a delayed fourth
                        if (!continuousRequested && delayedFourthEnabled) {
                            shooterState = ShooterState.DELAY_BEFORE_FOURTH;
                            shooterTimer.reset();
                        } else {
                            shooterState = ShooterState.SPIN_DOWN;
                            shooterTimer.reset();
                        }
                    }
                }
                break;

            // Wait 1 second, then fire a single final shot
            case DELAY_BEFORE_FOURTH:
                // keep launcher spinning at current power during the pause
                if (shooterTimer.milliseconds() >= FOURTH_DELAY_MS) {
                    // feed one last time
                    leftFeeder.setPower(sshooterSpeed);
                    rightFeeder.setPower(sshooterSpeed);
                    delayedFourthEnabled = false; // only once
                    shooterTimer.reset();
                    shooterState = ShooterState.FEED;
                }
                break;

            // Apply −0.01 power for 100 ms to stop spin before next cycle
            case POST_FOURTH_BRAKE:
                if (shooterTimer.milliseconds() >= BRAKE_MS) {
                    launcher.setPower(0);
                    leftFeeder.setPower(0);
                    rightFeeder.setPower(0);
                    shooterState = ShooterState.SPIN_DOWN;
                    shooterTimer.reset();
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

        // Hard cancel
        if (gamepad1.y) {
            continuousRequested = false;
            burstShots = 0;
            firstShotDone = false;
            delayedFourthEnabled = false;
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
