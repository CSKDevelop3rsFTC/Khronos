package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.VoltageSensor;

@TeleOp(name="MeetZeroTeleOp", group="Robot")
public class MeetZeroTeleOp extends LinearOpMode {

    // Drive train motors
    private DcMotor frontLeftDrive, frontRightDrive, backLeftDrive, backRightDrive;
    private CRServo leftFeeder, rightFeeder;
    private DcMotorEx launcher;

    // Servo constants (unchanged)
    private static final double SERVO_FEED_POWER = -1.0;
    private static final long SHOT_SPACE_MS = (long)(0.3 * 1000);
    private static final long INTERSHOT_MS  = (long)(0.5 * 1000);

    // Launcher baseline values (at 12.3V)
    private static final double BASE_POWER_1  = 0.66; // first shot
    private static final double BASE_POWER_2  = 0.75; // second shot (tunable midpoint)
    private static final double BASE_POWER_34 = 0.73; // shots 3 and 4

    // Charge-up timing
    private static final long CHARGE_UP_MS = (long)(1.6 * 1000);

    // NEW: reverse brake after 4th shot
    private static final long POST_BURST_BRAKE_MS = 300;

    // State machine
    private enum ShooterState { IDLE, SPIN_UP, FEED, INTERSHOT, SPIN_DOWN, COOLDOWN, REVERSE_BRAKE }
    private ShooterState shooterState = ShooterState.IDLE;

    private com.qualcomm.robotcore.util.ElapsedTime shooterTimer =
            new com.qualcomm.robotcore.util.ElapsedTime();
    private com.qualcomm.robotcore.util.ElapsedTime cooldownTimer =
            new com.qualcomm.robotcore.util.ElapsedTime();

    private boolean prevTrigger = false;
    private boolean prevBumper = false;
    private boolean continuousRequested = false;
    private int burstShots = 0;

    private boolean firstShotDone = false;
    private int shotsInCycle = 0; // counts shots fired since spin-up (1,2,3,...)

    // Voltage scaling parameters
    private static final double NOMINAL_VOLTAGE = 12.3;
    private static final double VOLTAGE_SCALE = 0.06; // 10% change per ±0.5V deviation

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

    @Override
    public void runOpMode() {
        initialize();
        waitForStart();

        while (opModeIsActive()) {
            driveController();
            shooter();
            sleep(20);
        }
    }

    private void initialize() {
        frontLeftDrive  = hardwareMap.get(DcMotor.class, "frontLeftDrive");
        frontRightDrive = hardwareMap.get(DcMotor.class, "frontRightDrive");
        backLeftDrive   = hardwareMap.get(DcMotor.class, "backLeftDrive");
        backRightDrive  = hardwareMap.get(DcMotor.class, "backRightDrive");

        launcher    = hardwareMap.get(DcMotorEx.class, "launcher");
        leftFeeder  = hardwareMap.get(CRServo.class, "left_feeder");
        rightFeeder = hardwareMap.get(CRServo.class, "right_feeder");

        rightFeeder.setDirection(DcMotorSimple.Direction.REVERSE);

        frontLeftDrive.setDirection(DcMotorSimple.Direction.FORWARD);
        backLeftDrive.setDirection(DcMotorSimple.Direction.FORWARD);
        frontRightDrive.setDirection(DcMotorSimple.Direction.REVERSE);
        backRightDrive.setDirection(DcMotorSimple.Direction.REVERSE);

        leftFeeder.setPower(0);
        rightFeeder.setPower(0);
        launcher.setPower(0);
    }

    private void driveController() {
        double speed = (gamepad1.right_trigger > 0) ? 0.5 : 1.0;
        double x = gamepad1.left_stick_x;
        double y = gamepad1.left_stick_y;
        double pivot = -gamepad1.right_stick_x * 0.5;

        frontLeftDrive.setPower(((y - x) + pivot) * speed);
        backLeftDrive.setPower(((y + x) + pivot) * speed);
        frontRightDrive.setPower(((y + x) - pivot) * speed);
        backRightDrive.setPower(((y - x) - pivot) * speed);
    }

    private void shooter() {
        boolean trigger = gamepad1.left_trigger > 0.15;
        boolean bumper = gamepad1.left_bumper;
        double voltage = getVoltage();

        // Precompute scaled stage powers
        double p1  = scalePower(BASE_POWER_1,  voltage);
        double p2  = scalePower(BASE_POWER_2,  voltage);
        double p34 = scalePower(BASE_POWER_34, voltage);

        // Start/stop via trigger (continuous)
        if (trigger && !prevTrigger) {
            continuousRequested = true;
            firstShotDone = false;
            shotsInCycle = 0;
            launcher.setPower(p1);
            shooterState = ShooterState.SPIN_UP;
            shooterTimer.reset();
        }
        if (!trigger && prevTrigger) {
            continuousRequested = false;
            shooterState = ShooterState.SPIN_DOWN;
        }
        prevTrigger = trigger;

        // Left bumper: 4-shot burst
        if (bumper && !prevBumper) {
            if (shooterState != ShooterState.COOLDOWN) {
                continuousRequested = false;
                burstShots = 4;
                firstShotDone = false;
                shotsInCycle = 0;
                launcher.setPower(p1);
                shooterState = ShooterState.SPIN_UP;
                shooterTimer.reset();
            }
        }
        prevBumper = bumper;

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

        // Hard cancel
        if (gamepad1.y) {
            launcher.setPower(0);
            leftFeeder.setPower(0);
            rightFeeder.setPower(0);
            shooterState = ShooterState.IDLE;
            shotsInCycle = 0;
        }

        telemetry.addData("Voltage", "%.2f V", voltage);
        telemetry.addData("Launcher Pwr (p1,p2,p34)", "%.2f, %.2f, %.2f", p1, p2, p34);
        telemetry.addData("Shot#", shotsInCycle);
        telemetry.addData("State", shooterState);
        telemetry.update();
    }
}
