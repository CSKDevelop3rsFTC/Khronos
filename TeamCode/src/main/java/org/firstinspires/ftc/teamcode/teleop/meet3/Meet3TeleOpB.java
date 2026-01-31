package org.firstinspires.ftc.teamcode.teleop.meet3;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;

import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.teleop.ShooterPID;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.bylazar.configurables.annotations.Configurable;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoImplEx;

@TeleOp(name="Meet3TeleOp - Blue", group="Robot")
@Disabled
public class Meet3TeleOpB extends LinearOpMode {

    public DcMotor frontLeftDrive = null;
    public DcMotor frontRightDrive = null;
    public DcMotor backLeftDrive = null;
    public DcMotor backRightDrive = null;

    public DcMotor intakeMotor = null;
    public DcMotor intakeMotor2 = null;
    public DcMotor turretMotor = null;

    public Servo kickerServo = null;
    public Servo stopperServo = null;

    public ServoImplEx hoodServo = null;

    public ServoImplEx leftLight = null;
    public ServoImplEx rightLight = null;
    public ServoImplEx top = null;

    public GoBildaPinpointDriver pinpoint;

    // --- SIMPLE TWO-PRESET SHOOT CONTROL ---
    private enum ShooterMode {OFF, CLOSE, FAR}
    private ShooterMode shooterMode = ShooterMode.OFF;

    private boolean prevLB = false;
    private boolean prevMid = false;

    // Field pose we will use everywhere (aim + distance + rpm)
    // Your convention: forward +X, left +Y
    private double fieldX = 0.0;          // inches
    private double fieldY = 0.0;          // inches
    private double fieldHeadingDeg = 0.0; // raw heading from Pinpoint (unknown sign convention)
    private double headingCCWDeg = 0.0;   // converted heading where CCW is positive (math standard)

    // Shooter PID subsystem (your existing class)
    private ShooterPID shooter;

    // Turret encoder conversions
    private static final double MOTOR_CPR = 537.6;
    private static final int GEAR_INPUT_TEETH = 63;
    private static final int GEAR_OUTPUT_TEETH = 190;
    private static final double GEAR_RATIO = (double) GEAR_OUTPUT_TEETH / GEAR_INPUT_TEETH;
    private static final double TICKS_PER_TURRET_REV = MOTOR_CPR * GEAR_RATIO;
    private static final double TICKS_PER_DEG = TICKS_PER_TURRET_REV / 360.0;
    private static final double DEG_PER_TICK = 360.0 / TICKS_PER_TURRET_REV;

    // ===== TRANSFER STATE MACHINE (simplified) =====
    private enum TransferState {
        IDLE,
        AIMING,
        FEEDING   // gate already open; run intake for a fixed time then close
    }

    private TransferState transferState = TransferState.IDLE;
    private long transferStateEndMs = 0;

    // Require turret to be inside deadband for a short time before advancing
    private long turretSettledSinceMs = 0;

    private final ElapsedTime ledBlinkTimer = new ElapsedTime();
    private boolean whiteOn = false;

    // Pick the one your DS5 actually reports.
    private boolean midButtonPressed() {
        return gamepad1.touchpad;
    }

    @Config
    public static class Goal {
        // Must be in SAME coordinate system as fieldX/fieldY
        // forward +X, left +Y
        public static double X = 72.0;
        public static double Y = 68.0 - 48; // 20
        public static double TURRET_MAX_POWER = 0.25;
    }

    @Config
    public static class AimOffsets {
        public static double SHOOT_OFFSET_DEG = 0.0;
    }

    @Config
    public static class HeadingConfig {
        // Set true if rotating the robot to the RIGHT makes Pinpoint heading INCREASE.
        // That is CW-positive. We convert to CCW-positive for atan2 math.
        public static boolean PINPOINT_HEADING_CW_POSITIVE = false;
    }

    @Configurable
    @Config
    public static class transferConstants {
        public static double intakePower = 1.0;

        // Gate positions (you already use these values in your code)
        public static double GATE_OPEN = 0.0;
        public static double GATE_CLOSED = 0.5;

        // Timing
        public static long aimTimeoutMs = 1500;     // max time to try aiming before just feeding anyway
        public static long settleMs = 120;          // require "aimed" for this long before feeding
        public static long feedMs = 300;            // how long intake runs after gate opens
    }

    private static double wrapDeg(double a) {
        while (a > 180) a -= 360;
        while (a < -180) a += 360;
        return a;
    }

    private static double angleToGoalDeg(double x, double y) {
        double dx = Goal.X - x;
        double dy = Goal.Y - y;
        return Math.toDegrees(Math.atan2(dy, dx)); // CCW-positive
    }

    private static double distanceToGoal(double x, double y) {
        double dx = Goal.X - x;
        double dy = Goal.Y - y;
        return Math.hypot(dx, dy);
    }

    private double distanceToGoal() {
        return distanceToGoal(fieldX, fieldY);
    }

    private double getTurretDegNow() {
        return turretMotor.getCurrentPosition() * DEG_PER_TICK;
    }

    private double getTurretCmdDegToGoal(double x, double y) {
        // fieldAim is CCW-positive
        double fieldAim = angleToGoalDeg(x, y);

        // headingCCWDeg is CCW-positive (converted from Pinpoint)
        // turret cmd is the relative angle turret must rotate (CCW-positive) to face the goal
        return wrapDeg((fieldAim - headingCCWDeg) + AimOffsets.SHOOT_OFFSET_DEG);
    }

    public void initialize() {
        frontLeftDrive = hardwareMap.get(DcMotor.class, "fl");
        frontRightDrive = hardwareMap.get(DcMotor.class, "fr");
        backLeftDrive = hardwareMap.get(DcMotor.class, "bl");
        backRightDrive = hardwareMap.get(DcMotor.class, "br");

        shooter = new ShooterPID(hardwareMap, "f1M", "f2M");
        shooter.setTelemetry(new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry()));

        intakeMotor = hardwareMap.get(DcMotor.class, "intake");
        intakeMotor2 = hardwareMap.get(DcMotor.class, "intake2");
        turretMotor = hardwareMap.get(DcMotor.class, "turret");

        kickerServo = hardwareMap.get(Servo.class, "kicker");
        stopperServo = hardwareMap.get(Servo.class, "stopper");

        hoodServo = hardwareMap.get(ServoImplEx.class, "hood");

        leftLight = hardwareMap.get(ServoImplEx.class, "left");
        rightLight = hardwareMap.get(ServoImplEx.class, "right");
        top = hardwareMap.get(ServoImplEx.class, "top");

        pinpoint = hardwareMap.get(GoBildaPinpointDriver.class, "pinpoint");

        frontLeftDrive.setDirection(DcMotorSimple.Direction.REVERSE);
        backLeftDrive.setDirection(DcMotorSimple.Direction.REVERSE);
        frontRightDrive.setDirection(DcMotorSimple.Direction.FORWARD);
        backRightDrive.setDirection(DcMotorSimple.Direction.FORWARD);

        leftLight.setPwmEnable();
        rightLight.setPwmEnable();

        kickerServo.setPosition(0.5);
        stopperServo.setPosition(transferConstants.GATE_CLOSED);

        turretMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        turretMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        turretMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        turretMotor.setDirection(DcMotorSimple.Direction.FORWARD);

        hoodServo.setDirection(Servo.Direction.REVERSE);

        // Pinpoint is odometry + heading source
        pinpoint.resetPosAndIMU();

        ledBlinkTimer.reset();
    }

    public void driveMovement(double driveSpeed) {
        double horizontal = -gamepad1.left_stick_x;
        double vertical = gamepad1.left_stick_y;
        double pivot = gamepad1.right_stick_x * 0.5;

        frontRightDrive.setPower(((vertical + horizontal) + pivot) * driveSpeed);
        frontLeftDrive.setPower(((vertical - horizontal) - pivot) * driveSpeed);
        backRightDrive.setPower(((vertical - horizontal) + pivot) * driveSpeed);
        backLeftDrive.setPower(((vertical + horizontal) - pivot) * driveSpeed);
    }

    public void driveController() {
        if (gamepad1.right_stick_button) driveMovement(0.5);
        else driveMovement(1.0);
    }

    public void intake() {
        // Manual intake control is disabled while transfer is active.
        if (transferActive()) return;

        if (gamepad1.right_trigger > 0) {
            intakeMotor.setPower(1);
        }
        else if (gamepad1.left_trigger > 0) {
            intakeMotor.setPower(-1);
        }
        else if (gamepad1.left_stick_y > 0) {
            intakeMotor.setPower(1);
        }

        else intakeMotor.setPower(0);
    }

    public void kicker() {
        if (gamepad1.x) kickerServo.setPosition(0);
        if (gamepad1.b) kickerServo.setPosition(0.5);
    }

    public void resetServo() {
        // left as-is, you can keep or remove
        if (gamepad2.x) hoodServo.setPosition(0.0);
        if (gamepad2.a) hoodServo.setPosition(0.5);
    }

    public void stopper() {
        // Manual gate control is disabled while transfer is active.
        if (transferActive()) return;

        if (gamepad1.y) stopperServo.setPosition(transferConstants.GATE_CLOSED);
        if (gamepad1.a) stopperServo.setPosition(transferConstants.GATE_OPEN);
    }

    public void shooterGamepad() {
        boolean lb = gamepad1.left_bumper;
        if (lb && !prevLB) {
            shooterMode = (shooterMode == ShooterMode.CLOSE) ? ShooterMode.OFF : ShooterMode.CLOSE;
        }
        prevLB = lb;

        boolean mid = midButtonPressed();
        if (mid && !prevMid) {
            shooterMode = (shooterMode == ShooterMode.FAR) ? ShooterMode.OFF : ShooterMode.FAR;
        }
        prevMid = mid;
    }

    public void shooterControl() {
        switch (shooterMode) {
            case CLOSE:
            case FAR:
                shooter.setTargetRPM(shooterCalc());
                break;
            case OFF:
            default:
                shooter.setTargetRPM(0.0);
                break;
        }
    }

    private boolean isShooterAtSpeed() {
        double target = shooter.getTargetRPM();
        if (target < 100) return false;
        return Math.abs(shooter.getCurrentRPM() - target) <= 25.0;
    }

    private void updateBlinkWhiteIfAtSpeed() {
        if (!isShooterAtSpeed()) return;
        if (ledBlinkTimer.milliseconds() >= 150) {
            whiteOn = !whiteOn;
            lightControl(whiteOn ? "white" : "null");
            ledBlinkTimer.reset();
        }
    }

    public void lightControl(String color) {
        if ("null".equalsIgnoreCase(color)) {
            leftLight.setPosition(0);
            rightLight.setPosition(0);
            top.setPosition(0);
        } else if ("red".equalsIgnoreCase(color)) {
            leftLight.setPosition(0.28);
            rightLight.setPosition(0.28);
            top.setPosition(0.28);
        } else if ("blue".equalsIgnoreCase(color)) {
            leftLight.setPosition(0.57);
            rightLight.setPosition(0.57);
            top.setPosition(0.57);
        } else if ("white".equalsIgnoreCase(color)) {
            leftLight.setPosition(1.0);
            rightLight.setPosition(1.0);
            top.setPosition(1.0);
        }
    }

    // ===== Bang-bang turret control (fixed power + deadband) =====
    private boolean driveTurretToDeg(double targetTurretDeg) {
        int ticksNow = turretMotor.getCurrentPosition();
        int targetTicks = (int) Math.round(targetTurretDeg * TICKS_PER_DEG);
        int errTicks = targetTicks - ticksNow;

        final int BAND_TICKS = 15;
        final int SLOW_ZONE_TICKS = 120;

        final double FAST_PWR = 0.20;
        final double SLOW_PWR = 0.08;

        if (Math.abs(errTicks) <= BAND_TICKS) {
            turretMotor.setPower(0.0);
            return true;
        }

        double dir = Math.signum(errTicks);
        double pwr = (Math.abs(errTicks) <= SLOW_ZONE_TICKS) ? SLOW_PWR : FAST_PWR;

        turretMotor.setPower(dir * pwr);
        return false;
    }

    private void holdTurretRobotZero() {
        driveTurretToDeg(0.0);
    }

    private boolean aimTurretAtGoal(double x, double y) {
        double turretCmdDeg = getTurretCmdDegToGoal(x, y);
        return driveTurretToDeg(turretCmdDeg);
    }

    private void startTransfer() {
        if (transferState != TransferState.IDLE) return;
        transferState = TransferState.AIMING;
        transferStateEndMs = System.currentTimeMillis() + transferConstants.aimTimeoutMs;
        turretSettledSinceMs = 0;

        // Make sure transfer starts from a known safe state.
        intakeMotor.setPower(0.0);
        intakeMotor2.setPower(0.0);
        stopperServo.setPosition(transferConstants.GATE_CLOSED);
    }

    private boolean transferActive() {
        return transferState != TransferState.IDLE;
    }

    private void updateTransfer() {
        if (transferState == TransferState.IDLE) return;

        shooter.update();

        switch (transferState) {

            case AIMING: {
                boolean aimed = aimTurretAtGoal(fieldX, fieldY);
                boolean timedOut = System.currentTimeMillis() >= transferStateEndMs;

                if (aimed) {
                    if (turretSettledSinceMs == 0) turretSettledSinceMs = System.currentTimeMillis();
                    if (System.currentTimeMillis() - turretSettledSinceMs >= transferConstants.settleMs) {
                        // Step 1: open gate, then start intake feed timer
                        stopperServo.setPosition(transferConstants.GATE_OPEN);
                        intakeMotor.setPower(transferConstants.intakePower);
                        intakeMotor2.setPower(transferConstants.intakePower);

                        transferState = TransferState.FEEDING;
                        transferStateEndMs = System.currentTimeMillis() + transferConstants.feedMs;
                    }
                } else {
                    turretSettledSinceMs = 0;
                }

                // Safety: if aim never settles, still feed after timeout
                if (timedOut) {
                    stopperServo.setPosition(transferConstants.GATE_OPEN);
                    intakeMotor2.setPower(transferConstants.intakePower);
                    intakeMotor.setPower(transferConstants.intakePower);
                    intakeMotor2.setPower(transferConstants.intakePower);

                    transferState = TransferState.FEEDING;
                    transferStateEndMs = System.currentTimeMillis() + transferConstants.feedMs;
                }
                break;
            }

            case FEEDING: {
                // Keep turret tracking while feeding if you want:
                aimTurretAtGoal(fieldX, fieldY);

                if (System.currentTimeMillis() >= transferStateEndMs) {
                    intakeMotor.setPower(0.0);
                    intakeMotor2.setPower(0.0);
                    stopperServo.setPosition(transferConstants.GATE_CLOSED);

                    transferState = TransferState.IDLE;
                }
                break;
            }

            default:
                intakeMotor.setPower(0.0);
                intakeMotor2.setPower(0.0);
                stopperServo.setPosition(transferConstants.GATE_CLOSED);
                transferState = TransferState.IDLE;
                break;
        }
    }

    public double shooterCalc() {
        double dist = distanceToGoal(fieldX, fieldY);
        if (dist >= 115) return (1.91456 * dist) + 466.82523;
        return (1.91456 * dist) + (466.82523 - 50);
    }

    @Override
    public void runOpMode() {
        initialize();
        waitForStart();
        lightControl("blue");

        while (opModeIsActive()) {
            driveController();

            // Pinpoint odometry + heading
            pinpoint.update();
            Pose2D p = pinpoint.getPosition();

            // forward +X, left +Y
            fieldX = p.getX(DistanceUnit.INCH);
            fieldY = -p.getY(DistanceUnit.INCH);

            // raw heading from pinpoint
            fieldHeadingDeg = p.getHeading(AngleUnit.DEGREES);

            // convert to CCW-positive for atan2 math
            headingCCWDeg = HeadingConfig.PINPOINT_HEADING_CW_POSITIVE ? -fieldHeadingDeg : fieldHeadingDeg;
            headingCCWDeg = wrapDeg(headingCCWDeg);

            double turretNowDeg = getTurretDegNow();
            double aimFieldDeg = angleToGoalDeg(fieldX, fieldY);
            double turretCmdDeg = getTurretCmdDegToGoal(fieldX, fieldY);

            telemetry.addData("Field", "x=%.2f y=%.2f hRaw=%.1f", fieldX, fieldY, fieldHeadingDeg);
            telemetry.addData("HeadingCCW", "%.1f", headingCCWDeg);
            telemetry.addData("Goal", "x=%.2f y=%.2f", Goal.X, Goal.Y);
            telemetry.addData("dX dY", "dx=%.2f dy=%.2f", (Goal.X - fieldX), (Goal.Y - fieldY));
            telemetry.addData("Aim Field", "%.1f deg", aimFieldDeg);
            telemetry.addData("Turret Now", "%.1f deg (ticks=%d)", turretNowDeg, turretMotor.getCurrentPosition());
            telemetry.addData("Turret Cmd", "%.1f deg", turretCmdDeg);
            telemetry.addData("Dist to goal (in)", "%.2f", distanceToGoal());
            telemetry.addData("Transfer", "%s", transferState);

            shooterGamepad();
            stopper();
            resetServo();
            kicker();
            intake();

            if (gamepad1.right_bumper) startTransfer();

            if (transferActive()) {
                updateTransfer();

                if (isShooterAtSpeed()) updateBlinkWhiteIfAtSpeed();
                else {
                    whiteOn = false;
                    lightControl("blue");
                    ledBlinkTimer.reset();
                }
            } else {
                holdTurretRobotZero();

                if (isShooterAtSpeed()) updateBlinkWhiteIfAtSpeed();
                else {
                    whiteOn = false;
                    lightControl("blue");
                    ledBlinkTimer.reset();
                }
            }

            if (!transferActive()) shooterControl();
            shooter.update();

            telemetry.update();
            sleep(20);
        }

        // Safety stop
        intakeMotor.setPower(0.0);
        intakeMotor2.setPower(0.0);
        stopperServo.setPosition(transferConstants.GATE_CLOSED);
        shooter.stop();
    }
}