package org.firstinspires.ftc.teamcode.teleop;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.ColorRangeSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoImplEx;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.JavaUtil;
import org.firstinspires.ftc.teamcode.pedropathing.Constants;

@Config
@TeleOp(name = "MasterTeleOp - Blue (Safety)", group = "Pedro Pathing")
public class MasterTeleOpBlueSafety extends OpMode {

    private Follower follower;

    // Turret
    private CRServo turret;
    private CRServo turret2;

    // NOTE: original code used "intake2" motor as encoder source; keep that behavior
    private DcMotorEx turretEncoder;

    // Intake
    private DcMotorEx intake;
    private DcMotorEx intake2;

    // Shooter
    private ShooterPID shooter;

    // Transfer gate + hood
    private Servo stopper;
    private ServoImplEx hood;

    // Lights/indicators (as in your original)
    public ServoImplEx left = null;
    public ServoImplEx right = null;

    // Color sensors
    private NormalizedColorSensor lower;
    private NormalizedColorSensor upper;

    // =======================
    // Starting pose (unchanged)
    // =======================
    public static double START_X = 33;
    public static double START_Y = 20;
    public static double START_HEADING = Math.toRadians(145);

    // =======================
    // Encoder + gearing (unchanged)
    // =======================
    public static double ENCODER_CPR = 8192.0;
    public static double ENCODER_TURNS_PER_TURRET_TURN = 222.0 / 12.0;
    public static int ENCODER_SIGN = 1;

    public static double TURRET_SERVO_SIGN = -1.0;
    public static double TURRET2_SERVO_SIGN = -1.0;

    // =======================
    // Cable-safe window (UNWRAPPED degrees)
    // =======================
    public static double MIN_UNWRAPPED_DEG = -45.0;
    public static double MAX_UNWRAPPED_DEG = 315.0;
    public static double LIMIT_MARGIN_DEG = 2.0;
    public static boolean ALLOW_RECOVERY_WHEN_OUTSIDE_LIMITS = true;

    // =======================
    // Manual turret wrap-around control
    // =======================
    // Holding D-pad moves this target (wrapped -180..180); turret chooses a safe equivalent within cable window.
    private double manualTargetWrappedDeg = 0.0;
    private final ElapsedTime turretDt = new ElapsedTime();

    // How fast the target angle changes while holding D-pad
    public static double MANUAL_RATE_DEG_PER_SEC = 180.0;

    // Simple P controller to drive toward safe target
    public static double MANUAL_kP = 0.020;

    // Stop buzzing near target
    public static double MANUAL_DEADBAND_DEG = 1.5;

    // Power clamp for turret CRServos
    public static double TURRET_MAX_POWER = 0.45;

    // =======================
    // Manual shooter + hood setpoints (replaces equations)
    // =======================
    public static double MANUAL_CLOSE_RPM = 1300;
    public static double MANUAL_FAR_RPM = 1650.0;
    public static double MANUAL_HOOD_POS = 0.63; // 0..1

    // =======================
    // Intake sensor hold (kept from your version)
    // =======================
    private long lastUpperDetectMs = 0;
    private long lastLowerDetectMs = 0;

    // Light “both detected” hold
    private long lastBothDetectedMs = 0;

    // =======================
    // Transfer state machine (kept)
    // =======================
    private enum TransferState {IDLE, AIMING, PRE_FEED, FEEDING}
    private enum ShooterMode {OFF, CLOSE, FAR}
    private ShooterMode shooterMode = ShooterMode.OFF;

    private TransferState transferState = TransferState.IDLE;
    private long transferStateEndMs = 0;
    private long turretSettledSinceMs = 0;

    private boolean prevLB = false;
    private boolean prevMid = false;

    // Turret encoder zero offset
    private int zeroTicks = 0;

    @Config
    public static class transferConstants {
        public static double intakePower = -1.0;

        public static double GATE_OPEN = 0.3;
        public static double GATE_CLOSED = 0.1;

        public static long aimTimeoutMs = 1500;
        public static long settleMs = 120;
        public static long feedMs = 1230;
        public static long preMs = 200;

        // how long to keep “both detected” light on after both stop
        public static long bothHoldMs = 250;
    }

    // =======================
    // Lifecycle
    // =======================
    @Override
    public void init() {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(new Pose(START_X, START_Y, START_HEADING));
        follower.update();

        turret = hardwareMap.get(CRServo.class, "turret");
        turret2 = hardwareMap.get(CRServo.class, "turret2");

        intake = hardwareMap.get(DcMotorEx.class, "intake");
        intake2 = hardwareMap.get(DcMotorEx.class, "intake2");

        shooter = new ShooterPID(hardwareMap, "f1M", "f2M");
        shooter.setTelemetry(new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry()));

        // Use intake2 motor encoder for turret tracking (as in your original)
        turretEncoder = intake2;
        turretEncoder.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        int ticksNow = turretEncoder.getCurrentPosition();
        double ticksFromZeroWanted = 115591.3; // your existing calibration value
        zeroTicks = (int) Math.round(ticksNow - (ticksFromZeroWanted / ENCODER_SIGN));

        stopper = hardwareMap.get(Servo.class, "stopper");
        hood = hardwareMap.get(ServoImplEx.class, "hood");

        left = hardwareMap.get(ServoImplEx.class, "left");
        right = hardwareMap.get(ServoImplEx.class, "right");

        lower = hardwareMap.get(ColorRangeSensor.class, "lower");
        upper = hardwareMap.get(ColorRangeSensor.class, "upper");

        stopper.setPosition(transferConstants.GATE_CLOSED);
        setTurretPower(0.0);

        manualTargetWrappedDeg = wrapDeg(getCurrentTurretUnwrappedDeg());
        turretDt.reset();
    }

    @Override
    public void start() {
        follower.startTeleopDrive();
        follower.update();

        manualTargetWrappedDeg = wrapDeg(getCurrentTurretUnwrappedDeg());
        turretDt.reset();
    }

    @Override
    public void loop() {
        // Subsystems
        transferControl();
        shooterGamepad();

        // Manual hood (constant)
        if (hood != null) {
            hood.setPwmEnable();
            hood.setPosition(clamp(MANUAL_HOOD_POS, 0.0, 1.0));
        }

        // Shooter target control (constants)
        if (!transferActive()) shooterControl();
        shooter.update();

        // Intake runs only when not transferring (like your original)
        if (!transferActive()) intake();

        // Lights (unchanged behavior)
        lightControl();

        // Drive
        follower.setTeleOpDrive(
                -gamepad1.left_stick_y,
                -gamepad1.left_stick_x,
                -gamepad1.right_stick_x,
                true
        );
        follower.update();

        // Turret: D-pad changes wrapped target; turret chooses safe equivalent in cable window and drives to it
        turretManualWrapAround();

        // Re-zero turret offset (kept from your original); also re-sync target to avoid jumps
        if (gamepad1.x) {
            zeroTicks = turretEncoder.getCurrentPosition();
            manualTargetWrappedDeg = wrapDeg(getCurrentTurretUnwrappedDeg());
        }

        Pose p = follower.getPose();
        telemetry.addData("Pose (x,y)", "%.1f, %.1f", p.getX(), p.getY());
        telemetry.addData("ShooterMode", shooterMode);
        telemetry.addData("TargetRPM", "%.1f", shooter.getTargetRPM());
        telemetry.addData("HoodPos", "%.3f", clamp(MANUAL_HOOD_POS, 0.0, 1.0));
        telemetry.update();
    }

    @Override
    public void stop() {
        setTurretPower(0.0);
    }

    // =======================
    // Gamepad modes
    // =======================
    private boolean midButtonPressed() { return gamepad1.touchpad; }

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

    // =======================
    // Shooter control (constants)
    // =======================
    public void shooterControl() {
        switch (shooterMode) {
            case CLOSE:
                shooter.setTargetRPM(MANUAL_CLOSE_RPM);
                break;
            case FAR:
                shooter.setTargetRPM(MANUAL_FAR_RPM);
                break;
            case OFF:
            default:
                shooter.setTargetRPM(0.0);
                break;
        }
    }

    // =======================
    // Intake (kept from your version)
    // =======================
    public void intake() {
        String lowerColor = getColor(lower);
        String upperColor = getColor(upper);

        long now = System.currentTimeMillis();

        if (isGreenOrPurple(upperColor)) lastUpperDetectMs = now;
        if (isGreenOrPurple(lowerColor)) lastLowerDetectMs = now;

        boolean upperRecent = (now - lastUpperDetectMs) < 1000;
        // boolean lowerRecent = (now - lastLowerDetectMs) < 1000; // available if you want it later

        // Driver intent
        double p1 = 0.0; // intake
        double p2 = 0.0; // intake2

        if (gamepad1.left_trigger > 0) {
            p1 = transferConstants.intakePower;
            p2 = -transferConstants.intakePower;
        } else if (gamepad1.right_trigger > 0) {
            p1 = -transferConstants.intakePower;
            p2 = transferConstants.intakePower;
        }

        // Sensor hold: stop top stage if upper sees a ball recently (your original logic)
        if (upperRecent) {
            p2 = 0.0;
        }

        intake.setPower(p1);
        intake2.setPower(p2);
    }

    // =======================
    // Transfer internals (same structure you had)
    // =======================
    public void transferControl() {
        if (gamepad1.right_bumper) startTransfer();
        if (transferActive()) updateTransfer();
    }

    private void startTransfer() {
        if (transferState != TransferState.IDLE) return;

        transferState = TransferState.AIMING;
        transferStateEndMs = System.currentTimeMillis() + transferConstants.aimTimeoutMs;
        turretSettledSinceMs = 0;

        intake.setPower(0.0);
        intake2.setPower(0.0);
        stopper.setPosition(transferConstants.GATE_CLOSED);
    }

    private boolean transferActive() {
        return transferState != TransferState.IDLE;
    }

    private void updateTransfer() {
        if (transferState == TransferState.IDLE) return;

        switch (transferState) {
            case AIMING: {
                boolean aimed = true; // unchanged placeholder from your code
                boolean timedOut = System.currentTimeMillis() >= transferStateEndMs;

                if (aimed) {
                    if (turretSettledSinceMs == 0) turretSettledSinceMs = System.currentTimeMillis();
                    if (System.currentTimeMillis() - turretSettledSinceMs >= transferConstants.settleMs) {
                        stopper.setPosition(transferConstants.GATE_OPEN);
                        intake.setPower(0);
                        intake2.setPower(0);

                        transferState = TransferState.PRE_FEED;
                        transferStateEndMs = System.currentTimeMillis() + transferConstants.preMs;
                    }
                } else {
                    turretSettledSinceMs = 0;
                }

                if (timedOut) {
                    stopper.setPosition(transferConstants.GATE_OPEN);
                    intake.setPower(0);
                    intake2.setPower(0);

                    transferState = TransferState.PRE_FEED;
                    transferStateEndMs = System.currentTimeMillis() + transferConstants.preMs;
                }
                break;
            }

            case PRE_FEED: {
                stopper.setPosition(transferConstants.GATE_OPEN);
                intake.setPower(0);
                intake2.setPower(0);

                if (System.currentTimeMillis() >= transferStateEndMs) {
                    transferState = TransferState.FEEDING;
                    transferStateEndMs = System.currentTimeMillis() + transferConstants.feedMs;
                }
                break;
            }

            case FEEDING: {
                intake.setPower(1.0);
                intake2.setPower(-1.0);

                if (System.currentTimeMillis() >= transferStateEndMs) {
                    stopper.setPosition(transferConstants.GATE_CLOSED);
                    intake.setPower(0.0);
                    intake2.setPower(0.0);
                    transferState = TransferState.IDLE;
                }
                break;
            }

            default:
                intake.setPower(0.0);
                intake2.setPower(0.0);
                stopper.setPosition(transferConstants.GATE_CLOSED);
                transferState = TransferState.IDLE;
                break;
        }
    }

    // =======================
    // Lights (kept from your original style)
    // =======================
    public void lightControl() {
        String lowerColor = getColor(lower);
        String upperColor = getColor(upper);

        long now = System.currentTimeMillis();

        boolean bothDetected = isGreenOrPurple(upperColor) && isGreenOrPurple(lowerColor);

        if (bothDetected) {
            lastBothDetectedMs = now;
        }

        boolean bothHoldActive = lastBothDetectedMs != 0 && (now - lastBothDetectedMs) < transferConstants.bothHoldMs;

        if (bothHoldActive) {
            left.setPosition(1.0);
            right.setPosition(1.0);
        } else {
            left.setPosition(0.0);
            right.setPosition(0.0);
        }
    }

    // =======================
    // Turret: D-pad wrap-around target + cable-window safe equivalent
    // =======================
    private void turretManualWrapAround() {
        double currentUnwrappedDeg = getCurrentTurretUnwrappedDeg();

        double dt = turretDt.seconds();
        turretDt.reset();
        if (dt <= 0) dt = 1e-3;

        int dir = 0;
        if (gamepad2.dpad_right && !gamepad2.dpad_left) dir = +1;
        else if (gamepad2.dpad_left && !gamepad2.dpad_right) dir = -1;

        if (dir != 0) {
            manualTargetWrappedDeg = wrapDeg(manualTargetWrappedDeg + dir * MANUAL_RATE_DEG_PER_SEC * dt);
        }

        double safeTargetUnwrappedDeg = chooseTargetWithinCableWindow(manualTargetWrappedDeg, currentUnwrappedDeg);

        boolean outsideHard = (currentUnwrappedDeg < MIN_UNWRAPPED_DEG) || (currentUnwrappedDeg > MAX_UNWRAPPED_DEG);
        if (outsideHard && !ALLOW_RECOVERY_WHEN_OUTSIDE_LIMITS) {
            setTurretPower(0.0);
            return;
        }

        double errorDeg = safeTargetUnwrappedDeg - currentUnwrappedDeg;

        if (Math.abs(errorDeg) <= MANUAL_DEADBAND_DEG) {
            setTurretPower(0.0);
            return;
        }

        double out = MANUAL_kP * errorDeg;
        out = clamp(out, -TURRET_MAX_POWER, TURRET_MAX_POWER);

        // Extra safety: if outside hard window, only allow motion back inside
        if (outsideHard) {
            if (currentUnwrappedDeg < MIN_UNWRAPPED_DEG && out < 0) out = 0.0;
            if (currentUnwrappedDeg > MAX_UNWRAPPED_DEG && out > 0) out = 0.0;
        }

        setTurretPower(out);

        telemetry.addData("TurretCur(unwrapped)", "%.1f", currentUnwrappedDeg);
        telemetry.addData("TurretTarget(wrapped)", "%.1f", manualTargetWrappedDeg);
        telemetry.addData("TurretTargetSAFE(unwrapped)", "%.1f", safeTargetUnwrappedDeg);
        telemetry.addData("TurretOut", "%.2f", out);
    }

    private double getCurrentTurretUnwrappedDeg() {
        int ticks = turretEncoder.getCurrentPosition();
        int ticksFromZero = (ticks - zeroTicks) * ENCODER_SIGN;
        return turretDegFromTicksUnwrapped(ticksFromZero);
    }

    private void setTurretPower(double ccwPositivePower) {
        turret.setPower(ccwPositivePower * TURRET_SERVO_SIGN);
        turret2.setPower(ccwPositivePower * TURRET2_SERVO_SIGN);
    }

    private double turretDegFromTicksUnwrapped(int ticksFromZero) {
        double ticksPerTurretRev = ENCODER_CPR * ENCODER_TURNS_PER_TURRET_TURN;
        return (ticksFromZero / ticksPerTurretRev) * 360.0;
    }

    private double chooseTargetWithinCableWindow(double targetWrappedDeg, double currentUnwrappedDeg) {
        double minEff = MIN_UNWRAPPED_DEG + LIMIT_MARGIN_DEG;
        double maxEff = MAX_UNWRAPPED_DEG - LIMIT_MARGIN_DEG;

        double c0 = targetWrappedDeg;
        double cPlus = targetWrappedDeg + 360.0;
        double cMinus = targetWrappedDeg - 360.0;

        boolean in0 = (c0 >= minEff && c0 <= maxEff);
        boolean inPlus = (cPlus >= minEff && cPlus <= maxEff);
        boolean inMinus = (cMinus >= minEff && cMinus <= maxEff);

        double best = Double.NaN;
        double bestDist = Double.POSITIVE_INFINITY;

        if (in0) {
            double d = Math.abs(c0 - currentUnwrappedDeg);
            if (d < bestDist) { bestDist = d; best = c0; }
        }
        if (inPlus) {
            double d = Math.abs(cPlus - currentUnwrappedDeg);
            if (d < bestDist) { bestDist = d; best = cPlus; }
        }
        if (inMinus) {
            double d = Math.abs(cMinus - currentUnwrappedDeg);
            if (d < bestDist) { bestDist = d; best = cMinus; }
        }

        if (!Double.isNaN(best)) {
            return best;
        }

        double near = nearestEquivalentOfAngle(currentUnwrappedDeg, targetWrappedDeg);
        return clamp(near, minEff, maxEff);
    }

    private double nearestEquivalentOfAngle(double referenceUnwrapped, double angleWrapped) {
        double k = Math.round((referenceUnwrapped - angleWrapped) / 360.0);
        return angleWrapped + 360.0 * k;
    }

    private double clamp(double v, double lo, double hi) {
        if (v < lo) return lo;
        if (v > hi) return hi;
        return v;
    }

    private double wrapDeg(double deg) {
        deg %= 360.0;
        if (deg >= 180.0) deg -= 360.0;
        if (deg < -180.0) deg += 360.0;
        return deg;
    }

    // =======================
    // Color helpers (same as your original)
    // =======================
    public String getColor(NormalizedColorSensor sensor) {
        NormalizedRGBA color = sensor.getNormalizedColors();
        double hue = JavaUtil.colorToHue(color.toColor());

        if (hue < 30) return "red";
        else if (hue < 60) return "orange";
        else if (hue < 90) return "yellow";
        else if (hue < 150) return "green";
        else if (hue < 225) return "green";
        else if (hue < 350) return "purple";
        else return "red";
    }

    private boolean isGreenOrPurple(String c) {
        return "green".equals(c) || "purple".equals(c);
    }
}