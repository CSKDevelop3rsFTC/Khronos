package org.firstinspires.ftc.teamcode.teleop.test;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.pedropathing.Constants;

@Config
@TeleOp(name = "Turret Tracking Constant", group = "Pedro Pathing")
public class angleTest extends OpMode {

    private Follower follower;

    // Two CR servos that rotate the turret
    private CRServo turret;
    private CRServo turret2;

    // REV Through Bore encoder (configured as a motor port)
    private DcMotorEx turretEncoder;

    // =======================
    // Dashboard-tunable knobs
    // =======================

    // Target point on the field (inches)
    public static double TARGET_X = 70;
    public static double TARGET_Y = 63;

    // If your Pose heading is weirdly scaled (example: 90 deg shows up as 0.9),
    // flip this on and set HEADING_SCALE = 100.
    public static boolean USE_HEADING_SCALE = false;
    public static double HEADING_SCALE = 100.0;

    // Starting pose for Pedro follower
    public static double START_X = 0;
    public static double START_Y = 48;
    public static double START_HEADING = 0; // radians (normal Pedro Pose)

    // Encoder constants
    public static double ENCODER_CPR = 8192.0;

    // Encoder turns per 1 turret turn.
    // Example: 61T (encoder) driving 201T (turret) => 201/61.
    // Your old code used 201/33, keep whatever matches your real gears.
    public static double ENCODER_TURNS_PER_TURRET_TURN = 222.0 / 12.0;

    // Flip if encoder direction is backwards
    public static int ENCODER_SIGN = 1; // 1 or -1

    // Servo output signs (because the two CR servos often need opposite power)
    public static double TURRET_SERVO_SIGN = -1.0;
    public static double TURRET2_SERVO_SIGN = -1.0;

    // Bang-bang tuning
    public static double FAST_POWER = 1;     // big move
    public static double SLOW_POWER = 0.30;     // near target
    public static double DEADBAND_DEG = 4.5;    // stop here
    public static double SLOW_BAND_DEG = 6.5;   // within this, use SLOW_POWER
    public static boolean ENABLE_TURRET = true;

    // =======================
    // Cable-safe window (UNWRAPPED degrees)
    // =======================
    // You described the tight spot as the same physical orientation from both sides:
    // -142.5 deg and 217.5 deg are equivalent, but wrap makes it display as -142.5 after 180.
    // So we keep an unwrapped turret angle and constrain it to this window.
    public static double MIN_UNWRAPPED_DEG = -142.5;
    public static double MAX_UNWRAPPED_DEG = 217.5;

    // Keep a little buffer so small overshoot does not kiss the cable-tight point.
    public static double LIMIT_MARGIN_DEG = 2.0;

    // If the turret drifts outside the window (bad zero, overshoot), allow motion back inside.
    public static boolean ALLOW_RECOVERY_WHEN_OUTSIDE_LIMITS = true;

    // =======================
    // Internal state
    // =======================
    private int zeroTicks = 0;

    @Override
    public void init() {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(new Pose(START_X, START_Y, START_HEADING));
        follower.update();

        turret = hardwareMap.get(CRServo.class, "turret");
        turret2 = hardwareMap.get(CRServo.class, "turret2");

        turretEncoder = hardwareMap.get(DcMotorEx.class, "intake2");
        turretEncoder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        turretEncoder.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        zeroTicks = 0;
        setTurretPower(0);
    }

    @Override
    public void start() {
        follower.startTeleopDrive();
        follower.update();
    }

    @Override
    public void loop() {
        // TeleOp drive
        follower.setTeleOpDrive(
                -gamepad1.left_stick_y,
                gamepad1.left_stick_x,
                -gamepad1.right_stick_x,
                true
        );
        follower.update();

        // Zero turret when physically straight ahead
        if (gamepad1.y) {
            zeroTicks = turretEncoder.getCurrentPosition();
        }

        // Robot pose
        Pose p = follower.getPose();

        // Desired turret angle relative to robot forward (wrapped deg, CCW+)
        double desiredTurretWrappedDeg = desiredTurretDegToPoint(p, TARGET_X, TARGET_Y);

        // Current turret angle from encoder:
        // - Unwrapped is what we use for cable logic and error
        // - Wrapped is only for display
        int ticks = turretEncoder.getCurrentPosition();
        int ticksFromZero = (ticks - zeroTicks) * ENCODER_SIGN;

        double currentTurretUnwrappedDeg = turretDegFromTicksUnwrapped(ticksFromZero);
        double currentTurretWrappedDeg = wrapDeg(currentTurretUnwrappedDeg);

        // Pick a target representation (target, target+360, target-360) that stays in the safe window.
        double safeTargetUnwrappedDeg = chooseTargetWithinCableWindow(
                desiredTurretWrappedDeg,
                currentTurretUnwrappedDeg
        );

        // Error in unwrapped space
        double errorDeg = safeTargetUnwrappedDeg - currentTurretUnwrappedDeg;

        // Drive the turret servos
        if (ENABLE_TURRET) {
            applyBangBang(errorDeg, currentTurretUnwrappedDeg);
        } else {
            setTurretPower(0);
        }

        telemetry.addData("Pose (x,y)", "%.1f, %.1f", p.getX(), p.getY());
        telemetry.addData("Robot Heading", "%.1f deg", getRobotHeadingDeg(p));

        telemetry.addData("Turret Now (wrap)", "%.1f", currentTurretWrappedDeg);
        telemetry.addData("Turret Now (raw)", "%.1f", currentTurretUnwrappedDeg);

        telemetry.addData("Turret Target (wrap)", "%.1f", desiredTurretWrappedDeg);
        telemetry.addData("Turret Target (raw)", "%.1f", safeTargetUnwrappedDeg);

        telemetry.addData("Error (deg)", "%.1f", errorDeg);

        telemetry.addData("Cable Window", "[%.1f, %.1f]", MIN_UNWRAPPED_DEG, MAX_UNWRAPPED_DEG);
        telemetry.addData("Margin", "%.1f deg", LIMIT_MARGIN_DEG);
        telemetry.addLine("Y = zero turret");
        telemetry.update();
    }

    @Override
    public void stop() {
        setTurretPower(0);
    }

    private void applyBangBang(double errorDeg, double currentTurretUnwrappedDeg) {
        double minEff = MIN_UNWRAPPED_DEG + LIMIT_MARGIN_DEG;
        double maxEff = MAX_UNWRAPPED_DEG - LIMIT_MARGIN_DEG;

        boolean outsideHard = (currentTurretUnwrappedDeg < MIN_UNWRAPPED_DEG) || (currentTurretUnwrappedDeg > MAX_UNWRAPPED_DEG);
        boolean outsideEff = (currentTurretUnwrappedDeg < minEff) || (currentTurretUnwrappedDeg > maxEff);

        if (outsideHard) {
            if (!ALLOW_RECOVERY_WHEN_OUTSIDE_LIMITS) {
                setTurretPower(0);
                return;
            }

            // Recovery only: move back toward the window.
            // If we're too low, only allow increasing (CCW+).
            if (currentTurretUnwrappedDeg < MIN_UNWRAPPED_DEG && errorDeg <= 0) {
                setTurretPower(0);
                return;
            }
            // If we're too high, only allow decreasing (CW).
            if (currentTurretUnwrappedDeg > MAX_UNWRAPPED_DEG && errorDeg >= 0) {
                setTurretPower(0);
                return;
            }
        }

        // If we're inside hard window but hugging the margin zone, still allow motion,
        // just be aware that overshoot could hit the cut. The safe target chooser already
        // tries to keep us away from the cut by LIMIT_MARGIN_DEG.
        double absErr = Math.abs(errorDeg);
        if (absErr <= DEADBAND_DEG) {
            setTurretPower(0);
            return;
        }

        double power = (absErr <= SLOW_BAND_DEG) ? SLOW_POWER : FAST_POWER;
        double dir = Math.signum(errorDeg); // CCW+ is positive

        // Extra guard: if we're outside effective window and trying to move deeper out, block it.
        if (outsideEff) {
            if (currentTurretUnwrappedDeg < minEff && dir < 0) {
                setTurretPower(0);
                return;
            }
            if (currentTurretUnwrappedDeg > maxEff && dir > 0) {
                setTurretPower(0);
                return;
            }
        }

        setTurretPower(dir * power);
    }

    private void setTurretPower(double ccwPositivePower) {
        turret.setPower(ccwPositivePower * TURRET_SERVO_SIGN);
        turret2.setPower(ccwPositivePower * TURRET2_SERVO_SIGN);
    }

    private double desiredTurretDegToPoint(Pose robotPose, double targetX, double targetY) {
        double dx = targetX - robotPose.getX();
        double dy = targetY - robotPose.getY();

        // Field bearing: CCW+, 0 deg = +X
        double targetBearingDeg = Math.toDegrees(Math.atan2(dy, dx));

        // Robot heading in degrees, CCW+
        double robotHeadingDeg = getRobotHeadingDeg(robotPose);

        // Desired turret angle relative to robot forward: CCW+
        return wrapDeg(targetBearingDeg - robotHeadingDeg);
    }

    private double getRobotHeadingDeg(Pose robotPose) {
        double h = robotPose.getHeading();
        if (USE_HEADING_SCALE) {
            // If heading is stored like "degrees / 100", set HEADING_SCALE=100.
            return wrapDeg(h * HEADING_SCALE);
        }
        // Normal Pedro: heading is radians
        return wrapDeg(Math.toDegrees(h));
    }

    // Unwrapped turret angle in degrees from encoder ticks.
    // This can grow past 180 and up to ~217.5 in your case, which is exactly what we need for cable logic.
    private double turretDegFromTicksUnwrapped(int ticksFromZero) {
        double ticksPerTurretRev = ENCODER_CPR * ENCODER_TURNS_PER_TURRET_TURN;
        return (ticksFromZero / ticksPerTurretRev) * 360.0;
    }

    // Chooses the unwrapped target angle that:
    // 1) Represents the same physical direction as targetWrappedDeg, and
    // 2) Lies inside the cable-safe unwrapped window, and
    // 3) Requires the smallest motion from currentUnwrappedDeg.
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

        // Fallback: take the equivalent of the target closest to current, then clamp into the effective window.
        double near = nearestEquivalentOfAngle(currentUnwrappedDeg, targetWrappedDeg);
        return clamp(near, minEff, maxEff);
    }

    // Returns angleWrapped + 360*k that is closest to referenceUnwrapped.
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
}