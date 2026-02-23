package org.firstinspires.ftc.teamcode.auto;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.pedropathing.paths.PathConstraints;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
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
import org.firstinspires.ftc.teamcode.teleop.ShooterPID;

import java.util.ArrayList;
import java.util.List;

/**
 * blue18 (long auto, turret kept simple)
 *
 * Updates in THIS version:
 *  - Removed "must be at target RPM" gating before feeding (no WAIT_SPEED step).
 *  - Constant shot settings: RPM=950, Hood=0.7 for ALL shots.
 *  - FTC-safe enum at class scope.
 *  - Turret: now uses TeleOp-style field-point tracking of the goal (TARGET_X/Y),
 *    using Pedro odometry.
 *  - Turret encoder mapping matches TeleOp: "intake2".
 */
@Autonomous(name = "red18", group = "Autonomous")
public class red18 extends OpMode {

    // =========================
    // FTC-safe shoot state enum
    // =========================
    private enum ShootState { SETTLE, PRE, FEED, DONE }

    // =========================
    // Shooter settings (constant)
    // =========================
    private static final double AUTO_RPM = 1670;
    private static final double AUTO_HOOD = 0.63; // servo pos 0..1

    private static final double SECOND_RPM = 1685;
    private static final double SECOND_HOOD = 0.63; // servo pos 0..1

    private static final double THIRD_SHOT_RPM = 1750;
    private static final double THIRD_SHOT_HOOD = 0.63;

    private static final double FOURTH_SHOT_RPM = 1625;
    private static final double FOURTH_SHOT_HOOD = 0.65;

    private static final double FIFTH_SHOT_RPM = 1750;
    private static final double FIFTH_SHOT_HOOD = 0.63;

    private static final double LAST_SHOT_RPM = 1575;
    private static final double LAST_SHOT_HOOD = 0.60;

    // =========================
    // Turret + field tracking settings
    // =========================
    // Field goal location on the field (same concept as TeleOp, can be tuned)
    public static double TARGET_X = 126.5;
    public static double TARGET_Y = -60.5;

    public static double LAST_TARGET_X = 130.25;
    public static double LAST_TARGET_Y = -63.0;

    private enum ShotType {NORMAL, SECOND, THIRD, FOURTH, FIFTH, LAST}

    private boolean useLastShotTarget = false;

    public static boolean USE_HEADING_SCALE = false;
    public static double HEADING_SCALE = 100.0;

    // Turret encoder + gearing
    public static double ENCODER_CPR = 8192.0;
    public static double ENCODER_TURNS_PER_TURRET_TURN = 222.0 / 12.0; // from TeleOp
    public static int ENCODER_SIGN = 1;

    public static double TURRET_SERVO_SIGN = -1.0;
    public static double TURRET2_SERVO_SIGN = -1.0;

    // Angle (deg) we want the turret to *logically* be at when auto starts.
    // Put the robot down, leave the turret where you like, and this tells the
    // code what angle that position should be treated as.
    public static double TURRET_START_DEG = -179.5;

    // Mechanical aiming offset (deg). Shifts where the turret points relative to ideal field angle.
    public static double TURRET_AIM_OFFSET_DEG = 0.0;

    // Cable-safe window (UNWRAPPED degrees) — copied from TeleOp
    public static double MIN_UNWRAPPED_DEG = -45.0;
    public static double MAX_UNWRAPPED_DEG = 315.0;
    public static double LIMIT_MARGIN_DEG = 2.0;
    public static boolean ALLOW_RECOVERY_WHEN_OUTSIDE_LIMITS = true;

    // Legacy simple move settings (kept but effectively unused)
    public static int TURRET_MOVE_DIR = +1;
    public static double TURRET_MOVE_DEG = 45.0;
    public static double TURRET_MOVE_POWER = 0.35;
    public static double TURRET_HOLD_POWER = 0.0;

    // ---- TeleOp-style PIDF knobs
    public static boolean ENABLE_PID = true;
    public static double kP = 0.018;
    public static double kI = 0.0;
    public static double kD = 0.0;
    public static double kF = 0.08;
    public static boolean USE_F = true;

    public static double MAX_OUTPUT = 1.0;
    public static double MIN_OUTPUT = 0.0;
    public static double DEADBAND_DEG = 2.0;
    public static double OUTPUT_SLEW_PER_SEC = 0.0;

    // ---- TeleOp-style PID internal state
    private double integral = 0.0;
    private double prevError = 0.0;
    private double lastOutput = 0.0;
    private final ElapsedTime pidTimer = new ElapsedTime();

    // encoder zero (same concept as TeleOp's zeroTicks)
    private int turretZeroTicks = 0;

    // target we want to hold (legacy hold, no longer used for tracking)
    private double turretHoldTargetDeg = 0.0;
    private boolean turretHoldEnabled = false;

    // =========================
    // Transfer timing (TeleOp-style)
    // =========================
    public static class transferConstants {
        public static double intakePower = 1.0;

        public static double GATE_OPEN = 0.3;
        public static double GATE_CLOSED = 0.1;

        public static long settleMs = 50;
        public static long preMs = 100;
        public static long feedMs = 700;
    }

    private static final double GATE_WAIT_SEC = 0.9;

    // =========================
    // Pedro + paths
    // =========================
    private Follower follower;

    private PathChain preload;

    private PathChain rankingPoint;

    private PathChain moveOut;

    private PathChain toRow2Pre, intakeRow2, intakeSafe2, toShootFromRow2;
    private PathChain shootToGate, gateIntake, gateToShoot, gateShoot1;

    private PathChain toRow1Pre, intakeRow1, toShootFromRow1;
    private PathChain toRow3Pre, intakeRow3, intakeSafe3, toShootFromRow3, rotationEnd;

    private static final PathConstraints SLOW = new PathConstraints(
            18,
            33,
            3.0,
            4.0
    );

    // =========================
    // Hardware
    // =========================
    private DcMotor intake;
    private DcMotorEx intake2;          // also turret encoder (TeleOp-style)
    private DcMotorEx turretEncoder;    // points to intake2

    private NormalizedColorSensor lower;
    private NormalizedColorSensor upper;

    private Servo stopper;
    private ServoImplEx hood;
    private ShooterPID shooter;

    private CRServo turret;
    private CRServo turret2;

    // =========================
    // Intake intent flags
    // =========================
    private boolean intakeNow = false;
    private boolean reverseIntake = false;

    private long lastUpperDetectMs = 0;
    private long lastLowerDetectMs = 0;

    // =========================
    // Step queue framework
    // =========================
    interface Step {
        void init();
        void update();
        boolean isFinished();
    }

    static class StepRunner {
        private final List<Step> steps;
        private int idx = 0;
        private boolean inited = false;

        StepRunner(List<Step> steps) { this.steps = steps; }

        void run() {
            if (idx >= steps.size()) return;
            Step s = steps.get(idx);
            if (!inited) { s.init(); inited = true; }
            s.update();
            if (s.isFinished()) { idx++; inited = false; }
        }

        void reset() {
            idx = 0;
            inited = false;
        }

        boolean isDone() { return idx >= steps.size(); }
        int getIndex() { return idx; }
    }

    private final List<Step> steps = new ArrayList<>();
    private StepRunner runner;

    // =========================
    // Turret legacy one-time move state
    // (kept, but startTurretRotate45 becomes a no-op so it doesn't fight tracking)
    // =========================
    private boolean turretMoved = false;
    private int turretMoveStartTicks = 0;
    private int turretMoveTargetDeltaTicks = 0;

    private boolean transferOverride = false;

    // =========================
    // Shooter priming
    // =========================
    private boolean shooterPrimed = false;

    // =========================
    // Poses
    // =========================
    private final Pose startPose = new Pose(112, -54.5, Math.toRadians(170));


    private final Pose shoot0    = new Pose(76, -16.3, Math.toRadians(240));
    private final Pose shoot     = new Pose(76, -16.3, Math.toRadians(240));
    private final Pose shoot1     = new Pose(84.5, -22.5, Math.toRadians(270));
    private final Pose preintakeOne = new Pose(57, -24, Math.toRadians(260));
    private final Pose intakePose1  = new Pose(55, -53, Math.toRadians(270));
    private final Pose intakesafe1 = new Pose(55,-54.5, Math.toRadians(270));

    private final Pose rotate = new Pose(55.5, -52, Math.toRadians(300));
    private final Pose gate   = new Pose(59.5, -60.5, Math.toRadians(295));
    private final Pose gate1   = new Pose(59.4, -60.5, Math.toRadians(306));

    //private final Pose backitup = new Pose(59.3, 59, Math.toRadians(44));


    private final Pose preintake  = new Pose(83, -24, Math.toRadians(270));
    private final Pose intakePose = new Pose(84.5, -44, Math.toRadians(270));

    private final Pose preintakeTwo = new Pose(27, -19, Math.toRadians(270));
    private final Pose intakePose2  = new Pose(32, -55, Math.toRadians(270));
    private final Pose preEnd = new Pose(32, -52, Math.toRadians(215));
    private final Pose parkPose = new Pose(110, -17.5, Math.toRadians(135));

    // =========================
    // Build paths
    // =========================
    private void buildPaths() {

        preload = follower.pathBuilder()
                .addPath(new BezierLine(startPose, shoot0))
                .setLinearHeadingInterpolation(startPose.getHeading(), shoot0.getHeading())
                .build();

        // 2nd row
        toRow2Pre = follower.pathBuilder()
                .addPath(new BezierCurve(shoot0, preintakeOne, intakePose1))
                .setLinearHeadingInterpolation(shoot0.getHeading(), preintakeOne.getHeading(), intakePose1.getHeading())
                .build();
        intakeSafe2 = follower.pathBuilder()
                .addPath(new BezierLine(intakePose1, intakesafe1))
                .setLinearHeadingInterpolation(intakePose1.getHeading(), intakesafe1.getHeading())
                .build();
        toShootFromRow2 = follower.pathBuilder()
                .addPath(new BezierLine(intakesafe1, shoot))
                .setLinearHeadingInterpolation(intakesafe1.getHeading(), shoot.getHeading())
                .build();
        // gate loop
        shootToGate = follower.pathBuilder()
                .addPath(new BezierCurve(shoot, rotate, gate))
                .setLinearHeadingInterpolation(shoot.getHeading(), rotate.getHeading(), gate.getHeading())
                .setConstraints(SLOW)
                .build();
        gateIntake = follower.pathBuilder()
                .addPath(new BezierLine(gate, gate1))
                .setLinearHeadingInterpolation(gate.getHeading(), gate1.getHeading())
                .build();
        gateShoot1 = follower.pathBuilder()
                .addPath(new BezierCurve(gate, rotate, shoot))
                .setLinearHeadingInterpolation(gate.getHeading(), shoot.getHeading(), shoot.getHeading())
                .build();
        gateToShoot = follower.pathBuilder()
                .addPath(new BezierCurve(gate, rotate, shoot1))
                .setLinearHeadingInterpolation(gate.getHeading(), shoot1.getHeading(), shoot1.getHeading())
                .build();
        // 1st row
        toRow1Pre = follower.pathBuilder()
                .addPath(new BezierLine(shoot1, intakePose))
                .setLinearHeadingInterpolation(shoot1.getHeading(), intakePose.getHeading())
                .build();

        intakeRow1 = follower.pathBuilder()
                .addPath(new BezierLine(preintake, intakePose))
                .setLinearHeadingInterpolation(preintake.getHeading(), intakePose.getHeading())
                .build();

        toShootFromRow1 = follower.pathBuilder()
                .addPath(new BezierLine(intakePose, shoot))
                .setLinearHeadingInterpolation(intakePose.getHeading(), shoot.getHeading())
                .build();

        // 3rd row
        toRow3Pre = follower.pathBuilder()
                .addPath(new BezierCurve(shoot, preintakeTwo, intakePose2))
                .setLinearHeadingInterpolation(shoot.getHeading(), preintakeTwo.getHeading(), intakePose2.getHeading())
                .build();

        rotationEnd = follower.pathBuilder()
                .addPath(new BezierLine(intakePose2, preEnd))
                .setLinearHeadingInterpolation(intakePose2.getHeading(), preEnd.getHeading())
                .build();
        toShootFromRow3 = follower.pathBuilder()
                .addPath(new BezierLine(preEnd, parkPose))
                .setLinearHeadingInterpolation(preEnd.getHeading(), parkPose.getHeading())
                .build();

    }

    // =========================
    // Step helpers
    // =========================
    private Step follow(PathChain path, boolean intakeOn) {
        return new Step() {
            @Override public void init() {
                intakeNow = intakeOn;
                reverseIntake = false;
                follower.followPath(path, true);
            }
            @Override public void update() { }
            @Override public boolean isFinished() { return !follower.isBusy(); }
        };
    }

    private Step waitSec(double sec) {
        return new Step() {
            final ElapsedTime t = new ElapsedTime();
            @Override public void init() { t.reset(); }
            @Override public void update() { }
            @Override public boolean isFinished() { return t.seconds() >= sec; }
        };
    }

    private Step followWithMaxPower(PathChain path, boolean intakeOn, double maxPower) {
        return new Step() {
            boolean restored = false;

            @Override public void init() {
                intakeNow = intakeOn;
                reverseIntake = false;
                try { follower.setMaxPower(maxPower); } catch (Exception ignored) { }
                follower.followPath(path, true);
            }

            @Override public void update() { }

            @Override public boolean isFinished() {
                boolean done = !follower.isBusy();
                if (done && !restored) {
                    restored = true;
                    try { follower.setMaxPower(1.0); } catch (Exception ignored) { }
                }
                return done;
            }
        };
    }

    // =========================
    // SHOOT STEP (TeleOp-style transfer sequence)
    // NOTE: NO "wait for RPM" gating.
    // =========================
    private Step shootSequenceStep(ShotType shotType) {
        return new Step() {

            ShootState st;
            long endMs;

            @Override
            public void init() {
                intakeNow = true;
                reverseIntake = false;

                transferOverride = true;

                double targetRpm = AUTO_RPM;
                double targetHoodPos = AUTO_HOOD;

                // Choose RPM / hood based on whether this is the last shot
                switch (shotType) {
                    case SECOND:
                        targetRpm = SECOND_RPM;
                        targetHoodPos = SECOND_HOOD;
                        break;

                    case THIRD:
                        targetRpm = THIRD_SHOT_RPM;
                        targetHoodPos = THIRD_SHOT_HOOD;
                        break;

                    case FOURTH:
                        targetRpm = FOURTH_SHOT_RPM;
                        targetHoodPos = FOURTH_SHOT_HOOD;
                        break;

                    case FIFTH:
                        targetRpm = FIFTH_SHOT_RPM;
                        targetHoodPos = FIFTH_SHOT_HOOD;
                        break;

                    case LAST:
                        targetRpm = LAST_SHOT_RPM;
                        targetHoodPos = LAST_SHOT_HOOD;
                        useLastShotTarget = true;
                        break;

                    case NORMAL:
                    default:
                        break;
                }

                shooter.setTargetRPM(targetRpm);
                if (red18.this.hood != null) {
                    red18.this.hood.setPosition(targetHoodPos);
                }

                stopper.setPosition(transferConstants.GATE_CLOSED);
                intake.setPower(0.0);
                intake2.setPower(0.0);

                st = ShootState.SETTLE;
                endMs = System.currentTimeMillis() + transferConstants.settleMs;
            }

            @Override
            public void update() {
                long now = System.currentTimeMillis();

                switch (st) {
                    case SETTLE:
                        if (now >= endMs) {
                            stopper.setPosition(transferConstants.GATE_OPEN);
                            st = ShootState.PRE;
                            endMs = now + transferConstants.preMs;
                        }
                        break;

                    case PRE:
                        intake.setPower(transferConstants.intakePower);
                        intake2.setPower(-transferConstants.intakePower);

                        if (now >= endMs) {
                            st = ShootState.FEED;
                            endMs = now + transferConstants.feedMs;
                        }
                        break;

                    case FEED:
                        intake.setPower(transferConstants.intakePower);
                        intake2.setPower(-transferConstants.intakePower);
                        if (now >= endMs) {
                            stopper.setPosition(transferConstants.GATE_CLOSED);
                            intake.setPower(0.0);
                            intake2.setPower(0.0);
                            transferOverride = false;
                            st = ShootState.DONE;
                            intakeNow = false;
                            reverseIntake = false;
                        }
                        break;

                    case DONE:
                    default:
                        break;
                }
            }

            @Override
            public boolean isFinished() {
                return st == ShootState.DONE;
            }
        };
    }

    // =========================
    // Legacy Non-blocking turret rotate + hold (no-op now)
    // =========================
    private boolean turretRotating = false;

    private void startTurretRotate45() {
        // NO-OP for now to avoid fighting turret tracking.
        turretRotating = false;
        turretMoved = true;
    }

    private void updateTurretRotate45() {
        // NO-OP: we don't use the legacy rotate-45 behavior anymore.
    }

    // =========================
    // TeleOp-style PIDF
    // =========================
    private double pidfStep(double errorDeg, double dt) {
        if (Math.abs(errorDeg) <= DEADBAND_DEG) {
            integral = 0.0;
            prevError = errorDeg;
            lastOutput = 0.0;
            return 0.0;
        }

        integral += errorDeg * dt;
        integral = clamp(integral, -200.0, 200.0);

        double derivative = (errorDeg - prevError) / dt;
        prevError = errorDeg;

        double ff = 0.0;
        if (USE_F) ff = kF * Math.signum(errorDeg);

        double out = (kP * errorDeg) + (kI * integral) + (kD * derivative) + ff;

        if (MIN_OUTPUT > 0.0) {
            double s = Math.signum(out);
            if (Math.abs(out) < MIN_OUTPUT) out = s * MIN_OUTPUT;
        }

        out = clamp(out, -MAX_OUTPUT, MAX_OUTPUT);

        if (OUTPUT_SLEW_PER_SEC > 0.0) {
            double maxDelta = OUTPUT_SLEW_PER_SEC * dt;
            out = clamp(out, lastOutput - maxDelta, lastOutput + maxDelta);
        }

        lastOutput = out;
        return out;
    }

    private double turretDegFromTicksUnwrapped(int ticksFromZero) {
        double ticksPerTurretRev = ENCODER_CPR * ENCODER_TURNS_PER_TURRET_TURN;
        return (ticksFromZero / ticksPerTurretRev) * 360.0;
    }

    // deg -> ticks helper for calibration
    private int degToTicks(double deg) {
        double ticksPerTurretRev = ENCODER_CPR * ENCODER_TURNS_PER_TURRET_TURN;
        return (int) Math.round((deg / 360.0) * ticksPerTurretRev);
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

    private double nearestEquivalentOfAngle(double referenceUnwrapped, double angleWrapped) {
        double k = Math.round((referenceUnwrapped - angleWrapped) / 360.0);
        return angleWrapped + 360.0 * k;
    }

    private double getRobotHeadingDeg(Pose robotPose) {
        double h = robotPose.getHeading();
        if (USE_HEADING_SCALE) {
            return wrapDeg(h * HEADING_SCALE);
        }
        return wrapDeg(Math.toDegrees(h));
    }

    private double desiredTurretDegToPoint(Pose robotPose, double targetX, double targetY) {
        double dx = targetX - robotPose.getX();
        double dy = targetY - robotPose.getY();

        double targetBearingDeg = Math.toDegrees(Math.atan2(dy, dx));
        double robotHeadingDeg = getRobotHeadingDeg(robotPose);

        return wrapDeg(targetBearingDeg - robotHeadingDeg);
    }

    // === NEW: cable-window logic copied from TeleOp ===
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

    private void updateTurretHold() {
        if (!turretHoldEnabled) return;

        int ticks = turretEncoder.getCurrentPosition();
        int ticksFromZero = (ticks - turretZeroTicks);
        double currentDeg = turretDegFromTicksUnwrapped(ticksFromZero);

        double errorDeg = turretHoldTargetDeg - currentDeg;

        double dt = pidTimer.seconds();
        pidTimer.reset();
        if (dt <= 0) dt = 1e-3;

        double output = 0.0;
        if (ENABLE_PID) output = pidfStep(errorDeg, dt);

        setTurretPower(output);
    }

    // =========================
    // TeleOp-style constant turret tracking to field goal (no AprilTag)
    // =========================
    private void updateTurretTracking() {
        if (follower == null || turretEncoder == null) return;

        Pose p = follower.getPose();

        // Current turret position (unwrapped)
        int ticks = turretEncoder.getCurrentPosition();
        int ticksFromZero = (ticks - turretZeroTicks) * ENCODER_SIGN;
        double currentTurretUnwrappedDeg = turretDegFromTicksUnwrapped(ticksFromZero);

        double tx = useLastShotTarget ? LAST_TARGET_X : TARGET_X;
        double ty = useLastShotTarget ? LAST_TARGET_Y : TARGET_Y;

        // Desired turret angle to look at field goal, plus mechanical offset
        double desiredTurretWrappedDeg = desiredTurretDegToPoint(p, tx, ty);
        desiredTurretWrappedDeg = wrapDeg(desiredTurretWrappedDeg + TURRET_AIM_OFFSET_DEG);

        // NO cable limits: just choose the nearest 360° equivalent
        double safeTargetUnwrappedDeg = nearestEquivalentOfAngle(
                currentTurretUnwrappedDeg,
                desiredTurretWrappedDeg
        );

        double errorDeg = safeTargetUnwrappedDeg - currentTurretUnwrappedDeg;

        double dt = pidTimer.seconds();
        pidTimer.reset();
        if (dt <= 0) dt = 1e-3;

        double output;

        if (ENABLE_PID) {
            output = pidfStep(errorDeg, dt);
        } else {
            output = 0.0;
            integral = 0.0;
            prevError = 0.0;
            lastOutput = 0.0;
        }

        setTurretPower(output);
        telemetry.addData("TurretCur", currentTurretUnwrappedDeg);
        telemetry.addData("TurretTarget", safeTargetUnwrappedDeg);
        telemetry.addData("TurretErr", errorDeg);
        telemetry.addData("TurretOut", output);

    }

    // =========================
    // Build auton steps (LONG)
    // =========================
    private void buildAutonSteps() {
        steps.clear();


        steps.add(follow(preload, false));
        steps.add(waitSec(0.2));
        steps.add(shootSequenceStep(ShotType.NORMAL));

        // 2nd row: drive, intake, return -> SHOOT
        steps.add(follow(toRow2Pre, true));
        steps.add(follow(intakeSafe2, true));
        steps.add(follow(toShootFromRow2, false));
        steps.add(waitSec(0.2));
        steps.add(shootSequenceStep(ShotType.SECOND));
// gate sequence 1 (farther shooting)
        steps.add(followWithMaxPower(shootToGate, false, 0.9));
        steps.add(waitSec(0.02));
        steps.add(follow(gateIntake, true));
        steps.add(waitSec(GATE_WAIT_SEC));
        steps.add(follow(gateShoot1, false));
        steps.add(waitSec(0.2));
        steps.add(shootSequenceStep(ShotType.THIRD));

        //gate sequence 2
        steps.add(followWithMaxPower(shootToGate, false, 0.9));
        steps.add(waitSec(0.02));
        steps.add(follow(gateIntake, true));
        steps.add(waitSec(0.85));
        steps.add(follow(gateToShoot, false));
        steps.add(waitSec(0.2));
        steps.add(shootSequenceStep(ShotType.FOURTH));

        // 1st row: intake -> SHOOT
        steps.add(follow(toRow1Pre, true));
        steps.add(follow(toShootFromRow1, false));
        steps.add(waitSec(0.2));
        steps.add(shootSequenceStep(ShotType.FIFTH));

        // 3rd row: intake -> park waypoint -> SHOOT
        steps.add(follow(toRow3Pre, true));
        steps.add(follow(rotationEnd, false));
        steps.add(follow(toShootFromRow3, false));

        steps.add(waitSec(0.35));
        steps.add(shootSequenceStep(ShotType.LAST));
    }

    // =========================
    // FTC lifecycle
    // =========================
    @Override
    public void init() {
        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(startPose);

        buildPaths();

        intake = hardwareMap.get(DcMotor.class, "intake");

        // Turret encoder mapping matches TeleOp: "intake2"
        intake2 = hardwareMap.get(DcMotorEx.class, "intake2");
        turretEncoder = intake2;

        lower = hardwareMap.get(ColorRangeSensor.class, "lower");
        upper = hardwareMap.get(ColorRangeSensor.class, "upper");

        stopper = hardwareMap.get(Servo.class, "stopper");
        hood = hardwareMap.get(ServoImplEx.class, "hood");

        turret = hardwareMap.get(CRServo.class, "turret");
        turret2 = hardwareMap.get(CRServo.class, "turret2");

        shooter = new ShooterPID(hardwareMap, "f1M", "f2M");

        intake.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        intake2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        turretEncoder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        turretEncoder.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // Calibrate so that the physical turret angle at init is TURRET_START_DEG.
        int rawTicks = turretEncoder.getCurrentPosition(); // after reset, usually 0
        int offsetTicks = degToTicks(TURRET_START_DEG) * ENCODER_SIGN;
        // When ticks == rawTicks, turretDegFromTicksUnwrapped(...) == TURRET_START_DEG * ENCODER_SIGN
        turretZeroTicks = rawTicks - offsetTicks;

        stopper.setPosition(transferConstants.GATE_CLOSED);
        intake.setPower(0.0);
        intake2.setPower(0.0);
        setTurretPower(0.0);

        turretMoved = false;
        turretRotating = false;
        turretHoldEnabled = false;

        // prime shooter immediately
        shooter.setTargetRPM(AUTO_RPM);
        if (hood != null) hood.setPosition(AUTO_HOOD);
        shooterPrimed = true;

        pidTimer.reset();

        buildAutonSteps();
        runner = new StepRunner(steps);
    }

    @Override
    public void start() {
        if (runner != null) runner.reset();

        shooter.setTargetRPM(AUTO_RPM);
        if (hood != null) hood.setPosition(AUTO_HOOD);
        shooterPrimed = true;

        pidTimer.reset();
    }

    @Override
    public void loop() {
        follower.update();

        // turret tracking every loop based on Pedro odometry (TeleOp-style, field-point only)
        updateTurretTracking();

        // Legacy helpers (now effectively no-op, since rotate45 is disabled
        // and turretHoldEnabled is false)
        updateTurretRotate45();
        updateTurretHold();

        if (!shooterPrimed) {
            shooter.setTargetRPM(AUTO_RPM);
            if (hood != null) hood.setPosition(AUTO_HOOD);
            shooterPrimed = true;
        }
        shooter.update();

        if (!transferOverride) {
            autoIntakeControl(intakeNow, reverseIntake);
        }

        runner.run();

        if (runner.isDone() && !follower.isBusy()) {
            requestOpModeStop();
        }

        telemetry.update();
    }

    @Override
    public void stop() {
        intake.setPower(0.0);
        intake2.setPower(0.0);
        setTurretPower(0.0);
        if (stopper != null) stopper.setPosition(transferConstants.GATE_CLOSED);
        if (shooter != null) shooter.setTargetRPM(0.0);
    }

    // =========================
    // Turret power helper
    // =========================
    private void setTurretPower(double ccwPositivePower) {
        if (turret != null)  turret.setPower(ccwPositivePower * TURRET_SERVO_SIGN);
        if (turret2 != null) turret2.setPower(ccwPositivePower * TURRET2_SERVO_SIGN);
    }

    // =========================
    // Color-sensor intake logic
    // =========================
    private void autoIntakeControl(boolean wantIntake, boolean wantReverse) {
        if (!wantIntake) {
            intake.setPower(0.0);
            intake2.setPower(0.0);
            return;
        }

        String lowerColor = getColor(lower);
        String upperColor = getColor(upper);

        long now = System.currentTimeMillis();

        if (isGreenOrPurple(upperColor)) lastUpperDetectMs = now;
        if (isGreenOrPurple(lowerColor)) lastLowerDetectMs = now;

        boolean upperRecent = (now - lastUpperDetectMs) < 1000;

        double p1, p2;

        if (!wantReverse) {
            p1 = transferConstants.intakePower;
            p2 = -transferConstants.intakePower;
        } else {
            p1 = -transferConstants.intakePower;
            p2 = transferConstants.intakePower;
        }

        if (upperRecent) {
            p2 = 0.0;
        }

        intake.setPower(p1);
        intake2.setPower(p2);
    }

    private String getColor(NormalizedColorSensor sensor) {
        NormalizedRGBA color = sensor.getNormalizedColors();
        double hue = JavaUtil.colorToHue(color.toColor());

        if (hue < 30) return "red";
        if (hue < 60) return "orange";
        if (hue < 90) return "yellow";
        if (hue < 150) return "green";
        if (hue < 225) return "green";
        if (hue < 350) return "purple";
        return "red";
    }

    private boolean isGreenOrPurple(String c) {
        return "green".equals(c) || "purple".equals(c);
    }
}