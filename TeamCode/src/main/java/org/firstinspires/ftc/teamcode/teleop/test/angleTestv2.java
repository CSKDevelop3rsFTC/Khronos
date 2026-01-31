package org.firstinspires.ftc.teamcode.teleop.test;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
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
import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.teamcode.teleop.MasterTeleOp;
import org.firstinspires.ftc.teamcode.teleop.ShooterPID;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;

import org.firstinspires.ftc.teamcode.pedropathing.Constants;

@Config
@TeleOp(name = "Turret Tracking PIDF", group = "Pedro Pathing")
public class angleTestv2 extends OpMode {

    private Follower follower;

    private CRServo turret;
    private CRServo turret2;

    private DcMotorEx turretEncoder;

    // =======================
    // Field target fallback
    // =======================
    public static double TARGET_X = 63;
    public static double TARGET_Y = 63;

    public static boolean USE_HEADING_SCALE = false;
    public static double HEADING_SCALE = 100.0;

    public static double START_X = 24;
    public static double START_Y = 24;
    public static double START_HEADING = 0.7854;

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
    public static double MIN_UNWRAPPED_DEG = -142.5;
    public static double MAX_UNWRAPPED_DEG = 217.5;
    public static double LIMIT_MARGIN_DEG = 2.0;
    public static boolean ALLOW_RECOVERY_WHEN_OUTSIDE_LIMITS = true;

    private DcMotor intake;
    private DcMotor intake2;

    // =======================
    // AprilTag camera
    // =======================
    private static final boolean USE_WEBCAM = true;

    private final Position cameraPosition = new Position(DistanceUnit.INCH,
            0, 3.5, 13, 0);

    private final YawPitchRollAngles cameraOrientation = new YawPitchRollAngles(AngleUnit.DEGREES,
            0, -70, 0, 0);

    private AprilTagProcessor aprilTag;
    private VisionPortal visionPortal;

    // =======================
    // Tag yaw filtering
    // =======================
    public static double TAG_HOLD_MS = 250;
    public static double MIN_DECISION_MARGIN = 35.0;
    public static double YAW_LP_ALPHA = 0.28;
    public static double TAG_YAW_DEADBAND_DEG = 1.2;
    public static double MAX_ABS_TAG_YAW_DEG = 35.0;

    private boolean haveTagFieldAngle = false;
    private double tagFieldAngleDeg = 0.0;

    private final ElapsedTime tagHoldTimer = new ElapsedTime();
    private boolean hadRecentTag = false;
    private double filteredYawDeg = 0.0;

    public ServoImplEx left = null;
    public ServoImplEx right = null;
    public ServoImplEx top = null;

    private NormalizedColorSensor lower;
    private NormalizedColorSensor upper;

    public GoBildaPinpointDriver pinpoint;

    private boolean prevLB = false;
    private boolean prevMid = false;

    // Tracks last time BOTH sensors saw a ball (green/purple)
    private long lastBothDetectedMs = 0;

    // NEW: tracks last time UPPER-only saw a ball (upper green/purple, lower not)
    private long lastUpperOnlyDetectedMs = 0;

    // =======================
    // PIDF knobs (Dashboard)
    // =======================

    private CRServo liftL;
    private CRServo liftR;

    private Servo stopper;

    public static boolean ENABLE_PID = true;

    public static double kP = 0.018;
    public static double kI = 0.0;
    public static double kD = 0.0;

    // Applied as kF * sign(error) when error is outside DEADBAND_DEG.
    public static double kF = 0.08;

    public static boolean USE_F = true;

    public static double MAX_OUTPUT = 1;
    public static double MIN_OUTPUT = 0.0;

    public static double DEADBAND_DEG = 2;

    // Optional: slew-rate limit output to reduce chatter (set 0 to disable)
    public static double OUTPUT_SLEW_PER_SEC = 0.0;
    // CLASS STATES - - - - -
    private enum TransferState {IDLE, AIMING, PRE_FEED, FEEDING}
    private enum ShooterMode {OFF, CLOSE, FAR}
    private ShooterMode shooterMode = ShooterMode.OFF;

    // TRANSFER CONSTANTS - - - -
    private TransferState transferState = TransferState.IDLE;
    private long transferStateEndMs = 0;
    private long turretSettledSinceMs = 0;

    // =======================
    // Internal state
    // =======================
    private int zeroTicks = 0;

    private double integral = 0.0;
    private double prevError = 0.0;
    private final ElapsedTime pidTimer = new ElapsedTime();
    private double lastOutput = 0.0;

    private ShooterPID shooter;

    // Telemetry helpers
    private boolean tagSeenThisLoop = false;
    private int lastTagId = -1;
    private double lastTagYawDegRaw = 0;
    private double lastTagYawDegUsed = 0;
    private double lastDecisionMargin = 0;

    @Configurable
    @Config
    public static class transferConstants {
        public static double intakePower = -1.0;
        public static double GATE_OPEN = 0.3;
        public static double GATE_CLOSED = 0.1;
        public static long aimTimeoutMs = 1500;
        public static long settleMs = 120;
        public static long feedMs = 1230;
        public static long preMs = 200;
        public static long closeRPM = 2200;
        public static long farRPM = 2300;

        // how long to keep intake running after BOTH sensors stop detecting
        public static long bothHoldMs = 250;

        // NEW: how long to keep intake STOPPED after upper-only detection stops flickering
        public static long upperOnlyStopMs = 250;
    }

    @Override
    public void init() {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        initAprilTag();

        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(new Pose(START_X, START_Y, START_HEADING));
        follower.update();

        turret = hardwareMap.get(CRServo.class, "turret");

        turret2 = hardwareMap.get(CRServo.class, "turret2");

        intake = hardwareMap.get(DcMotor.class, "intake");
        intake2 = hardwareMap.get(DcMotor.class, "intake2");

        shooter = new ShooterPID(hardwareMap, "f1M", "f2M");
        shooter.setTelemetry(new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry()));

        turretEncoder = hardwareMap.get(DcMotorEx.class, "intake2");
        turretEncoder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        turretEncoder.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        stopper = hardwareMap.get(Servo.class, "stopper");

        left = hardwareMap.get(ServoImplEx.class, "left");
        right = hardwareMap.get(ServoImplEx.class, "right");
        //top = hardwareMap.get(ServoImplEx.class, "top");

        lower = hardwareMap.get(ColorRangeSensor.class, "lower");
        upper = hardwareMap.get(ColorRangeSensor.class, "upper");

        liftL = hardwareMap.get(CRServo.class, "liftL");
        liftR = hardwareMap.get(CRServo.class, "liftR");

        zeroTicks = 0;
        setTurretPower(0);

        hadRecentTag = false;
        filteredYawDeg = 0.0;
        tagHoldTimer.reset();

        haveTagFieldAngle = false;
        tagFieldAngleDeg = 0.0;

        stopper.setPosition(MasterTeleOp.transferConstants.GATE_CLOSED);


        integral = 0.0;
        prevError = 0.0;
        lastOutput = 0.0;
        pidTimer.reset();
    }

    @Override
    public void start() {
        follower.startTeleopDrive();
        follower.update();
        pidTimer.reset();
    }

    public String getColor(NormalizedColorSensor sensor) {
        double hue;
        NormalizedRGBA color = sensor.getNormalizedColors();
        hue = JavaUtil.colorToHue(color.toColor());
        String detectedColor = "";

        if(hue < 30){
            detectedColor = "red";
        }
        else if (hue < 60) {
            detectedColor = "orange";
        }
        else if (hue < 90){
            detectedColor = "yellow";
        }
        else if (hue < 150){
            detectedColor = "green";
        }
        else if (hue < 225){
            detectedColor = "green";
        }
        else if (hue < 350){
            detectedColor = "purple";
        }
        else{
            detectedColor = "red";
        }
        return detectedColor;
    }

    private boolean isGreenOrPurple(String c) {
        return "green".equals(c) || "purple".equals(c);
    }

    public void liftControl() {
        if (gamepad1.y) {
            liftL.setPower(1);
            liftR.setPower(1);
        }
        else if (gamepad1.a) {
            liftL.setPower(-1);
            liftR.setPower(-1);
        } else {
            liftL.setPower(0);
            liftR.setPower(0);
        }
    }

    public void lightControl() {
        String lowerColor = getColor(lower);
        String upperColor = getColor(upper);

        long now = System.currentTimeMillis();

        boolean bothDetected = isGreenOrPurple(upperColor) && isGreenOrPurple(lowerColor);
        boolean upperOnlyDetected = isGreenOrPurple(upperColor) && !isGreenOrPurple(lowerColor);

        // if BOTH detect, refresh the "keep running" timer
        if (bothDetected) {
            lastBothDetectedMs = now;
        }

        // NEW: if UPPER-only detects, refresh the "keep stopped" timer
        if (upperOnlyDetected) {
            lastUpperOnlyDetectedMs = now;
        }
        //white when you have all three
        //off when nothing
        if (isGreenOrPurple(upperColor) && isGreenOrPurple(lowerColor)) {
            left.setPosition(1.0);
            right.setPosition(1.0);
        } else {
            left.setPosition(0);
            right.setPosition(0);
        }
    }

    public void intake() {
        String lowerColor = getColor(lower);
        String upperColor = getColor(upper);

        telemetry.addData("lower: ", lowerColor);
        telemetry.addData("upper: ", upperColor);
        telemetry.update();

        long now = System.currentTimeMillis();

        boolean bothDetected = isGreenOrPurple(upperColor) && isGreenOrPurple(lowerColor);
        boolean upperOnlyDetected = isGreenOrPurple(upperColor) && !isGreenOrPurple(lowerColor);

        // if BOTH detect, refresh the "keep running" timer
        if (bothDetected) {
            lastBothDetectedMs = now;
        }

        // NEW: if UPPER-only detects, refresh the "keep stopped" timer
        if (upperOnlyDetected) {
            lastUpperOnlyDetectedMs = now;
        }

        if (gamepad1.left_trigger > 0) {
            intake.setPower(transferConstants.intakePower);
            intake2.setPower(-transferConstants.intakePower); }
        else if (isGreenOrPurple(upperColor) && isGreenOrPurple(lowerColor)){
            //intake.setPower(-transferConstants.intakePower * 1);
        }

        // hold intake ON until we've gone a full second with no moment where BOTH detect
        boolean bothHoldActive = lastBothDetectedMs != 0 && (now - lastBothDetectedMs) < transferConstants.bothHoldMs;
        if (bothHoldActive) {
            //intake.setPower(-transferConstants.intakePower * 1);
            return;
        }

        // NEW: hold intake OFF until we've gone upperOnlyStopMs without UPPER-only detection
        boolean upperStopHoldActive = lastUpperOnlyDetectedMs != 0 && (now - lastUpperOnlyDetectedMs) < transferConstants.upperOnlyStopMs;
        if (upperStopHoldActive) {
            intake2.setPower(0);
            return;
        }

        // existing logic below (unchanged)

        else if (isGreenOrPurple(upperColor)) {
            if (gamepad1.left_trigger > 0) {
                intake.setPower(transferConstants.intakePower);
                intake2.setPower(-transferConstants.intakePower);
            } else if (gamepad1.right_trigger > 0) {
                intake.setPower(-transferConstants.intakePower);
                intake2.setPower(0);
            } else {
                intake.setPower(0);
                intake2.setPower(0);
            }
        }

        else {
            if (gamepad1.left_trigger > 0) {
                intake.setPower(transferConstants.intakePower);
                intake2.setPower(-transferConstants.intakePower);
            } else if (gamepad1.right_trigger > 0) {
                intake.setPower(-transferConstants.intakePower);
                intake2.setPower(transferConstants.intakePower);
            } else {
                intake.setPower(0);
                intake2.setPower(0);
            }
        }
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
                //boolean aimed = aimTurretAtGoal(fieldX, fieldY);
                boolean aimed = true; // FIX THIS ONCE AIM IS FINISHED
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
                intake.setPower(-transferConstants.intakePower);
                intake2.setPower( transferConstants.intakePower);

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

    public void transferControl() {
        if (gamepad1.right_bumper) startTransfer();

        if (transferActive()) {
            updateTransfer();
        }
    }

    @Override
    public void loop() {
        tagSeenThisLoop = false;
        lastTagId = -1;
        lastTagYawDegRaw = 0;
        lastTagYawDegUsed = 0;
        lastDecisionMargin = 0;

        transferControl();
        shooterGamepad();
        liftControl();
        //lightControl();

        if (!transferActive()) shooterControl();
        shooter.update();

        if (!transferActive()) {
            intake();
        }

        follower.setTeleOpDrive(
                -gamepad1.left_stick_y,
                gamepad1.left_stick_x,
                -gamepad1.right_stick_x,
                true
        );
        follower.update();

        if (gamepad1.y) {
            zeroTicks = turretEncoder.getCurrentPosition();
            haveTagFieldAngle = false;

            integral = 0.0;
            prevError = 0.0;
            lastOutput = 0.0;
            pidTimer.reset();
        }

        Pose p = follower.getPose();
        double robotHeadingDeg = getRobotHeadingDeg(p);

        int ticks = turretEncoder.getCurrentPosition();
        int ticksFromZero = (ticks - zeroTicks) * ENCODER_SIGN;

        double currentTurretUnwrappedDeg = turretDegFromTicksUnwrapped(ticksFromZero);
        double currentTurretWrappedDeg = wrapDeg(currentTurretUnwrappedDeg);

        // -----------------------
        // Tag yaw read/filter/hold
        // -----------------------
        Double rawYaw = getBestTagYawDegRaw();
        Double yawToUse = null;

        if (rawYaw != null) {
            double clamped = clamp(rawYaw, -MAX_ABS_TAG_YAW_DEG, MAX_ABS_TAG_YAW_DEG);
            filteredYawDeg = (YAW_LP_ALPHA * clamped) + ((1.0 - YAW_LP_ALPHA) * filteredYawDeg);

            hadRecentTag = true;
            tagHoldTimer.reset();
            yawToUse = filteredYawDeg;
        } else if (hadRecentTag && tagHoldTimer.milliseconds() <= TAG_HOLD_MS) {
            yawToUse = filteredYawDeg;
        } else {
            hadRecentTag = false;
            yawToUse = null;
        }

        if (yawToUse != null && Math.abs(yawToUse) < TAG_YAW_DEADBAND_DEG) {
            yawToUse = 0.0;
        }

        // -----------------------
        // Build desired turret angle
        // -----------------------
        double desiredTurretWrappedDeg;
        String aimMode;

        if (yawToUse != null) {
            tagFieldAngleDeg = wrapDeg(robotHeadingDeg + currentTurretWrappedDeg + yawToUse);
            haveTagFieldAngle = true;
            lastTagYawDegUsed = yawToUse;
        }

        if (haveTagFieldAngle && (yawToUse != null || (hadRecentTag && tagHoldTimer.milliseconds() <= TAG_HOLD_MS))) {
            desiredTurretWrappedDeg = wrapDeg(tagFieldAngleDeg - robotHeadingDeg);
            aimMode = "TAG_FIELD_LOCK";
        } else {
            desiredTurretWrappedDeg = desiredTurretDegToPoint(p, TARGET_X, TARGET_Y);
            aimMode = "FIELD_POINT";
            haveTagFieldAngle = false;
        }

        // Choose target representation that stays inside window
        double safeTargetUnwrappedDeg = chooseTargetWithinCableWindow(
                desiredTurretWrappedDeg,
                currentTurretUnwrappedDeg
        );

        // -----------------------
        // PIDF position control
        // -----------------------
        double errorDeg = safeTargetUnwrappedDeg - currentTurretUnwrappedDeg;

        double dt = pidTimer.seconds();
        pidTimer.reset();
        if (dt <= 0) dt = 1e-3;

        double output = 0.0;

        if (ENABLE_PID) {
            double minEff = MIN_UNWRAPPED_DEG + LIMIT_MARGIN_DEG;
            double maxEff = MAX_UNWRAPPED_DEG - LIMIT_MARGIN_DEG;

            boolean outsideHard = (currentTurretUnwrappedDeg < MIN_UNWRAPPED_DEG) || (currentTurretUnwrappedDeg > MAX_UNWRAPPED_DEG);

            if (outsideHard && !ALLOW_RECOVERY_WHEN_OUTSIDE_LIMITS) {
                output = 0.0;
            } else {
                // If outside hard window, only allow motion back inside
                if (outsideHard) {
                    if (currentTurretUnwrappedDeg < MIN_UNWRAPPED_DEG && errorDeg <= 0) {
                        output = 0.0;
                    } else if (currentTurretUnwrappedDeg > MAX_UNWRAPPED_DEG && errorDeg >= 0) {
                        output = 0.0;
                    } else {
                        output = pidfStep(errorDeg, dt);
                    }
                } else {
                    // Inside window: normal control, but keep target in effective window already
                    safeTargetUnwrappedDeg = clamp(safeTargetUnwrappedDeg, minEff, maxEff);
                    errorDeg = safeTargetUnwrappedDeg - currentTurretUnwrappedDeg;
                    output = pidfStep(errorDeg, dt);
                }
            }
        } else {
            output = 0.0;
            integral = 0.0;
            prevError = 0.0;
            lastOutput = 0.0;
        }

        setTurretPower(output);

        telemetry.addData("Aim Mode", aimMode);

        telemetry.addData("Pose (x,y)", "%.1f, %.1f", p.getX(), p.getY());
        telemetry.addData("Robot Heading", "%.1f deg", robotHeadingDeg);

        telemetry.addData("Turret Now (wrap)", "%.1f", currentTurretWrappedDeg);
        telemetry.addData("Turret Now (raw)", "%.1f", currentTurretUnwrappedDeg);

        telemetry.addData("Turret Target (wrap)", "%.1f", desiredTurretWrappedDeg);
        telemetry.addData("Turret Target (raw)", "%.1f", safeTargetUnwrappedDeg);

        telemetry.addData("Error (deg)", "%.2f", errorDeg);
        telemetry.addData("PID Out", "%.3f", output);
        telemetry.addData("kP/kF", "%.3f / %.3f", kP, kF);

        telemetry.addData("Tag Field Angle", haveTagFieldAngle ? "%.1f" : "none", tagFieldAngleDeg);
        telemetry.addData("Tag Fresh", "%s hold=%.0fms", hadRecentTag ? "yes" : "no", tagHoldTimer.milliseconds());

        if (tagSeenThisLoop) {
            telemetry.addData("Tag", "id=%d dm=%.1f rawYaw=%.2f usedYaw=%.2f",
                    lastTagId, lastDecisionMargin, lastTagYawDegRaw, lastTagYawDegUsed);
        } else {
            telemetry.addData("Tag", "none");
        }

        telemetry.addLine("Y = zero turret");
        telemetry.update();
    }

    @Override
    public void stop() {
        setTurretPower(0);
        if (visionPortal != null) {
            visionPortal.close();
        }
    }

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

    public void shooterControl() {
        switch (shooterMode) {
            case CLOSE:
                shooter.setTargetRPM(transferConstants.closeRPM);
                break;
            case FAR:
                shooter.setTargetRPM(transferConstants.farRPM);
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

    // One PIDF step given error in degrees
    private double pidfStep(double errorDeg, double dt) {
        // Deadband to stop hunting
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
        if (USE_F) {
            ff = kF * Math.signum(errorDeg);
        }

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

        // Extra guard at the effective window edges to avoid pushing deeper into the margin zone
        double minEff = MIN_UNWRAPPED_DEG + LIMIT_MARGIN_DEG;
        double maxEff = MAX_UNWRAPPED_DEG - LIMIT_MARGIN_DEG;
        // We don't know current position here; caller already clamps target and handles outsideHard recovery.
        // Keeping this function pure on error.

        return out;
    }

    private void initAprilTag() {
        aprilTag = new AprilTagProcessor.Builder()
                .setCameraPose(cameraPosition, cameraOrientation)
                .setOutputUnits(DistanceUnit.INCH, AngleUnit.DEGREES)
                .setLensIntrinsics(553.605, 553.605, 298.688, 236.59)
                .build();

        VisionPortal.Builder builder = new VisionPortal.Builder();

        if (USE_WEBCAM) {
            builder.setCamera(hardwareMap.get(WebcamName.class, "anki"));
        } else {
            builder.setCamera(BuiltinCameraDirection.BACK);
        }

        builder.setStreamFormat(VisionPortal.StreamFormat.MJPEG);
        builder.addProcessor(aprilTag);

        visionPortal = builder.build();
    }

    private Double getBestTagYawDegRaw() {
        if (aprilTag == null) return null;

        List<AprilTagDetection> detections = aprilTag.getDetections();
        if (detections == null || detections.isEmpty()) return null;

        AprilTagDetection best = null;
        double bestDecision = -1e9;
        double bestRange = 1e9;

        for (AprilTagDetection d : detections) {
            if (d == null) continue;
            if (d.ftcPose == null) continue;

            double decision = d.decisionMargin;
            if (decision < MIN_DECISION_MARGIN) continue;

            double range = d.ftcPose.range;

            boolean better =
                    (decision > bestDecision) ||
                            (decision == bestDecision && range < bestRange);

            if (best == null || better) {
                best = d;
                bestDecision = decision;
                bestRange = range;
            }
        }

        if (best == null) return null;

        tagSeenThisLoop = true;
        lastTagId = best.id;
        lastDecisionMargin = best.decisionMargin;
        lastTagYawDegRaw = best.ftcPose.yaw;

        return best.ftcPose.yaw;
    }

    private void setTurretPower(double ccwPositivePower) {
        turret.setPower(ccwPositivePower * TURRET_SERVO_SIGN);
        turret2.setPower(ccwPositivePower * TURRET2_SERVO_SIGN);
    }

    private double desiredTurretDegToPoint(Pose robotPose, double targetX, double targetY) {
        double dx = targetX - robotPose.getX();
        double dy = targetY - robotPose.getY();

        double targetBearingDeg = Math.toDegrees(Math.atan2(dy, dx));
        double robotHeadingDeg = getRobotHeadingDeg(robotPose);

        return wrapDeg(targetBearingDeg - robotHeadingDeg);
    }

    private double getRobotHeadingDeg(Pose robotPose) {
        double h = robotPose.getHeading();
        if (USE_HEADING_SCALE) {
            return wrapDeg(h * HEADING_SCALE);
        }
        return wrapDeg(Math.toDegrees(h));
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
}
