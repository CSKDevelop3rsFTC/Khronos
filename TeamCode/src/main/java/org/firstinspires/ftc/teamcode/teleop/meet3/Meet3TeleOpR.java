package org.firstinspires.ftc.teamcode.teleop.meet3;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.teleop.ShooterPID;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;

import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.bylazar.configurables.annotations.Configurable;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoImplEx;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.vision.apriltag.AprilTagGameDatabase;
import org.firstinspires.ftc.vision.apriltag.AprilTagMetadata;

@TeleOp(name="Meet3TeleOp - Red", group="Robot")
@Disabled
public class Meet3TeleOpR extends LinearOpMode {

    public DcMotor frontLeftDrive = null;
    public DcMotor frontRightDrive = null;
    public DcMotor backLeftDrive = null;
    public DcMotor backRightDrive = null;

    public DcMotor intakeMotor = null;

    public DcMotor turretMotor = null;

    public Servo   kickerServo = null;
    public Servo   stopperServo = null;

    public ServoImplEx   hoodServo = null;

    public ServoImplEx leftLight  = null;
    public ServoImplEx rightLight = null;
    public ServoImplEx top = null;

    public GoBildaPinpointDriver pinpoint;

    public IMU imu;

    public boolean lightBool;
    public boolean shooterBool;

    AprilTagProcessor aprilTag;
    VisionPortal portal;

    // --- SIMPLE TWO-PRESET SHOOT CONTROL ---
    private enum ShooterMode { OFF, CLOSE, FAR }
    private ShooterMode shooterMode = ShooterMode.OFF;

    private boolean prevLB = false;      // close toggle (LB)
    private boolean prevMid = false;     // far toggle (center/alt button)

    // Pick the one your DS5 actually reports.
    private boolean midButtonPressed() {
        // If options doesn't work, try: gamepad1.ps or gamepad1.share
        return gamepad1.touchpad;
    }

    public int pos;

    public double CALC_RPM = 0.0;

    // Shooter PID subsystem (newTuner-style)
    private ShooterPID shooter;

    private static final double MOTOR_CPR        = 537.6;      // GoBILDA 312RPM CPR at output shaft
    private static final int    GEAR_INPUT_TEETH = 63;         // motor gear
    private static final int    GEAR_OUTPUT_TEETH= 190;        // lazy susan gear

    // motor turns per 1 turret turn = output/input
    private static final double GEAR_RATIO       = (double) GEAR_OUTPUT_TEETH / GEAR_INPUT_TEETH; // 190/63 ≈ 3.01587
    private static final double TICKS_PER_TURRET_REV = MOTOR_CPR * GEAR_RATIO;
    private static final double TICKS_PER_DEG = TICKS_PER_TURRET_REV / 360.0;
    private static final double DEG_PER_TICK = 360.0 / TICKS_PER_TURRET_REV;

    // simple position controller
    private static final double TURRET_KP        = 0.01;
    private static final double TURRET_HOLD      = 0.05;
    private static final double TURRET_DEG_TOL   = 2.0;

    // Fused pose used for aiming (pinpoint + tag corrections)
    private double fusedX = -63.0;
    private double fusedY = 0.0;
    private double fusedHeadingDeg = 0.0;

    private double odoOffX = 0.0;
    private double odoOffY = 0.0;

    private boolean fusedInitialized = false;
    private double lastOdoX = 0.0;
    private double lastOdoY = 0.0;

    private final ElapsedTime ledBlinkTimer = new ElapsedTime();
    private boolean whiteOn = false;

    Gamepad.LedEffect red = new Gamepad.LedEffect.Builder()
            .addStep(1, 0, 0, 250)
            .build();

    Gamepad.LedEffect blue = new Gamepad.LedEffect.Builder()
            .addStep(0.12, 0.32, 1, 250)
            .build();

    Gamepad.LedEffect white = new Gamepad.LedEffect.Builder()
            .addStep(1, 1, 1, 250)
            .build();

    public void initialize() {
        frontLeftDrive  = hardwareMap.get(DcMotor.class, "fl");
        frontRightDrive = hardwareMap.get(DcMotor.class, "fr");
        backLeftDrive   = hardwareMap.get(DcMotor.class, "bl");
        backRightDrive  = hardwareMap.get(DcMotor.class, "br");

        // ShooterPID owns f1M and f2M
        shooter = new ShooterPID(hardwareMap, "f1M", "f2M");
        shooter.setTelemetry(
                new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry())
        );

        intakeMotor     = hardwareMap.get(DcMotor.class, "intake");

        turretMotor     = hardwareMap.get(DcMotor.class, "turret");

        kickerServo     = hardwareMap.get(Servo.class, "kicker");
        stopperServo    = hardwareMap.get(Servo.class, "stopper");

        hoodServo       = hardwareMap.get(ServoImplEx.class, "hood");

        leftLight       = hardwareMap.get(ServoImplEx.class, "left");
        rightLight      = hardwareMap.get(ServoImplEx.class, "right");
        top             = hardwareMap.get(ServoImplEx.class, "top");

        pinpoint = hardwareMap.get(GoBildaPinpointDriver.class, "pinpoint");

        frontLeftDrive.setDirection(DcMotorSimple.Direction.REVERSE);
        backLeftDrive.setDirection(DcMotorSimple.Direction.REVERSE);
        frontRightDrive.setDirection(DcMotorSimple.Direction.FORWARD);
        backRightDrive.setDirection(DcMotorSimple.Direction.FORWARD);

        leftLight.setPwmEnable();
        rightLight.setPwmEnable();

        lightBool = false;
        shooterBool = false;

        // safe defaults
        kickerServo.setPosition(0.5);
        stopperServo.setPosition(0.5);

        pos = 0;

        imu = hardwareMap.get(IMU.class, "imu");
        IMU.Parameters imuParams = new IMU.Parameters(
                new RevHubOrientationOnRobot(
                        RevHubOrientationOnRobot.LogoFacingDirection.RIGHT,
                        RevHubOrientationOnRobot.UsbFacingDirection.UP
                )
        );

        // aprilTag = AprilTagProcessor.easyCreateWithDefaults();
        aprilTag = new AprilTagProcessor.Builder()
                // Swap this to your season library:
                .setTagLibrary(AprilTagGameDatabase.getDecodeTagLibrary())
                .build();


        portal = new VisionPortal.Builder()
                .setCamera(hardwareMap.get(WebcamName.class, "anki"))
                .setStreamFormat(VisionPortal.StreamFormat.MJPEG)
                .addProcessor(aprilTag)
                .build();

        turretMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        turretMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        hoodServo.setDirection(Servo.Direction.REVERSE);
        turretMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        turretMotor.setDirection(DcMotorSimple.Direction.FORWARD);

        imu.initialize(imuParams);
        pinpoint.resetPosAndIMU();

        ledBlinkTimer.reset();
    }

    public void driveMovement(double driveSpeed) {
        double horizontal = -gamepad1.left_stick_x;   // flip strafe
        double vertical   =  gamepad1.left_stick_y;   // keep forward/back the same
        double pivot      =  gamepad1.right_stick_x * 0.5; // flip turn

        frontRightDrive.setPower(((vertical + horizontal) + pivot) * driveSpeed);
        frontLeftDrive.setPower(((vertical - horizontal) - pivot) * driveSpeed);
        backRightDrive.setPower(((vertical - horizontal) + pivot) * driveSpeed);
        backLeftDrive.setPower(((vertical + horizontal) - pivot) * driveSpeed);
    }

    // ====== VISION RELocalization (AprilTag) ======

    // Camera pose relative to robot center (your coordinate convention must match your odom field axes).
// Assume robot X is field X, robot Y is field Y; and robot forward is +Y in field.
// If your coordinate system differs, we’ll adjust, but start here.
    @Config
    public static class VisionReloc {
        // inches: camera position relative to robot center
        public static double CAM_FORWARD_IN  = 3.3458;   // +forward
        public static double CAM_LEFT_IN     = 0.0;   // +left
        public static double CAM_YAW_DEG     = 0.0;   // camera yaw relative to robot forward (+CCW)

        // How hard we trust tags. 0.05 means “5% correction per detection”.
        public static double BLEND_XY        = 0.03;
        public static double BLEND_HEADING   = 0.10;

        public static boolean SWAP_FIELD_XY = false;
        public static boolean INVERT_FIELD_X = false;
        public static boolean INVERT_FIELD_Y = false;

        public static double MAX_CORRECTION_IN = 18.0;

        // Only trust tags within this distance (inches). Prevents garbage far away.
        public static double MAX_RANGE_IN    = 90.0;

        // Require ambiguity low-ish if your SDK exposes it (some versions do).
        public static double MAX_AMBIGUITY   = 0.20;

    }

    private static double wrapDeg180(double a) {
        while (a > 180) a -= 360;
        while (a < -180) a += 360;
        return a;
    }

    private static double lerp(double a, double b, double t) {
        return a + (b - a) * t;
    }

    private static double lerpAngleDeg(double aDeg, double bDeg, double t) {
        double err = wrapDeg180(bDeg - aDeg);
        return wrapDeg180(aDeg + err * t);
    }

    private AprilTagDetection pickBestTag() {
        AprilTagDetection best = null;
        double bestRange = 1e9;

        for (AprilTagDetection det : aprilTag.getDetections()) {
            if (det == null) continue;
            if (det.metadata == null) continue; // no known field location, skip
            if (det.ftcPose == null) continue;

            // ftcPose.y is typically "forward" distance from camera to tag (inches)
            double range = Math.hypot(det.ftcPose.x, det.ftcPose.y);
            if (range > VisionReloc.MAX_RANGE_IN) continue;

            if (range < bestRange) {
                bestRange = range;
                best = det;
            }
        }
        return best;
    }

    /**
     * Estimate robot field pose from a tag detection using tag's known field position + camera-to-tag measurement.
     * Returns true if we successfully updated fused pose.
     */
    private boolean relocalizeFromAprilTags(double odoX, double odoY) {
        AprilTagDetection det = pickBestTag();
        if (det == null) return false;

        AprilTagMetadata meta = det.metadata;
        if (meta == null || meta.fieldPosition == null) return false;

        // Tag position on the FIELD (inches)
        // meta.fieldPosition is a Position in the SDK with units baked in; we’ll read INCH.
        double tagFieldX = meta.fieldPosition.get(0);
        double tagFieldY = meta.fieldPosition.get(1);

        double tx = tagFieldX;
        double ty = tagFieldY;

        if (VisionReloc.SWAP_FIELD_XY) {
            double tmp = tx; tx = ty; ty = tmp;
        }
        if (VisionReloc.INVERT_FIELD_X) tx = -tx;
        if (VisionReloc.INVERT_FIELD_Y) ty = -ty;

        tagFieldX = tx;
        tagFieldY = ty;

        // Camera -> Tag measurement in the CAMERA frame (inches)
        // FTC Pose convention: +x left, +y forward, +z up (common FTC vision convention)
        double camToTag_left = det.ftcPose.x;
        double camToTag_fwd  = det.ftcPose.y;

        // Convert camera->tag into FIELD frame using estimated robot heading (from IMU) as the base.
        // We use IMU yaw because it’s stable short-term and you already have it.
        double robotYawDeg = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
        double camYawDeg = wrapDeg180(robotYawDeg + VisionReloc.CAM_YAW_DEG);

        double camYawRad = Math.toRadians(camYawDeg);

        // Camera->Tag vector in FIELD axes (assuming field +X right, +Y forward)
        // camToTag_left is +left. Convert to +X right by negating left.
        double camToTag_right = -camToTag_left;
        double camToTag_forward = camToTag_fwd;

        double camToTag_fieldX = camToTag_right * Math.cos(camYawRad) - camToTag_forward * Math.sin(camYawRad);
        double camToTag_fieldY = camToTag_right * Math.sin(camYawRad) + camToTag_forward * Math.cos(camYawRad);

        // Camera field position = Tag field position - (Camera->Tag field vector)
        double camFieldX = tagFieldX - camToTag_fieldX;
        double camFieldY = tagFieldY - camToTag_fieldY;

        // Robot field position = Camera field position - (Robot->Camera offset rotated into FIELD)
        double robToCam_fwd  = VisionReloc.CAM_FORWARD_IN;
        double robToCam_left = VisionReloc.CAM_LEFT_IN;

        double robToCam_fieldX = (-robToCam_left) * Math.cos(Math.toRadians(robotYawDeg))
                + (robToCam_fwd)  * Math.sin(Math.toRadians(robotYawDeg));
        double robToCam_fieldY = (-robToCam_left) * Math.sin(Math.toRadians(robotYawDeg))
                + (robToCam_fwd)  * Math.cos(Math.toRadians(robotYawDeg));

        double visionRobotX = camFieldX - robToCam_fieldX;
        double visionRobotY = camFieldY - robToCam_fieldY;

        // Heading: for now, keep IMU yaw as the heading truth.
        double visionHeadingDeg = robotYawDeg;

        double dx = visionRobotX - fusedX;
        double dy = visionRobotY - fusedY;
        double corr = Math.hypot(dx, dy);

        telemetry.addData("tag id", det.id);
        telemetry.addData("tagField", "x=%.1f y=%.1f", tagFieldX, tagFieldY);
        telemetry.addData("cam->tag", "L=%.1f F=%.1f", camToTag_left, camToTag_fwd);
        telemetry.addData("visionRobot", "x=%.1f y=%.1f", visionRobotX, visionRobotY);
        telemetry.addData("fused", "x=%.1f y=%.1f", fusedX, fusedY);
        telemetry.addData("corr", "%.1f", Math.hypot(visionRobotX - fusedX, visionRobotY - fusedY));

        if (corr > VisionReloc.MAX_CORRECTION_IN) {
            telemetry.addData("Tag reject", corr);
            return false;
        }

        double desOffX = visionRobotX - odoX;
        double desOffY = visionRobotY - odoY;

        odoOffX = lerp(odoOffX, desOffX, VisionReloc.BLEND_XY);
        odoOffY = lerp(odoOffY, desOffY, VisionReloc.BLEND_XY);

        // Blend into fused pose (soft correction)
        fusedX = lerp(fusedX, visionRobotX, VisionReloc.BLEND_XY);
        fusedY = lerp(fusedY, visionRobotY, VisionReloc.BLEND_XY);
        fusedHeadingDeg = lerpAngleDeg(fusedHeadingDeg, visionHeadingDeg, VisionReloc.BLEND_HEADING);

        return true;
    }

    @Config
    public static class Goal {
        public static double X =  72.0;
        public static double Y =  68.0 - 48.0;
        public static double TURRET_MAX_POWER = 0.2;
    }

    private static double wrapDeg(double a){
        while(a > 180)  a -= 360;
        while(a < -180) a += 360;
        return a;
    }

    private static double angleToGoalDeg(double robotX, double robotY){
        double dx = Goal.X - robotX;
        double dy = Goal.Y - robotY;
        return Math.toDegrees(Math.atan2(dy, dx));
    }

    private static double distanceToGoal(double robotX, double robotY) {
        double dx = Goal.X - robotX;
        double dy = Goal.Y - robotY;
        return Math.hypot(dx, dy);
    }

    private double distanceToGoal() {
        return distanceToGoal(currentPose.x, currentPose.y);
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

    private boolean driveTurretToDeg(double targetTurretDeg){
        int ticks = turretMotor.getCurrentPosition();
        double turretDegNow = ticks * DEG_PER_TICK;
        double errDeg = wrapDeg(targetTurretDeg - turretDegNow);

        if (Math.abs(errDeg) <= 0.5) {
            turretMotor.setPower(0.0);
            return true;
        }

        double cmd = TURRET_KP * (errDeg * TICKS_PER_DEG);
        cmd = Math.max(-Goal.TURRET_MAX_POWER, Math.min(Goal.TURRET_MAX_POWER, cmd));

        if (Math.abs(cmd) < TURRET_HOLD) cmd = Math.signum(cmd) * TURRET_HOLD;

        turretMotor.setPower(cmd);
        return Math.abs(errDeg) <= TURRET_DEG_TOL;
    }

    // ====== TURRET HEADING CODE (from your working version) ======

    private void holdTurretRobotZero() {
        driveTurretToDeg(0.0);
    }

    private boolean aimTurretAtGoal(double robotX, double robotY) {
        double yawDeg = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
        double fieldAim = angleToGoalDeg(robotX, robotY);

        // Only applied when aiming to shoot (transfer uses this function).
        double turretCmdDeg = wrapDeg((fieldAim - yawDeg) + AimOffsets.SHOOT_OFFSET_DEG);

        return driveTurretToDeg(turretCmdDeg);
    }

    @Config
    public static class AimOffsets {
        // Positive = turret aims more to the left or right depending on your turret motor direction.
        // Tune this on the field until shots land center.
        public static double SHOOT_OFFSET_DEG = 0.0;
    }

    @Configurable
    @Config
    public static class transferConstants {
        public static int counterConstant = 3;

        public static double intakePower = 1;

        public static double serv = 0;

        public static int CALC_RPM = 0;

        public static long firstWait     = 150;
        public static long waitAfterKick = 200;
        public static long pauseWait     = 100;
        public static long pauseWait2    = 200;
    }

    public void driveController() {
        if (gamepad1.right_stick_button) driveMovement(0.5);
        else driveMovement(1.0);
    }

    public void intake() {
        if (gamepad1.right_trigger > 0)      intakeMotor.setPower(1);
        else if (gamepad1.left_trigger > 0)  intakeMotor.setPower(-1);
        else if (gamepad1.left_stick_y > 0)  intakeMotor.setPower(1);
        else                                 intakeMotor.setPower(0);
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
                CALC_RPM = shooterCalc();
                shooter.setTargetRPM(CALC_RPM);
                //hoodServo.setPosition(hoodCalc());
                break;
            case OFF:
            default:
                shooter.setTargetRPM(0.0);
                break;
        }
    }

    public void kicker() {
        if (gamepad1.x) kickerServo.setPosition(0);
        if (gamepad1.b) kickerServo.setPosition(0.5);
    }

    public void resetServo() {
        if (gamepad2.x) hoodServo.setPosition(transferConstants.serv);
    }

    public void stopper() {
        if (gamepad1.y) stopperServo.setPosition(0.5);
        if (gamepad1.a) stopperServo.setPosition(0);
    }

    // ====== UPDATED: keeps turret aimed while waiting ======
    private void sleepWithShooter(long millis) {
        long end = System.currentTimeMillis() + millis;
        while (opModeIsActive() && System.currentTimeMillis() < end) {
            shooter.update();
            aimTurretAtGoal(currentPose.x, currentPose.y);
            updateBlinkWhiteIfAtSpeed();
            telemetry.update();
            sleep(10);
        }
    }

    // ====== UPDATED: aims first, fires, then returns to robot-zero ======
    public void transfer() {
        long aimTimeoutMs = 1500;
        long end = System.currentTimeMillis() + aimTimeoutMs;
        while (opModeIsActive() && System.currentTimeMillis() < end) {
            shooter.update();
            if (aimTurretAtGoal(currentPose.x, currentPose.y)) break;
            sleep(10);
        }

        stopperServo.setPosition(0);
        sleepWithShooter(transferConstants.pauseWait);
        intakeMotor.setPower(transferConstants.intakePower);
        sleepWithShooter(transferConstants.firstWait);
        intakeMotor.setPower(0);

        intakeMotor.setPower(transferConstants.intakePower);
        sleepWithShooter(transferConstants.pauseWait2);

        kickerServo.setPosition(0);
        sleepWithShooter(transferConstants.waitAfterKick);
        kickerServo.setPosition(0.5);

        stopperServo.setPosition(0.5);
        intakeMotor.setPower(0);

        long backTimeoutMs = 1500;
        end = System.currentTimeMillis() + backTimeoutMs;
        while (opModeIsActive() && System.currentTimeMillis() < end) {
            shooter.update();
            holdTurretRobotZero();
            sleep(10);
        }

        whiteOn = false;
        lightControl("blue");
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

    // =========================
    //  CORNER LOCALIZATION
    // =========================
    public static class Pose {
        public double x;        // inches
        public double y;
        public double heading;  // radians

        public Pose(double x, double y, double heading) {
            this.x = x;
            this.y = y;
            this.heading = heading;
        }
    }

    private static final double HALF_FIELD = 72.0;

    private static final Pose BLUE_CORNER_POSE =
            new Pose(-HALF_FIELD, -HALF_FIELD, Math.toRadians(0));

    private static final Pose RED_CORNER_POSE =
            new Pose(HALF_FIELD, HALF_FIELD, Math.toRadians(180));

    private Pose currentPose = new Pose(-63, 0, 0);

    public void localizeAtBlueCorner() {
        currentPose = new Pose(
                BLUE_CORNER_POSE.x,
                BLUE_CORNER_POSE.y,
                BLUE_CORNER_POSE.heading
        );
    }

    public void localizeAtRedCorner() {
        currentPose = new Pose(
                RED_CORNER_POSE.x,
                RED_CORNER_POSE.y,
                RED_CORNER_POSE.heading
        );
    }

    public Pose getBlueCornerPose() {
        return BLUE_CORNER_POSE;
    }

    public Pose getRedCornerPose() {
        return RED_CORNER_POSE;
    }

    public Pose getCurrentPose() {
        return currentPose;
    }

    public double shooterCalc() {
        /**
         double quad1, quad2, quad3, quad4;
         double dist = Meet2TeleOp.distanceToGoal(currentPose.x, currentPose.y);

         quad4 = 0.000027215 * Math.pow(dist, 4);

         quad3 = 0.0101462 * Math.pow(dist, 3);

         quad2 = 1.33935 * Math.pow(dist, 2);

         quad1 = (70.22393*dist);

         double pow = (quad4 - quad3 + quad2 - quad1) + 1669.51649;

         return pow;

         double quad1, quad2;
         double dist = distanceToGoal(currentPose.x, currentPose.y);

         quad2 = -1263.57192;
         quad1 = 407.62728 * (Math.log(dist));

         return (quad2 + quad1);
         **/
        // dist is your input variable (x in the equation)
        double dist = distanceToGoal(currentPose.x, currentPose.y);
        //double y = 774.82083 / (1.0 + Math.exp(-(0.0562895 * dist - 4.89505)));
        if (dist >= 115) {
            double y = (1.91456 * dist) + 466.82523;
            return y;
        } else {
            double x = (1.91456 * dist) + (466.82523 - 50);
            return x;
        }
    }

    public double hoodCalc() {
        double quad1, quad2, quad3, quad4;
        double dist = distanceToGoal(currentPose.x, currentPose.y);

        quad4 = 0.00000930357 * Math.pow(dist, 4);
        quad3 = 0.0032622 * Math.pow(dist, 3);
        quad2 = 0.395889 * Math.pow(dist, 2);
        quad1 = (19.05993*dist);

        double pow = (quad4 - quad3 + quad2 - quad1) + 325.42975;
        return 0.08 + ((pow - 20) * 0.87) / 30;
    }

    @Override
    public void runOpMode() {
        initialize();
        waitForStart();
        lightControl("blue");
        imu.resetYaw();

        while (opModeIsActive()) {
            driveController();

            try {
                pinpoint.update();
                Pose2D p = pinpoint.getPosition();

                double odoX = p.getX(DistanceUnit.INCH);
                double odoY = p.getY(DistanceUnit.INCH);

                boolean tagUsed = relocalizeFromAprilTags(odoX, odoY);

                fusedX = odoX + odoOffX;
                fusedY = odoY + odoOffY;

                fusedHeadingDeg = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);

                currentPose.x = fusedX;
                currentPose.y = fusedY;

                telemetry.addData("Tag Reloc", tagUsed ? "USED" : "none");
                telemetry.addData("OdoOff", "x=%.1f y=%.1f", odoOffX, odoOffY);


                telemetry.addData("Tag Reloc", tagUsed ? "USED" : "none");
            } catch (Exception ignored) {}


            shooterGamepad();
            stopper();
            resetServo();
            kicker();
            intake();

            AprilTagDetection tag20 = null;
            for (AprilTagDetection det : aprilTag.getDetections()) {
                if (det.id == 24) {
                    tag20 = det;
                    break;
                }
            }

            if (tag20 != null) {
                telemetry.addData("Tag 20 X", "%.2f", tag20.ftcPose.x);
                telemetry.addData("Tag 20 Y", "%.2f", tag20.ftcPose.y);
                telemetry.addData("Tag 20 Z", "%.2f", tag20.ftcPose.z);
            } else {
                telemetry.addLine("Tag 20 not visible");
            }

            double yawDeg = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
            double fieldAim = angleToGoalDeg(currentPose.x, currentPose.y);
            double turretCmdDeg = wrapDeg(fieldAim - yawDeg);
            telemetry.addData("Yaw deg", "%.1f", yawDeg);
            telemetry.addData("Hood Angle", hoodCalc());
            telemetry.addData("Turret ticks", turretMotor.getCurrentPosition());
            telemetry.addData("Dist to goal (in)", "%.2f", distanceToGoal());

            // ====== UPDATED turret behavior (single place) ======
            if (gamepad1.right_bumper) {
                transfer();
            } else {
                holdTurretRobotZero();

                if (isShooterAtSpeed()) {
                    updateBlinkWhiteIfAtSpeed();
                } else {
                    whiteOn = false;
                    lightControl("blue");
                    ledBlinkTimer.reset();
                }
            }

            // normal shooter control + PIDF update when NOT in transfer
            if (!gamepad1.right_bumper) {
                shooterControl();
                shooter.update();
            }

            telemetry.update();
            sleep(20);
        }

        if (portal != null) {
            portal.close();
        }
        shooter.stop();
    }
}