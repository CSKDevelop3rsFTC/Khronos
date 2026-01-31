package org.firstinspires.ftc.teamcode.teleop.meet3;

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
import org.firstinspires.ftc.teamcode.teleop.ShooterPID;

@TeleOp(name = "Meet2TeleOp", group = "Robot")
@Disabled
public class Meet3TeleOpSafety extends LinearOpMode {

    public DcMotor frontLeftDrive  = null;
    public DcMotor frontRightDrive = null;
    public DcMotor backLeftDrive   = null;
    public DcMotor backRightDrive  = null;

    public DcMotor intakeMotor = null;
    public DcMotor turretMotor = null;

    public Servo kickerServo  = null;
    public Servo stopperServo = null;

    public ServoImplEx leftLight  = null;
    public ServoImplEx rightLight = null;
    public ServoImplEx top        = null;

    public GoBildaPinpointDriver pinpoint;
    public IMU imu;

    public boolean lightBool;
    public boolean shooterBool;

    // toggle logic
    private boolean prevLeftBumper = false;

    public int pos;

    // Shooter PID subsystem (newTuner-style)
    private ShooterPID shooter;

    // Shooter target RPM (flat, adjustable with dpad up/down)
    private double targetRPM = 532;
    private boolean prevDpadUp = false;
    private boolean prevDpadDown = false;

    // Turret constants (only used for telemetry / debug now)
    private static final double MOTOR_CPR        = 537.6;
    private static final int    GEAR_INPUT_TEETH = 63;
    private static final int    GEAR_OUTPUT_TEETH= 190;
    private static final double GEAR_RATIO       = (double) GEAR_OUTPUT_TEETH / GEAR_INPUT_TEETH;
    private static final double TICKS_PER_TURRET_REV = MOTOR_CPR * GEAR_RATIO;
    private static final double TICKS_PER_DEG    = TICKS_PER_TURRET_REV / 360.0;
    private static final double DEG_PER_TICK     = 1.0 / TICKS_PER_DEG;

    private static final double TURRET_KP      = 0.01; // kept for future if you re-enable PID
    private static final double TURRET_HOLD    = 0.05;
    private static final double TURRET_DEG_TOL = 2.0;

    // turret home angle (for telemetry only now)
    private double turretHomeDeg = 0.0;

    // Manual turret speed (gamepad2 bumpers)
    @Config
    public static class TurretConfig {
        public static double MANUAL_SPEED = 0.15; // change on dashboard if you want faster/slower
    }

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

        intakeMotor = hardwareMap.get(DcMotor.class, "intake");
        turretMotor = hardwareMap.get(DcMotor.class, "turret");

        kickerServo  = hardwareMap.get(Servo.class, "kicker");
        stopperServo = hardwareMap.get(Servo.class, "stopper");

        leftLight  = hardwareMap.get(ServoImplEx.class, "left");
        rightLight = hardwareMap.get(ServoImplEx.class, "right");
        top        = hardwareMap.get(ServoImplEx.class, "top");

        pinpoint = hardwareMap.get(GoBildaPinpointDriver.class, "pinpoint");

        frontLeftDrive.setDirection(DcMotorSimple.Direction.FORWARD);
        backLeftDrive.setDirection(DcMotorSimple.Direction.FORWARD);
        frontRightDrive.setDirection(DcMotorSimple.Direction.REVERSE);
        backRightDrive.setDirection(DcMotorSimple.Direction.REVERSE);

        leftLight.setPwmEnable();
        rightLight.setPwmEnable();

        lightBool = false;
        shooterBool = false;
        prevLeftBumper = false;

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

        turretMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        turretMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        turretMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        imu.initialize(imuParams);
        pinpoint.resetPosAndIMU();

        ledBlinkTimer.reset();
    }

    public void driveMovement(double driveSpeed) {
        double horizontal = gamepad1.left_stick_x;
        double vertical   = -gamepad1.left_stick_y;
        double pivot      = -gamepad1.right_stick_x * 0.5;

        frontRightDrive.setPower(((vertical + horizontal) + pivot) * driveSpeed);
        frontLeftDrive.setPower(((vertical - horizontal) - pivot) * driveSpeed);
        backRightDrive.setPower(((vertical - horizontal) + pivot) * driveSpeed);
        backLeftDrive.setPower(((vertical + horizontal) - pivot) * driveSpeed);
    }

    @Config
    public static class Goal {
        // set these to your real field coordinates for the far goal
        public static double X = 72.0;  // inches example
        public static double Y = 74.0;  // inches example
        public static double TURRET_MAX_POWER = 0.3;   // kept for future if you re-enable PID
    }

    // Wrap an angle to [-180, 180]
    private static double wrapDeg(double a) {
        while (a > 180) a -= 360;
        while (a < -180) a += 360;
        return a;
    }

    // Field angle from robot to goal, in degrees
    private static double angleToGoalDeg(double robotX, double robotY) {
        double dx = Goal.X - robotX;
        double dy = Goal.Y - robotY;
        return Math.toDegrees(Math.atan2(dy, dx));
    }

    private static double distanceToGoal(double robotX, double robotY) {
        double dx = Goal.X - robotX;
        double dy = Goal.Y - robotY;
        return Math.hypot(dx, dy);
    }

    // Convenience overload using currentPose
    private double distanceToGoal() {
        return distanceToGoal(currentPose.x, currentPose.y);
    }

    // within Â±75 RPM of target?
    private boolean isShooterAtSpeed() {
        double target = shooter.getTargetRPM();
        if (target < 100) return false;
        return Math.abs(shooter.getCurrentRPM() - target) <= 75.0;
    }

    // Blink white when at speed
    private void updateBlinkWhiteIfAtSpeed() {
        if (isShooterAtSpeed()) {
            if (ledBlinkTimer.milliseconds() >= 100) {
                whiteOn = !whiteOn;
                lightControl(whiteOn ? "white" : "null");
                ledBlinkTimer.reset();
            }
        }
    }

    // Convert a turret command in degrees to encoder ticks (kept for debug)
    private static int degToTicks(double deg) {
        return (int) Math.round(deg * TICKS_PER_DEG);
    }

    // Closed-loop turret (currently unused, but left here if you want to re-enable PID later)
    private boolean driveTurretToDeg(double targetTurretDeg) {
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

    @Configurable
    @Config
    public static class transferConstants {
        public static int counterConstant = 3;

        public static double intakePower = 1;

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
        else if (gamepad1.left_stick_y > 0)  intakeMotor.setPower(gamepad1.left_stick_y);
        else                                 intakeMotor.setPower(0);
    }

    // toggle shooter on/off with left bumper (GAMEPAD 1)
    public void shooterGamepad() {
        boolean lb = gamepad1.left_bumper;
        if (lb && !prevLeftBumper) {
            shooterBool = !shooterBool;
        }
        prevLeftBumper = lb;
    }

    // Dpad up/down to change targetRPM (GAMEPAD 1)
    private void updateShooterTargetFromDpad() {
        boolean dpadUp   = gamepad1.dpad_up;
        boolean dpadDown = gamepad1.dpad_down;

        if (dpadUp && !prevDpadUp) {
            targetRPM += 40.0;
        }

        if (dpadDown && !prevDpadDown) {
            targetRPM -= 40.0;
            if (targetRPM < 0) targetRPM = 0;
        }

        prevDpadUp   = dpadUp;
        prevDpadDown = dpadDown;
    }

    // manual shooter control using flat RPM
    public void shooterControl() {
        if (shooterBool) {
            shooter.setTargetRPM(targetRPM);
        } else {
            shooter.setTargetRPM(0.0);
        }
    }

    public void kicker() {
        if (gamepad1.x) kickerServo.setPosition(0);
        if (gamepad1.b) kickerServo.setPosition(0.5);
    }

    public void stopper() {
        if (gamepad1.y) stopperServo.setPosition(0.5);
        if (gamepad1.a) stopperServo.setPosition(0);
    }

    // Manual turret control on GAMEPAD 2 bumpers
    // left_bumper  â†’ move turret left
    // right_bumper â†’ move turret right
    // On release, power = 0, BRAKE holds position
    private void turretManualControl() {
        double power = 0.0;

        if (gamepad2.left_bumper) {
            // This matches the previous "left" direction from dpad_left
            power = TurretConfig.MANUAL_SPEED;
        } else if (gamepad2.right_bumper) {
            power = -TurretConfig.MANUAL_SPEED;
        } else {
            power = 0.0; // BRAKE holds position
        }

        turretMotor.setPower(power);
    }

    // "sleep" while keeping the shooter PIDF running
    private void sleepWithShooter(long millis) {
        long end = System.currentTimeMillis() + millis;
        while (opModeIsActive() && System.currentTimeMillis() < end) {
            shooter.update();
            updateBlinkWhiteIfAtSpeed();
            telemetry.update();
        }
    }

    // Shooting sequence (turret stays where driver last put it)
    public void transfer() {
        if (gamepad1.right_bumper) {

            // Stop turret movement and let BRAKE hold
            turretMotor.setPower(0.0);

            // 1) Open and fire sequence (timings from transferConstants)

            stopperServo.setPosition(0);
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

            whiteOn = false;
            lightControl("blue");
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

    // =========================
    //  CORNER LOCALIZATION
    // =========================

    public static class Pose {
        public double x;
        public double y;
        public double heading;  // radians

        public Pose(double x, double y, double heading) {
            this.x = x;
            this.y = y;
            this.heading = heading;
        }
    }

    private static final double HALF_FIELD = 72.0; // inches

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

    @Override
    public void runOpMode() {
        initialize();
        waitForStart();
        lightControl("blue");
        imu.resetYaw();

        // Capture turret home angle at the start (for telemetry only now)
        int startTicks = turretMotor.getCurrentPosition();
        turretHomeDeg = startTicks * DEG_PER_TICK;

        while (opModeIsActive()) {
            driveController();

            try {
                pinpoint.update();
                Pose2D p = pinpoint.getPosition();
                currentPose.x = p.getX(DistanceUnit.INCH);
                currentPose.y = p.getY(DistanceUnit.INCH);
            } catch (Exception ignored) {}

            shooterGamepad();
            stopper();
            kicker();
            intake();

            // Manual turret control on gamepad2
            turretManualControl();

            // Adjust shooter RPM with dpad up/down (gamepad1)
            updateShooterTargetFromDpad();

            double yawDeg       = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
            double fieldAim     = angleToGoalDeg(currentPose.x, currentPose.y);
            double turretCmdDeg = wrapDeg(fieldAim - yawDeg);

            telemetry.addData("Yaw deg", "%.1f", yawDeg);
            telemetry.addData("Aim field deg", "%.1f", fieldAim);
            telemetry.addData("Turret cmd deg (unused aim)", "%.1f", turretCmdDeg);
            telemetry.addData("x", pinpoint.getPosX(DistanceUnit.INCH));
            telemetry.addData("y", pinpoint.getPosY(DistanceUnit.INCH));
            telemetry.addData("Turret ticks", turretMotor.getCurrentPosition());
            telemetry.addData("Turret home deg", "%.2f", turretHomeDeg);
            telemetry.addData("Dist to goal (in)", "%.2f", distanceToGoal());
            telemetry.addData("Target RPM", "%.1f", targetRPM);

            // run transfer sequence if RB is pressed (this internally updates shooter)
            if (gamepad1.right_bumper) {
                transfer();
            } else {
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

        shooter.stop();
    }
}