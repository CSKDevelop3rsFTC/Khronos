package org.firstinspires.ftc.teamcode.teleop;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.ColorRangeSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoImplEx;

import org.firstinspires.ftc.robotcore.external.JavaUtil;

@TeleOp(name = "Master TeleOp", group = "TeleOp")
public class MasterTeleOp extends LinearOpMode {

    private DcMotor fr;
    private DcMotor fl;
    private DcMotor br;
    private DcMotor bl;

    private DcMotor intake;
    private DcMotor intake2;

    private Servo stopper;

    private CRServo liftL;
    private CRServo liftR;

    private CRServo turret;
    private CRServo turret2;

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

    // CLASS STATES - - - - -
    private enum TransferState {IDLE, AIMING, PRE_FEED, FEEDING}
    private enum ShooterMode {OFF, CLOSE, FAR}
    private ShooterMode shooterMode = ShooterMode.OFF;

    // TRANSFER CONSTANTS - - - -
    private TransferState transferState = TransferState.IDLE;
    private long transferStateEndMs = 0;
    private long turretSettledSinceMs = 0;

    // SHOOTER CONSTANTS
    private ShooterPID shooter;

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

        // how long to keep intake running after BOTH sensors stop detecting
        public static long bothHoldMs = 250;

        // NEW: how long to keep intake STOPPED after upper-only detection stops flickering
        public static long upperOnlyStopMs = 250;
    }

    public void initialize() {
        fl = hardwareMap.get(DcMotor.class,"fl");
        bl = hardwareMap.get(DcMotor.class,"bl");
        fr = hardwareMap.get(DcMotor.class,"fr");
        br = hardwareMap.get(DcMotor.class,"br");

        intake = hardwareMap.get(DcMotor.class, "intake");
        intake2 = hardwareMap.get(DcMotor.class, "intake2");
        stopper = hardwareMap.get(Servo.class, "stopper");

        left = hardwareMap.get(ServoImplEx.class, "left");
        right = hardwareMap.get(ServoImplEx.class, "right");
        //top = hardwareMap.get(ServoImplEx.class, "top");

        lower = hardwareMap.get(ColorRangeSensor.class, "lower");
        upper = hardwareMap.get(ColorRangeSensor.class, "upper");

        liftL = hardwareMap.get(CRServo.class, "liftL");
        liftR = hardwareMap.get(CRServo.class, "liftR");

        shooter = new ShooterPID(hardwareMap, "f1M", "f2M");
        shooter.setTelemetry(new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry()));

        pinpoint = hardwareMap.get(GoBildaPinpointDriver.class, "pinpoint");

        fl.setDirection(DcMotorSimple.Direction.REVERSE);
        bl.setDirection(DcMotorSimple.Direction.REVERSE);
        fr.setDirection(DcMotorSimple.Direction.FORWARD);
        br.setDirection(DcMotorSimple.Direction.FORWARD);
        liftR.setDirection(DcMotorSimple.Direction.REVERSE);

        stopper.setPosition(transferConstants.GATE_CLOSED);

        pinpoint.resetPosAndIMU();
    }

    public void driveMovement(double driveSpeed) {
        double horizontal = gamepad1.left_stick_x;
        double vertical = gamepad1.left_stick_y;
        double pivot = gamepad1.right_stick_x * 0.5;

        fr.setPower(((vertical + horizontal) + pivot) * driveSpeed);
        fl.setPower(((vertical - horizontal) - pivot) * driveSpeed);
        br.setPower(((vertical - horizontal) + pivot) * driveSpeed);
        bl.setPower(((vertical + horizontal) - pivot) * driveSpeed);
    }

    public void driveController() {
        if (gamepad1.right_stick_button) driveMovement(0.5);
        else driveMovement(1.0);
    }

    //COLOR DETECTION
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

    private boolean midButtonPressed() {
        return gamepad1.touchpad;
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
                shooter.setTargetRPM(800);
            case FAR:
                shooter.setTargetRPM(1900);
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

    // LIFT CONTROL CODE
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

    // INTAKE/TRANSFER CODE
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

        // hold intake ON until we've gone a full second with no moment where BOTH detect
        boolean bothHoldActive = lastBothDetectedMs != 0 && (now - lastBothDetectedMs) < transferConstants.bothHoldMs;
        if (bothHoldActive) {
            intake.setPower(-transferConstants.intakePower * 1);
            return;
        }

        // NEW: hold intake OFF until we've gone upperOnlyStopMs without UPPER-only detection
        boolean upperStopHoldActive = lastUpperOnlyDetectedMs != 0 && (now - lastUpperOnlyDetectedMs) < transferConstants.upperOnlyStopMs;
        if (upperStopHoldActive) {
            intake2.setPower(0);
            return;
        }

        // existing logic below (unchanged)
        if (isGreenOrPurple(upperColor) && isGreenOrPurple(lowerColor)){
            intake.setPower(-transferConstants.intakePower * 1);
        }

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

        shooter.update();

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
    public void runOpMode() {
        initialize();
        waitForStart();

        while (opModeIsActive()) {
            transferControl();
            driveController();
            shooterGamepad();
            liftControl();

            if (!transferActive()) shooterControl();
            shooter.update();

            if (!transferActive()) {
                intake();
            }

            sleep(20);
        }
    }
}