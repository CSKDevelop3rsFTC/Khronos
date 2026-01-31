package org.firstinspires.ftc.teamcode.teleop.test;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

@Config
@TeleOp(name = "Turret Hold", group = "Test")
public class crLogicTest extends LinearOpMode {

    private CRServo turret;
    private CRServo turret2;

    // Encoder device. In the RC config, this must be a DcMotor name whose encoder port
    // is where the REV Through Bore is plugged in.
    private DcMotorEx turretEnc;

    // ===== Dashboard-tunable settings =====

    public static double TARGET_DEG = 0.0;

    public static double DEADBAND_DEG = 2.5;
    public static double SLOW_ZONE_DEG = 6.0;

    public static double FAST_POWER = 1.0;
    public static double SLOW_POWER = 0.22;

    // turret2 mirrored mounting
    public static boolean TURR2_INVERT = false;

    // If pushing right makes currentDeg increase when you expected it to decrease, flip this.
    public static boolean INVERT_ENCODER = false;

    // If error sign looks correct but the turret moves the wrong direction, flip this.
    public static boolean INVERT_OUTPUT = true;

    public static double ENCODER_CPR = 8192.0;

    // Adjust if your gear math is inverted.
    public static double ENCODER_TO_TURRET = 12.0 / 222.0;

    public static boolean USE_SOFT_LIMITS = true;
    public static double MIN_DEG = -90.0;
    public static double MAX_DEG = 90.0;

    // Prevent pushing harder into limits (good for cables).
    public static double LIMIT_MARGIN_DEG = 1.0;

    @Override
    public void runOpMode() {
        turret = hardwareMap.get(CRServo.class, "turret");
        turret2 = hardwareMap.get(CRServo.class, "turret2");

        // You had "intake2" here, keeping it as-is.
        // Change this string if your encoder device name is different.
        turretEnc = hardwareMap.get(DcMotorEx.class, "intake2");

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        turretEnc.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        turretEnc.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        waitForStart();

        while (opModeIsActive()) {
            double target = TARGET_DEG;
            if (USE_SOFT_LIMITS) target = clamp(target, MIN_DEG, MAX_DEG);

            double current = getTurretDegWrapped();
            double error = angleErrorDeg(target, current);

            double absErr = Math.abs(error);
            double cmd;

            if (absErr <= DEADBAND_DEG) {
                cmd = 0.0;
            } else if (absErr <= SLOW_ZONE_DEG) {
                cmd = Math.copySign(SLOW_POWER, error);
            } else {
                cmd = Math.copySign(FAST_POWER, error);
            }

            // Flip output direction if needed
            if (INVERT_OUTPUT) cmd *= -1.0;

            // Cable safety: if at/near a limit, block commands that push further out.
            if (USE_SOFT_LIMITS) {
                if (current >= (MAX_DEG - LIMIT_MARGIN_DEG) && cmd > 0) cmd = 0.0;
                if (current <= (MIN_DEG + LIMIT_MARGIN_DEG) && cmd < 0) cmd = 0.0;
            }

            turret.setPower(cmd);
            turret2.setPower(TURR2_INVERT ? -cmd : cmd);

            telemetry.addData("targetDeg", "%.2f", target);
            telemetry.addData("currentDeg", "%.2f", current);
            telemetry.addData("errorDeg", "%.2f", error);
            telemetry.addData("cmd", "%.2f", cmd);
            telemetry.addData("invertEnc", INVERT_ENCODER);
            telemetry.addData("invertOut", INVERT_OUTPUT);
            telemetry.update();
        }
    }

    private double getTurretDegWrapped() {
        int ticks = turretEnc.getCurrentPosition();
        if (INVERT_ENCODER) ticks *= -1;

        double encoderRevs = ticks / ENCODER_CPR;
        double encoderDeg = encoderRevs * 360.0;
        double turretDeg = encoderDeg * ENCODER_TO_TURRET;
        return wrap180(turretDeg);
    }

    private static double wrap180(double deg) {
        deg = deg % 360.0;
        if (deg >= 180.0) deg -= 360.0;
        if (deg < -180.0) deg += 360.0;
        return deg;
    }

    private static double angleErrorDeg(double targetDeg, double currentDeg) {
        return wrap180(targetDeg - currentDeg);
    }

    private static double clamp(double v, double lo, double hi) {
        return Math.max(lo, Math.min(hi, v));
    }
}