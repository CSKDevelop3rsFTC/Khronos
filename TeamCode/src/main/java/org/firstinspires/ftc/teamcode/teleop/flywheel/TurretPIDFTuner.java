package org.firstinspires.ftc.teamcode.teleop.flywheel;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;

@Config
@TeleOp(name = "Turret PIDF Tuner", group = "Test")
public class TurretPIDFTuner extends OpMode {

    // Hardware
    private CRServo turret;
    private CRServo turret2;
    private DcMotorEx turretEncoder;

    // =======================
    // Keep your existing constants
    // =======================
    public static double ENCODER_CPR = 8192.0;
    public static double ENCODER_TURNS_PER_TURRET_TURN = 222.0 / 12.0;
    public static int ENCODER_SIGN = 1;

    public static double TURRET_SERVO_SIGN = -1.0;
    public static double TURRET2_SERVO_SIGN = -1.0;

    // Cable-safe window (UNWRAPPED degrees)
    public static double MIN_UNWRAPPED_DEG = -142.5;
    public static double MAX_UNWRAPPED_DEG = 217.5;
    public static double LIMIT_MARGIN_DEG = 2.0;

    // =======================
    // PIDF knobs (Dashboard)
    // =======================
    public static boolean ENABLE_PID = true;

    public static double kP = 0.018;
    public static double kI = 0.0;
    public static double kD = 0.0;

    // F here is a static feedforward “kick” to overcome stiction.
    // Applied as kF * sign(error) when error is outside DEADBAND_DEG.
    public static double kF = 0.08;

    public static boolean USE_F = true;

    public static double MAX_OUTPUT = 1;
    public static double MIN_OUTPUT = 0.0; // if you want a minimum “move” power, set to e.g. 0.08

    public static double DEADBAND_DEG = 2;

    // Target setpoint in unwrapped degrees
    public static double TARGET_UNWRAPPED_DEG = 0.0;

    // Gamepad setpoint nudges
    public static double NUDGE_DEG = 1.0;

    // Optional: slew-rate limit output to reduce chatter (set 0 to disable)
    public static double OUTPUT_SLEW_PER_SEC = 0.0;

    // =======================
    // Internal state
    // =======================
    private int zeroTicks = 0;

    private double integral = 0.0;
    private double prevError = 0.0;

    private final ElapsedTime loopTimer = new ElapsedTime();
    private double lastOutput = 0.0;

    @Override
    public void init() {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        turret = hardwareMap.get(CRServo.class, "turret");
        turret2 = hardwareMap.get(CRServo.class, "turret2");

        turretEncoder = hardwareMap.get(DcMotorEx.class, "intake2");
        turretEncoder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        turretEncoder.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        zeroTicks = 0;
        loopTimer.reset();
        setTurretPower(0);
    }

    @Override
    public void loop() {
        double dt = loopTimer.seconds();
        loopTimer.reset();
        if (dt <= 0) dt = 1e-3;

        // Zero encoder reference
        if (gamepad1.y) {
            zeroTicks = turretEncoder.getCurrentPosition();
            integral = 0;
            prevError = 0;
        }

        // Nudge setpoint with dpad
        if (gamepad1.dpad_left)  TARGET_UNWRAPPED_DEG -= NUDGE_DEG;
        if (gamepad1.dpad_right) TARGET_UNWRAPPED_DEG += NUDGE_DEG;

        // Clamp target to effective safe window
        double minEff = MIN_UNWRAPPED_DEG + LIMIT_MARGIN_DEG;
        double maxEff = MAX_UNWRAPPED_DEG - LIMIT_MARGIN_DEG;
        TARGET_UNWRAPPED_DEG = clamp(TARGET_UNWRAPPED_DEG, minEff, maxEff);

        // Read position
        int ticks = turretEncoder.getCurrentPosition();
        int ticksFromZero = (ticks - zeroTicks) * ENCODER_SIGN;
        double posDeg = turretDegFromTicksUnwrapped(ticksFromZero);

        // Error in unwrapped degrees
        double error = TARGET_UNWRAPPED_DEG - posDeg;

        double output = 0.0;

        if (ENABLE_PID) {
            // Deadband stops hunting near target
            if (Math.abs(error) <= DEADBAND_DEG) {
                integral = 0.0;
                output = 0.0;
            } else {
                // Integrator with simple clamp to keep it sane if you ever use kI
                integral += error * dt;
                integral = clamp(integral, -200.0, 200.0);

                double derivative = (error - prevError) / dt;

                double ff = 0.0;
                if (USE_F) {
                    ff = kF * Math.signum(error);
                }

                output = (kP * error) + (kI * integral) + (kD * derivative) + ff;

                // Enforce minimum move power if desired
                if (MIN_OUTPUT > 0.0) {
                    double s = Math.signum(output);
                    if (Math.abs(output) < MIN_OUTPUT) output = s * MIN_OUTPUT;
                }

                output = clamp(output, -MAX_OUTPUT, MAX_OUTPUT);
                prevError = error;
            }
        }

        // Optional output slew-rate limiting
        if (OUTPUT_SLEW_PER_SEC > 0.0) {
            double maxDelta = OUTPUT_SLEW_PER_SEC * dt;
            output = clamp(output, lastOutput - maxDelta, lastOutput + maxDelta);
        }
        lastOutput = output;

        // Safety: if you ever drift out of hard window, stop pushing farther out
        boolean outsideHard = (posDeg < MIN_UNWRAPPED_DEG) || (posDeg > MAX_UNWRAPPED_DEG);
        if (outsideHard) {
            if (posDeg < MIN_UNWRAPPED_DEG && output < 0) output = 0;
            if (posDeg > MAX_UNWRAPPED_DEG && output > 0) output = 0;
        }

        setTurretPower(output);

        telemetry.addData("posDeg", "%.2f", posDeg);
        telemetry.addData("targetDeg", "%.2f", TARGET_UNWRAPPED_DEG);
        telemetry.addData("errorDeg", "%.2f", error);
        telemetry.addData("kP", kP);
        telemetry.addData("kF", kF);
        telemetry.addData("out", "%.3f", output);
        telemetry.addData("dt(ms)", "%.1f", dt * 1000.0);
        telemetry.addLine("Y: zero encoder. Dpad L/R: nudge target.");
        telemetry.update();
    }

    @Override
    public void stop() {
        setTurretPower(0);
    }

    private void setTurretPower(double ccwPositivePower) {
        turret.setPower(ccwPositivePower * TURRET_SERVO_SIGN);
        turret2.setPower(ccwPositivePower * TURRET2_SERVO_SIGN);
    }

    private double turretDegFromTicksUnwrapped(int ticksFromZero) {
        double ticksPerTurretRev = ENCODER_CPR * ENCODER_TURNS_PER_TURRET_TURN;
        return (ticksFromZero / ticksPerTurretRev) * 360.0;
    }

    private double clamp(double v, double lo, double hi) {
        if (v < lo) return lo;
        if (v > hi) return hi;
        return v;
    }
}
