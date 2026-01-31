package org.firstinspires.ftc.teamcode.teleop.flywheel;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

@Config
@TeleOp(name = "TurretServosTuner", group = "Servo")
public class TurretServosTuner extends OpMode {

    // ===== Dashboard-tunable gains (power per degree, power per deg*sec, power per deg/sec) =====
    public static double P = 0.00;
    public static double I = 0.00;
    public static double D = 0.00;

    // Simple static feedforward to help break stiction. Applied with sign(error).
    public static double F = 0.00;

    // Minimum magnitude of output once you're outside tolerance (helps overcome stiction).
    public static double MIN_OUTPUT = 0.15;

    // Stop band in degrees.
    public static double POS_TOLERANCE_DEG = 2.0;

    // Integral clamp (in "integral accumulator units", i.e., deg*sec).
    public static double MIN_INTEGRAL = -50.0;
    public static double MAX_INTEGRAL =  50.0;

    // Target turret angle (degrees). Keep in [-180, 180) if you want shortest-path behavior.
    public static double TARGET_DEG = 0.0;

    // Encoder constants.
    // REV Through Bore quadrature is 8192 counts per revolution.
    public static double ENCODER_CPR = 8192.0;

    // If your encoder is not 1:1 with the turret axle, set:
    // encoder revs per turret rev (example: encoder spins 2x for 1 turret rev => 2.0)
    public static double ENCODER_REVS_PER_TURRET_REV = 18.5;

    // If turret2 needs to spin opposite turret (common when servos face each other).
    public static boolean SERVO2_REVERSED = true;

    // Flip encoder direction if your ticks go the "wrong" way.
    public static boolean ENCODER_REVERSED = false;

    private CRServo turret;
    private CRServo turret2;
    private DcMotorEx intake2; // encoder-only "motor" device

    private final ElapsedTime loopTimer = new ElapsedTime();

    // PID state
    private double integral = 0.0;
    private double prevError = 0.0;
    private boolean firstLoop = true;

    @Override
    public void init() {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        turret = hardwareMap.get(CRServo.class, "turret");
        turret2 = hardwareMap.get(CRServo.class, "turret2");

        intake2 = hardwareMap.get(DcMotorEx.class, "intake2");

        intake2.setDirection(ENCODER_REVERSED ? DcMotorSimple.Direction.REVERSE : DcMotorSimple.Direction.FORWARD);

        // We only read the encoder. Resetting here makes "current physical angle" become 0.
        intake2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        intake2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // Make sure turret is not moving on init.
        setTurretPower(0.0);

        loopTimer.reset();
    }

    @Override
    public void loop() {
        double dt = loopTimer.seconds();
        loopTimer.reset();
        if (dt <= 0) dt = 1e-3;

        // Convert encoder ticks to turret angle in degrees
        int ticks = intake2.getCurrentPosition();
        double ticksPerTurretRev = ENCODER_CPR * ENCODER_REVS_PER_TURRET_REV;
        double currentDeg = (ticks / ticksPerTurretRev) * 360.0;

        // Shortest-path error if you keep TARGET_DEG in [-180, 180)
        double errorDeg = wrapDeg(TARGET_DEG - currentDeg);

        boolean atSetpoint = Math.abs(errorDeg) <= POS_TOLERANCE_DEG;

        if (atSetpoint) {
            setTurretPower(0.0);
            integral = 0.0;
            firstLoop = true;
        } else {
            // PID
            integral += errorDeg * dt;
            integral = Range.clip(integral, MIN_INTEGRAL, MAX_INTEGRAL);

            double derivative;
            if (firstLoop) {
                derivative = 0.0;
                firstLoop = false;
            } else {
                derivative = (errorDeg - prevError) / dt;
            }
            prevError = errorDeg;

            double output = (P * errorDeg) + (I * integral) + (D * derivative);

            // Static feedforward with direction of error
            if (F != 0.0) {
                output += Math.signum(errorDeg) * F;
            }

            // Enforce minimum output magnitude outside the tolerance band
            if (Math.abs(output) < MIN_OUTPUT) {
                output = Math.signum(output) * MIN_OUTPUT;
            }

            output = Range.clip(output, -1.0, 1.0);
            setTurretPower(output);
        }

        telemetry.addData("ticks", ticks);
        telemetry.addData("currentDeg", currentDeg);
        telemetry.addData("targetDeg", TARGET_DEG);
        telemetry.addData("errorDeg", errorDeg);
        telemetry.addData("atSetpoint", atSetpoint);
        telemetry.addData("P I D F", "%.4f  %.4f  %.4f  %.4f", P, I, D, F);
        telemetry.addData("MIN_OUTPUT", MIN_OUTPUT);
        telemetry.addData("dt_ms", dt * 1000.0);
        telemetry.update();
    }

    @Override
    public void stop() {
        setTurretPower(0.0);
    }

    private void setTurretPower(double pwr) {
        turret.setPower(pwr);
        turret2.setPower(SERVO2_REVERSED ? -pwr : pwr);
    }

    // Wrap to [-180, 180)
    private double wrapDeg(double deg) {
        deg = deg % 360.0;
        if (deg >= 180.0) deg -= 360.0;
        if (deg < -180.0) deg += 360.0;
        return deg;
    }
}
