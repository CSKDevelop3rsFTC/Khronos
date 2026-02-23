package org.firstinspires.ftc.teamcode.teleop;

import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.seattlesolvers.solverslib.controller.PIDFController;

/**
 * Shooter controller that uses EXACTLY the same PIDF logic as your newTuner opmode.
 * TARGET_VEL is treated as "target RPM" (same units as getVelocity()).
 */
public class ShooterPID {

    // Same tunables as in newTuner
    public static double P = 0.003;
    public static double I = 0.0;
    public static double D = 0.0005;
    public static double F = 0.00042;

    // "RPM" / velocity units, same as TARGET_VEL in newTuner
    public static double TARGET_VEL = 0.0;
    public static double POS_TOLERANCE = 0.0;

    private final DcMotorEx leftMotor;
    private final DcMotorEx rightMotor;
    private final PIDFController launcherPIDF;

    // OPTIONAL: can be set from your OpMode if you want dashboard telemetry
    private MultipleTelemetry dashTel = null;

    private final VoltageSensor battery;

    // last measured velocity (same units as getVelocity())
    private double lastVel = 0.0;

    public ShooterPID(HardwareMap hardwareMap, String leftName, String rightName) {
        // Hardware: EXACT motor setup as newTuner
        leftMotor  = hardwareMap.get(DcMotorEx.class, leftName);
        rightMotor = hardwareMap.get(DcMotorEx.class, rightName);

        // Make them spin the same direction
        leftMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        rightMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        leftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        leftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        rightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        launcherPIDF = new PIDFController(P, I, D, F);
        launcherPIDF.setTolerance(POS_TOLERANCE, 0.0);

        VoltageSensor vs = null;
        if (hardwareMap.voltageSensor.iterator().hasNext()) {
            vs = hardwareMap.voltageSensor.iterator().next();
        }
        battery = vs;
    }

    // Call this once from your OpMode if you want dashboard telemetry
    public void setTelemetry(MultipleTelemetry telemetry) {
        this.dashTel = telemetry;
    }

    // ===== API for TeleOp =====

    // treat this as "target RPM", 1:1 with TARGET_VEL
    public void setTargetRPM(double rpm) {
        if (rpm < 0) rpm = 0;
        TARGET_VEL = rpm;
    }

    public double getTargetRPM() {
        return TARGET_VEL;
    }

    public double getCurrentRPM() {
        return lastVel;
    }

    /**
     * Call once per loop. This is literally the body of your newTuner while-loop,
     * minus the required telemetry.
     */
    public void update() {
        // read velocities ("ticks/sec" or "rpm" – same units as newTuner)
        double velLeft  = leftMotor.getVelocity();
        double velRight = rightMotor.getVelocity();
        double motorVel = 0.5 * (velLeft + velRight);

        lastVel = motorVel;

        // battery voltage for F scaling
        double voltage = 12.0;
        if (battery != null) {
            voltage = battery.getVoltage();
        }

        // update gains live from P/I/D/F
        // scale F by voltage so behavior stays similar as battery sags
        double scaledF = (voltage > 1.0) ? F / (voltage / 12.0) : F;
        launcherPIDF.setPIDF(P, I, D, scaledF);
        launcherPIDF.setTolerance(POS_TOLERANCE, 0.0);
        launcherPIDF.setSetPoint(TARGET_VEL);

        // controller output (same call)
        double power = launcherPIDF.calculate(motorVel, TARGET_VEL);

        // clamp (0..1 for flywheel)
        if (power > 1.0) power = 1.0;
        if (power < 0.0) power = 0.0;

        leftMotor.setPower(power);
        rightMotor.setPower(power);

        // only log if telemetry was actually set
        if (dashTel != null) {
            //dashTel.addData("Target vel", TARGET_VEL);
            //dashTel.addData("Actual vel", motorVel);
            //dashTel.addData("Power", power);
            //dashTel.addData("Voltage", voltage);
            dashTel.update();
        }
    }

    public void stop() {
        TARGET_VEL = 0.0;
        leftMotor.setPower(0.0);
        rightMotor.setPower(0.0);
    }
}
