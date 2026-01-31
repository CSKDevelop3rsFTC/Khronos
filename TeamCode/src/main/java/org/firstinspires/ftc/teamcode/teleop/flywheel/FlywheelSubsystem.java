package org.firstinspires.ftc.teamcode.teleop.flywheel;

import com.qualcomm.robotcore.hardware.DcMotorSimple;   // <-- add this
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.VoltageSensor;

import org.firstinspires.ftc.teamcode.teleop.SimplePIDF;

public class FlywheelSubsystem {
    // ======= CONFIG: set these for your hardware =======
    // Example: GoBilda Yellow Jacket 5202 19.2:1 = 103.8 CPR (enc counts/shaft rev) * external ratio
    // Most FTC SDK encoders are 28*cpr*gear = 537.6 or 383.6 etc. Put the true motor ticks per motor shaft rev here.
    public static final double MOTOR_TICKS_PER_REV = 28;   // e.g. 5202 19.2:1
    public static final double MOTOR_FREE_RPM = 6000;        // motor shaft RPM (depends on model)
    public static final double EXTERNAL_RATIO = 1.0;          // motor:wheel ratio. If 3:1 up-gearing to flywheel, EXTERNAL_RATIO = 1/3.0
    // Flywheel RPM = motorRPM * EXTERNAL_RATIO

    // Velocity smoothing
    private static final double RPM_EMA_ALPHA = 0.25;         // 0..1 higher = less smoothing

    // Ready gating
    private static final double READY_TOL_RPM = 40.0;
    private static final int READY_CONSEC_CYCLES = 6;

    private final DcMotorEx m1, m2;
    private final VoltageSensor battery;

    private final SimplePIDF pidf;
    private double targetRPM = 0.0;

    private int lastPos1, lastPos2;
    private boolean first = true;
    private double rpmFiltered = 0.0;
    private int readyCount = 0;

    public FlywheelSubsystem(HardwareMap hw, String motor1Name, String motor2Name,
                             SimplePIDF pidfGains, boolean reverseSecond) {
        this.m1 = hw.get(DcMotorEx.class, motor1Name);
        this.m2 = hw.get(DcMotorEx.class, motor2Name);
        if (reverseSecond) m2.setDirection(DcMotorSimple.Direction.REVERSE);

        // Use encoders in raw power mode; we do our own velocity control.
        m1.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        m2.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        m1.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        m2.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);

        this.battery = hw.voltageSensor.iterator().hasNext()
                ? hw.voltageSensor.iterator().next() : null;

        this.pidf = pidfGains;
        this.pidf.reset();
    }

    public void setTargetRPM(double rpm) {
        targetRPM = Math.max(0.0, rpm);
    }

    public double getTargetRPM() { return targetRPM; }
    public double getMeasuredRPM() { return rpmFiltered; }
    public boolean isReady() { return readyCount >= READY_CONSEC_CYCLES; }

    /**
     * Call every loop.
     * @param dt seconds since last call
     */
    public void update(double dt) {
        // 1) Measure RPM from encoder deltas
        int pos1 = m1.getCurrentPosition();
        int pos2 = m2.getCurrentPosition();
        double rpm1 = ticksToRpm(pos1 - (first ? pos1 : lastPos1), dt);
        double rpm2 = ticksToRpm(pos2 - (first ? pos2 : lastPos2), dt);

        lastPos1 = pos1; lastPos2 = pos2;
        if (first) { first = false; rpmFiltered = 0.0; } // boot
        double rpmRaw = 0.5 * (rpm1 + rpm2);
        rpmFiltered = RPM_EMA_ALPHA * rpmRaw + (1.0 - RPM_EMA_ALPHA) * rpmFiltered;

        // 2) Compute control
        double battV = battery != null ? battery.getVoltage() : 12.0;
        double setAcc = 0.0; // optional kA
        double cmd = pidf.update(targetRPM, rpmFiltered, setAcc, dt, battV);

        // 3) Apply power to both motors
        m1.setPower(cmd);
        m2.setPower(cmd);

        // 4) Update ready gating
        if (Math.abs(targetRPM - rpmFiltered) <= READY_TOL_RPM && targetRPM > 0.0) {
            readyCount = Math.min(READY_CONSEC_CYCLES, readyCount + 1);
        } else {
            readyCount = 0;
        }
    }

    public void stop() {
        targetRPM = 0.0;
        m1.setPower(0);
        m2.setPower(0);
        pidf.reset();
        readyCount = 0;
    }

    private static double ticksToRpm(int deltaTicks, double dt) {
        // motor shaft revs in this interval
        double motorRevs = deltaTicks / MOTOR_TICKS_PER_REV;
        double motorRps = motorRevs / Math.max(dt, 1e-6);
        double motorRpm = motorRps * 60.0;
        // flywheel RPM after external gearing
        return motorRpm * EXTERNAL_RATIO;
    }
}
