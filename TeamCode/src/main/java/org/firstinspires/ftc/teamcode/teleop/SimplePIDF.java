package org.firstinspires.ftc.teamcode.teleop;

public class SimplePIDF {
    public double kP, kI, kD;
    // Feedforward split for clarity. Use what you need.
    public double kS, kV, kA;

    // Integral clamp to limit windup
    private double iMin = -0.3, iMax = 0.3;
    private double integral, prevError;
    private boolean first = true;

    // Optional output rate limit to reduce jitter
    private double maxDeltaPerSec = 2.0; // change in output per second
    private double lastOutput = 0.0;

    public SimplePIDF(double kP, double kI, double kD, double kS, double kV, double kA) {
        this.kP = kP; this.kI = kI; this.kD = kD;
        this.kS = kS; this.kV = kV; this.kA = kA;
    }

    public void setIntegralClamp(double min, double max) { iMin = min; iMax = max; }
    public void setMaxDeltaPerSec(double v) { maxDeltaPerSec = v; }

    public void reset() {
        integral = 0.0;
        prevError = 0.0;
        first = true;
        lastOutput = 0.0;
    }

    /**
     * @param setVel   target velocity (RPM)
     * @param actVel   measured velocity (RPM)
     * @param setAcc   desired acceleration (RPM/s) if you want kA, else 0
     * @param dt       loop time in seconds
     * @param battV    measured battery voltage in Volts (for comp), pass 12.0 to disable
     * @return motor command in [-1, 1]
     */
    public double update(double setVel, double actVel, double setAcc, double dt, double battV) {
        double error = setVel - actVel;

        // PID
        if (first) {
            prevError = error;
            first = false;
        }
        integral += error * dt;
        integral = clamp(integral, iMin / Math.max(kI, 1e-9), iMax / Math.max(kI, 1e-9)); // scale clamp roughly

        double deriv = (error - prevError) / Math.max(dt, 1e-6);
        prevError = error;

        double pid = kP * error + kI * integral + kD * deriv;

        // FF: kS + kV*v + kA*a. For flywheel, kS small, kV dominates.
        double ff = kS * sgn(setVel) + kV * setVel + kA * setAcc;

        // Combine then voltage compensate
        double raw = pid + ff;

        double voltageComp = battV > 1.0 ? (12.0 / battV) : 1.0;
        double cmd = raw * voltageComp;

        // Optional slew-rate limit to keep output smooth
        double maxDelta = maxDeltaPerSec * dt;
        double delta = clamp(cmd - lastOutput, -maxDelta, maxDelta);
        lastOutput += delta;

        return clamp(lastOutput, -1.0, 1.0);
    }

    private static double clamp(double v, double lo, double hi) {
        return Math.max(lo, Math.min(hi, v));
    }
    private static double sgn(double v) {
        return v >= 0 ? 1.0 : -1.0;
    }
}
