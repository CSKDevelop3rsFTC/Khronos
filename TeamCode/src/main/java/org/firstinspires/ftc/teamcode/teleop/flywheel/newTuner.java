package org.firstinspires.ftc.teamcode.teleop.flywheel;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.ServoImplEx;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.seattlesolvers.solverslib.controller.PIDFController;

@Config
@TeleOp(name = "LaunchMotorTuner", group = "Motor")

public class newTuner extends LinearOpMode {

    // Tunables from Dashboard
    public static double P = 0.003;
    public static double I = 0.0;
    public static double D = 0.000;
    public static double F = 0.00042;

    // target velocity in ticks per second (or whatever units your encoder reports)
    public static double TARGET_VEL = 0.0;
    public static double POS_TOLERANCE = 0.0;

    private DcMotorEx leftMotor;
    private DcMotorEx rightMotor;

    private ServoImplEx hood;

    public static double hoodAngle = 0.78;

    private DcMotor intake;
    private DcMotor intake2;

    private final PIDFController launcherPIDF = new PIDFController(P, I, D, F);
    private MultipleTelemetry dashTel;
    private ElapsedTime loopTimer;

    @Override
    public void runOpMode() throws InterruptedException {
        // Hardware
        leftMotor  = hardwareMap.get(DcMotorEx.class, "f1M");
        rightMotor = hardwareMap.get(DcMotorEx.class, "f2M");

        intake = hardwareMap.get(DcMotor.class, "intake");
        intake2 = hardwareMap.get(DcMotor.class, "intake2");

        hood = hardwareMap.get(ServoImplEx.class, "hood");

        // Make them spin the same direction
        leftMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        rightMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        leftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        leftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        rightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        // Telemetry to DS + Dashboard
        dashTel = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        launcherPIDF.setTolerance(POS_TOLERANCE, 0.0);

        loopTimer = new ElapsedTime();
        loopTimer.reset();

        waitForStart();

        while (opModeIsActive()) {

            hoodAngle = Math.max(0.2, Math.min(0.99, hoodAngle));
            if (hood != null) {
                hood.setPosition(hoodAngle);
            }
            
            if (gamepad1.left_trigger > 0) {
                intake.setPower(-1);
                intake2.setPower(1);
            } else if (gamepad1.right_trigger > 0) {
                intake.setPower(1);
                intake2.setPower(-1);
            } else {
                intake.setPower(0);
                intake2.setPower(0);
            }

            // timing (optional, just to see loop time)
            double loopMs = loopTimer.milliseconds();
            loopTimer.reset();

            // read velocities (ticks/sec)
            double velLeft  = leftMotor.getVelocity();
            double velRight = rightMotor.getVelocity();
            double motorVel = 0.5 * (velLeft + velRight);

            // battery voltage for F scaling
            double voltage = 12.0;
            if (hardwareMap.voltageSensor.iterator().hasNext()) {
                voltage = hardwareMap.voltageSensor.iterator().next().getVoltage();
            }

            // update gains live from @Config variables
            // scale F by voltage so behavior stays similar as battery sags
            double scaledF = (voltage > 1.0) ? F / (voltage / 12.0) : F;
            launcherPIDF.setPIDF(P, I, D, scaledF);
            launcherPIDF.setTolerance(POS_TOLERANCE, 0.0);
            launcherPIDF.setSetPoint(TARGET_VEL);

            // controller output
            double power = launcherPIDF.calculate(motorVel, TARGET_VEL);

            // clamp
            if (power > 1.0) power = 1.0;
            if (power < 0.0) power = 0.0;

            leftMotor.setPower(power);
            rightMotor.setPower(power);

            // Telemetry
            dashTel.addData("Loop ms", loopMs);
            dashTel.addData("Target vel", TARGET_VEL);
            dashTel.addData("Actual vel", motorVel);
            dashTel.addData("Left vel", velLeft);
            dashTel.addData("Right vel", velRight);
            dashTel.addData("Power", power);
            dashTel.addData("Voltage", voltage);
            dashTel.update();
        }

        leftMotor.setPower(0);
        rightMotor.setPower(0);
    }
}
