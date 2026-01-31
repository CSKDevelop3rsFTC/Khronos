package org.firstinspires.ftc.teamcode.teleop.flywheel;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.teleop.SimplePIDF;

@TeleOp(name = "FlywheelTuner", group = "Shooter")
@Disabled
public class FlywheelTuner extends LinearOpMode {
    @Configurable
    public static class flyConstants {
        public static double kP = 0.00064; // 0.0002
        public static double kI = 0.0; //0.000001
        public static double kD = 0.0; // 0.00002
        public static double kS = 0.0012; // 0.0012
        public static double kV = 0.000208; // 0.0002
        public static double kA = 0.00; // 0
    }

    @Override
    public void runOpMode() throws InterruptedException {
        // Build gains from your constants EXACTLY as written
        SimplePIDF pidf = new SimplePIDF(
                flyConstants.kP,
                flyConstants.kI,
                flyConstants.kD,
                flyConstants.kS,
                flyConstants.kV,
                flyConstants.kA
        );

        // Build the subsystem after hardware is available
        FlywheelSubsystem fly = new FlywheelSubsystem(hardwareMap, "f1M", "f2M", pidf, true);

        double target = 2700; // start point; change in code or with D-pad below
        waitForStart();

        double last = getRuntime();
        while (opModeIsActive()) {
            double now = getRuntime();
            double dt = Math.max(0.001, now - last);
            last = now;

            // quick nudges to test
            if (gamepad1.dpad_up)    target += 100;
            if (gamepad1.dpad_down)  target -= 100;
            if (gamepad1.dpad_right) target += 25;
            if (gamepad1.dpad_left)  target -= 25;
            if (target < 0) target = 0;

            fly.setTargetRPM(target);
            fly.update(dt);

            telemetry.addData("Target", "%.0f", target);
            telemetry.addData("RPM", "%.0f", fly.getMeasuredRPM());
            telemetry.addData("Ready", fly.isReady());
            telemetry.addData("kP", "%.5f", flyConstants.kP);
            telemetry.addData("kI", "%.5f", flyConstants.kI);
            telemetry.addData("kD", "%.5f", flyConstants.kD);
            telemetry.addData("kS", "%.5f", flyConstants.kS);
            telemetry.addData("kV", "%.5f", flyConstants.kV);
            telemetry.addData("kA", "%.5f", flyConstants.kA);
            telemetry.update();

            sleep(20);
        }

        fly.stop();
    }
}
