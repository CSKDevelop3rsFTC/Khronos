package org.firstinspires.ftc.teamcode.paths;


import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.limelightvision.LLResult;

@Autonomous(name = "Limelight Ball Tracker", group = "Auto")
public class BallSeek extends LinearOpMode {

    private DcMotor frontLeftDrive, frontRightDrive, backLeftDrive, backRightDrive;
    private Limelight3A limelight;

    // Control constants
    private static final double DRIVE_GAIN = 0.02;    // Adjust for forward speed responsiveness
    private static final double TURN_GAIN  = 0.03;    // Adjust for how strongly it turns
    private static final double AREA_STOP_THRESHOLD = 8.0;  // Stop when area is this big (% of image)

    @Override
    public void runOpMode() {
        // Initialize hardware
        frontLeftDrive = hardwareMap.get(DcMotor.class, "frontLeftDrive");
        frontRightDrive = hardwareMap.get(DcMotor.class, "frontRightDrive");
        backLeftDrive = hardwareMap.get(DcMotor.class, "backLeftDrive");
        backRightDrive = hardwareMap.get(DcMotor.class, "backRightDrive");

        frontLeftDrive.setDirection(DcMotor.Direction.REVERSE);
        backLeftDrive.setDirection(DcMotor.Direction.REVERSE);

        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.start();

        telemetry.addLine("Initialized — Waiting for start");
        telemetry.update();
        waitForStart();

        while (opModeIsActive()) {
            LLResult result = limelight.getLatestResult();

            if (result != null && result.isValid()) {
                double tx = result.getTx();  // Horizontal offset (degrees)
                double ty = result.getTy();  // Vertical offset (degrees)
                double ta = result.getTa();  // Target area (0–100%)

                // --- Compute movement ---
                double turnPower = tx * TURN_GAIN;
                double drivePower = DRIVE_GAIN * (1.0 - (ta / AREA_STOP_THRESHOLD));

                // Clamp drivePower so it doesn’t reverse when close
                if (drivePower < 0) drivePower = 0;

                double leftPower = drivePower + turnPower;
                double rightPower = drivePower - turnPower;

                // Apply motor powers
                setMotorPowers(leftPower, rightPower);

                telemetry.addData("Limelight Target", "Detected");
                telemetry.addData("tx (deg)", tx);
                telemetry.addData("ty (deg)", ty);
                telemetry.addData("ta (area)", ta);
                telemetry.addData("Drive Power", drivePower);
                telemetry.addData("Turn Power", turnPower);
            } else {
                // No targets — stop motors
                setMotorPowers(0, 0);
                telemetry.addData("Limelight", "No Targets");
            }

            telemetry.update();
        }

        // Stop after loop ends
        setMotorPowers(0, 0);
    }

    // Helper function to set drive powers
    private void setMotorPowers(double leftPower, double rightPower) {
        frontLeftDrive.setPower(leftPower);
        backLeftDrive.setPower(leftPower);
        frontRightDrive.setPower(rightPower);
        backRightDrive.setPower(rightPower);
    }
}