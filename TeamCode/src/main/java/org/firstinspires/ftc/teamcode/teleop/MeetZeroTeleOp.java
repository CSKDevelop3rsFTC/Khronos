package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoImplEx;

@TeleOp(name="MeetZeroTeleOp", group="Robot")
public class MeetZeroTeleOp extends LinearOpMode {

    //drive train motors
    public DcMotor frontLeftDrive = null;
    public DcMotor frontRightDrive = null;
    public DcMotor backLeftDrive = null;
    public DcMotor backRightDrive = null;
    public CRServo leftFeeder = null;
    public CRServo rightFeeder = null;
    public DcMotorEx launcher = null;

    public double  shooterSpeed = -1;


    public void initialize() {

        frontLeftDrive = hardwareMap.get(DcMotor.class, "frontLeftDrive");
        frontRightDrive = hardwareMap.get(DcMotor.class, "frontRightDrive");
        backLeftDrive = hardwareMap.get(DcMotor.class, "backLeftDrive");
        backRightDrive = hardwareMap.get(DcMotor.class, "backRightDrive");


        frontLeftDrive.setDirection(DcMotorSimple.Direction.FORWARD);
        backLeftDrive.setDirection(DcMotorSimple.Direction.FORWARD);
        frontRightDrive.setDirection(DcMotorSimple.Direction.REVERSE);
        backRightDrive.setDirection(DcMotorSimple.Direction.REVERSE);

        // launcher + feeders
        launcher = hardwareMap.get(DcMotorEx.class, "launcher");
        leftFeeder = hardwareMap.get(CRServo.class, "left_feeder");
        rightFeeder = hardwareMap.get(CRServo.class, "right_feeder");

        launcher.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        launcher.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        launcher.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER,
                new PIDFCoefficients(300, 0, 0, 10)); // tune for your robot

        rightFeeder.setDirection(DcMotorSimple.Direction.REVERSE);
        leftFeeder.setPower(0);
        rightFeeder.setPower(0);

    }

    public void driveMovement(double driveSpeed) {
        double vertical;
        double horizontal;
        double pivot;

        horizontal = gamepad1.left_stick_x;
        vertical = -gamepad1.left_stick_y;
        pivot = -gamepad1.right_stick_x * 0.5;

        frontRightDrive.setPower(((vertical + horizontal) - pivot) * driveSpeed);
        frontLeftDrive.setPower(((vertical - horizontal) + pivot) * driveSpeed);
        backRightDrive.setPower(((vertical - horizontal) - pivot) * driveSpeed);
        backLeftDrive.setPower(((vertical + horizontal) + pivot) * driveSpeed);
    }

    public void shooter() {
        while (gamepad1.left_trigger > 0) {
            leftFeeder.setPower(shooterSpeed);
            rightFeeder.setPower(shooterSpeed);
            launcher.setPower(shooterSpeed);
        }
        if (gamepad1.right_bumper) {
            leftFeeder.setPower(shooterSpeed);
            rightFeeder.setPower(shooterSpeed);
            launcher.setPower(shooterSpeed);
            sleep(2000);
        }
        else {
            leftFeeder.setPower(0);
            rightFeeder.setPower(0);
            launcher.setPower(0);
        }
    }

    @Override
    public void runOpMode() {

        initialize();

        waitForStart();


        while (opModeIsActive()) {

            driveMovement(.5);
            shooter();
            sleep(50);
        }

    }
}