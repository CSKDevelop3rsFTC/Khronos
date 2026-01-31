package org.firstinspires.ftc.teamcode.teleop.test;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;

@TeleOp(name = "crservo", group = "Test")
public class crservo extends LinearOpMode {

    private CRServo turret;
    private CRServo turret2;

    // change this if it’s too fast
    private static final double SPEED = 1.0;

    @Override
    public void runOpMode() {
        turret = hardwareMap.get(CRServo.class, "turret");
        turret2 = hardwareMap.get(CRServo.class, "turret2");

        turret.setPower(0);
        turret2.setPower(0);

        waitForStart();

        while (opModeIsActive()) {
            double lt = gamepad1.left_trigger;   // 0..1
            double rt = gamepad1.right_trigger;  // 0..1

            double power = 0.0;

            if (lt > 0.05 && rt <= 0.05) {
                // one direction
                power = SPEED * lt;
            } else if (rt > 0.05 && lt <= 0.05) {
                // opposite direction
                power = -SPEED * rt;
            } else {
                // both pressed or neither pressed: stop
                power = 0.0;
            }

            turret.setPower(power);
            turret2.setPower(power);

            telemetry.addData("turret power", power);
            telemetry.addData("LT", lt);
            telemetry.addData("RT", rt);
            telemetry.update();
        }
    }
}
