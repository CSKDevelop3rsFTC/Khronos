package org.firstinspires.ftc.teamcode.teleop.flywheel;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name = "MeasureFlywheelTPS", group = "Test")

public class MeasureFlywheelTPS extends OpMode {

    private DcMotorEx m1, m2;

    private int lastPos1 = 0;
    private int lastPos2 = 0;
    private boolean firstLoop = true;

    private final ElapsedTime timer = new ElapsedTime();

    // YOUR motor ticks per revolution (5203 1:1 = 28 PPR)
    private static final double TICKS_PER_REV = 28.0;

    @Override
    public void init() {
        m1 = hardwareMap.get(DcMotorEx.class, "f1M");
        m2 = hardwareMap.get(DcMotorEx.class, "f2M");

        // reverse one so they spin the same direction
        m1.setDirection(DcMotorSimple.Direction.REVERSE);
        m2.setDirection(DcMotorSimple.Direction.FORWARD);

        m1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        m2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        m1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        m2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        timer.reset();
        firstLoop = true;
    }

    @Override
    public void start() {
        m1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        m2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        m1.setPower(1);
        m2.setPower(1);

        timer.reset();
    }

    @Override
    public void stop() {
        m1.setPower(0.0);
        m2.setPower(0.0);
    }


    @Override
    public void loop() {

        double dt = timer.seconds();
        if (dt <= 0) dt = 1e-3;
        timer.reset();

        int p1 = m1.getCurrentPosition();
        int p2 = m2.getCurrentPosition();

        double tps1, tps2;

        if (firstLoop) {
            tps1 = tps2 = 0;
            firstLoop = false;
        } else {
            tps1 = (p1 - lastPos1) / dt;   // ticks per second
            tps2 = (p2 - lastPos2) / dt;
        }

        lastPos1 = p1;
        lastPos2 = p2;

        double avgTps = 0.5 * (tps1 + tps2);

        // Display readings on Driver Hub
        telemetry.addData("TPS1", "%.1f", tps1);
        telemetry.addData("TPS2", "%.1f", tps2);
        telemetry.addData("Avg TPS", "%.1f", avgTps);
        telemetry.update();
    }
}
