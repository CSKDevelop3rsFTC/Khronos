package org.firstinspires.ftc.teamcode.auto;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.ColorRangeSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.JavaUtil;
import org.firstinspires.ftc.teamcode.pedropathing.Constants;
import com.pedropathing.paths.PathConstraints;

import java.util.ArrayList;
import java.util.List;

/**
 * blue18 (shooting disabled)
 *
 * Changes you requested:
 *  - ALL shooting/transfer sequences are disabled.
 *  - Anywhere the code used to "shoot", it now does a 0.75 second delay.
 *  - Intake + pathing still run as before (color-sensor intake logic).
 */
@Autonomous(name = "farBlue", group = "Autonomous")
public class farBlue extends OpMode {

    // -------------------------
    // Pedro + paths
    // -------------------------
    private Follower follower;

    private PathChain preload;

    private PathChain toRow2Pre, intakeRow2, intakeBackup, toShootFromRow2;
    private PathChain shootToGate, gateIntake, gateBackup, gateToShoot;

    private PathChain toBackIntake, reset, toShootFromBack;
    private PathChain toRow1Pre, intakeRow1, toShootFromRow1;
    private PathChain toRow3Pre, intakeRow3, toShootFromRow3;

    private PathChain parkPath;

    // -------------------------
    // Hardware
    // -------------------------
    private DcMotor intake;
    private DcMotor intake2;

    private NormalizedColorSensor lower;
    private NormalizedColorSensor upper;

    private Servo stopper; // gate (kept closed since shooting disabled)

    // -------------------------
    // Step queue framework
    // -------------------------
    interface Step {
        void init();
        void update();
        boolean isFinished();
    }

    private static final PathConstraints SLOW = new PathConstraints(
            18,
            33,
            3.0,
            4.0
    );
    static class StepRunner {
        private final List<Step> steps;
        private int idx = 0;
        private boolean inited = false;

        StepRunner(List<Step> steps) {
            this.steps = steps;
        }

        void run() {
            if (idx >= steps.size()) return;

            Step s = steps.get(idx);
            if (!inited) {
                s.init();
                inited = true;
            }
            s.update();
            if (s.isFinished()) {
                idx++;
                inited = false;
            }
        }

        boolean isDone() { return idx >= steps.size(); }
        int getIndex() { return idx; }
    }

    private final List<Step> steps = new ArrayList<>();
    private StepRunner runner;

    // -------------------------
    // Intake intent flags
    // -------------------------
    private boolean intakeNow = false;
    private boolean reverseIntake = false; // normally false in auto

    // Sensor timestamps (copied behavior)
    private long lastUpperDetectMs = 0;
    private long lastLowerDetectMs = 0;

    // -------------------------
    // Constants
    // -------------------------
    public static class transferConstants {
        public static double intakePower = -1.0;  // same sign usage as your TeleOp
        public static double GATE_CLOSED = 0.1;   // keep closed while shooting disabled
    }

    private static final double GATE_WAIT_SEC = 1.5;

    // NEW: replacement "shoot" delay
    private static final double NO_SHOOT_DELAY_SEC = 0.5;

    // -------------------------
    // Poses
    // -------------------------
    private final Pose startPose = new Pose(12, 18, Math.toRadians(0));

    // “shoot” pose (still used as a waypoint)
    private final Pose shoot      = new Pose(18, 18, Math.toRadians(90));

    // 2nd row
    private final Pose preintakeOne = new Pose(60, 22, Math.toRadians(90));
    private final Pose intakePose1  = new Pose(60, 66, Math.toRadians(90));

    // gate
    private final Pose gate = new Pose(63.5, 68.5, Math.toRadians(90));
    private final Pose rotate = new Pose(58, 71, Math.toRadians(20));
    private final Pose gate1 = new Pose(57.5, 75, Math.toRadians(15));
    private final Pose backup = new Pose(57, 70, Math.toRadians(90));

    // 1st row
    private final Pose preintake  = new Pose(84, 22, Math.toRadians(90));
    private final Pose intakePose = new Pose(84, 56, Math.toRadians(90));

    // 3rd row
    private final Pose preintakeTwo = new Pose(36, 22, Math.toRadians(90));
    private final Pose intakePose2  = new Pose(36, 66, Math.toRadians(90));
    // backsite
    private final Pose preintakeFour = new Pose(12, 60, Math.toRadians(90));
    private final Pose intakePose4  = new Pose(9, 68, Math.toRadians(90));
    // park
    private final Pose parkPose = new Pose(12, 35, Math.toRadians(90));
    private Step followWithMaxPower(PathChain path, boolean intakeOn, double maxPower) {
        return new Step() {
            @Override public void init() {
                intakeNow = intakeOn;
                reverseIntake = false;

                // Slow down ONLY for this path
                follower.setMaxPower(maxPower);  // <- if your follower supports it
                follower.followPath(path, true);
            }
            @Override public void update() {}
            @Override public boolean isFinished() {
                boolean done = !follower.isBusy();
                if (done) follower.setMaxPower(1.0); // restore normal speed
                return done;
            }
        };
    }

    // -------------------------
    // Build paths
    // -------------------------
    private void buildPaths() {



        // 2nd row
        toRow2Pre = follower.pathBuilder()
                .addPath(new BezierCurve(startPose, preintakeOne, preintakeOne))
                .setLinearHeadingInterpolation(startPose.getHeading(), preintakeOne.getHeading(), preintakeOne.getHeading())
                .build();

        intakeRow2 = follower.pathBuilder()
                .addPath(new BezierLine(preintakeOne, intakePose1))
                .setLinearHeadingInterpolation(preintakeOne.getHeading(), intakePose1.getHeading())
                .build();
        intakeBackup = follower.pathBuilder()
                .addPath(new BezierCurve(intakePose1, preintakeOne, gate))
                .setLinearHeadingInterpolation(intakePose1.getHeading(), preintakeOne.getHeading(), gate.getHeading())
                .build();
        toShootFromRow2 = follower.pathBuilder()
                .addPath(new BezierCurve(intakePose1, preintakeOne, shoot))
                .setLinearHeadingInterpolation(intakePose1.getHeading(), preintakeOne.getHeading(), shoot.getHeading())
                .build();

        // gate loop
        shootToGate = follower.pathBuilder()
                .addPath(new BezierLine(shoot, gate))
                .setLinearHeadingInterpolation(shoot.getHeading(), gate.getHeading())
                .setConstraints(SLOW)
                .build();


        gateToShoot = follower.pathBuilder()
                .addPath(new BezierCurve(gate1, backup, shoot))
                .setLinearHeadingInterpolation(gate1.getHeading(), backup.getHeading(), shoot.getHeading())
                .build();

        // 1st row
        toRow1Pre = follower.pathBuilder()
                .addPath(new BezierCurve(shoot, preintake, preintake))
                .setLinearHeadingInterpolation(shoot.getHeading(), preintake.getHeading(), preintake.getHeading())
                .build();

        intakeRow1 = follower.pathBuilder()
                .addPath(new BezierLine(preintake, intakePose))
                .setLinearHeadingInterpolation(preintake.getHeading(), intakePose.getHeading())
                .build();

        toShootFromRow1 = follower.pathBuilder()
                .addPath(new BezierCurve(intakePose, preintake, shoot))
                .setLinearHeadingInterpolation(intakePose.getHeading(), preintake.getHeading(), shoot.getHeading())
                .build();
        //backsite

        toBackIntake = follower.pathBuilder()
                .addPath(new BezierLine(shoot, intakePose4))
                .setLinearHeadingInterpolation(shoot.getHeading(), intakePose4.getHeading())
                .build();
        reset = follower.pathBuilder()
                .addPath(new BezierLine(intakePose4, preintakeFour))
                .setLinearHeadingInterpolation(intakePose4.getHeading(), preintakeFour.getHeading())
                .build();
        toShootFromBack = follower.pathBuilder()
                .addPath(new BezierLine(intakePose4, shoot ))
                .setLinearHeadingInterpolation(intakePose4.getHeading(), shoot.getHeading())
                .build();

        // 3rd row
        toRow3Pre = follower.pathBuilder()
                .addPath(new BezierCurve(shoot, preintakeTwo, preintakeTwo))
                .setLinearHeadingInterpolation(shoot.getHeading(), preintakeTwo.getHeading(), preintakeTwo.getHeading())
                .build();

        intakeRow3 = follower.pathBuilder()
                .addPath(new BezierLine(preintakeTwo, intakePose2))
                .setLinearHeadingInterpolation(preintakeTwo.getHeading(), intakePose2.getHeading())
                .build();

        toShootFromRow3 = follower.pathBuilder()
                .addPath(new BezierCurve(intakePose2, preintakeTwo, shoot))
                .setLinearHeadingInterpolation(intakePose2.getHeading(), preintakeTwo.getHeading(), shoot.getHeading())
                .build();

        parkPath = follower.pathBuilder()
                .addPath(new BezierLine(shoot, parkPose))
                .setLinearHeadingInterpolation(shoot.getHeading(), parkPose.getHeading())
                .build();
    }

    // -------------------------
    // Steps helpers
    // -------------------------
    private Step follow(PathChain path, boolean intakeOn) {
        return new Step() {
            @Override public void init() {
                intakeNow = intakeOn;
                reverseIntake = false;
                follower.followPath(path, true);
            }
            @Override public void update() {}
            @Override public boolean isFinished() { return !follower.isBusy(); }
        };
    }

    private Step waitSec(double sec) {
        return new Step() {
            final ElapsedTime t = new ElapsedTime();
            @Override public void init() { t.reset(); }
            @Override public void update() {}
            @Override public boolean isFinished() { return t.seconds() >= sec; }
        };
    }

    /**
     * Replacement for any "shoot" step:
     * - turns intake off
     * - waits 0.75s
     */
    private Step noShootDelayStep() {
        return new Step() {
            final ElapsedTime t = new ElapsedTime();
            @Override public void init() {
                intakeNow = false;
                reverseIntake = false;
                t.reset();
            }
            @Override public void update() {}
            @Override public boolean isFinished() { return t.seconds() >= NO_SHOOT_DELAY_SEC; }
        };
    }

    private void buildAutonSteps() {
        steps.clear();

        // Preload drive (shooting disabled -> just delay instead)
        steps.add(noShootDelayStep());

        // 2nd row: drive, intake, return to "shoot" waypoint, delay
        steps.add(follow(toRow2Pre, false));
        steps.add(follow(intakeRow2, true));

        steps.add(followWithMaxPower(intakeBackup, false,0.85));
        steps.add(waitSec(0.02));

        steps.add(follow(toShootFromRow2, false));
        steps.add(noShootDelayStep());

        //backsite
        steps.add(follow(toBackIntake, true));
        steps.add(follow(reset, true));
        steps.add(follow(toBackIntake, true));
        steps.add(follow(toShootFromBack, false));
        steps.add(noShootDelayStep());


        // 3rd row: intake then delay at "shoot" waypoint
        steps.add(follow(toRow3Pre, false));
        steps.add(follow(intakeRow3, true));
        steps.add(follow(toShootFromRow3, false));
        steps.add(noShootDelayStep());

        //backsite
        steps.add(follow(toBackIntake, true));
        steps.add(follow(reset, true));
        steps.add(follow(toBackIntake, true));
        steps.add(follow(toShootFromBack, false));
        steps.add(noShootDelayStep());
        // Park
          steps.add(follow(parkPath, false));
    }

    // -------------------------
    // FTC lifecycle
    // -------------------------
    @Override
    public void init() {
        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(startPose);

        buildPaths();

        intake  = hardwareMap.get(DcMotor.class, "intake");
        intake2 = hardwareMap.get(DcMotor.class, "intake2");

        lower = hardwareMap.get(ColorRangeSensor.class, "lower");
        upper = hardwareMap.get(ColorRangeSensor.class, "upper");

        stopper = hardwareMap.get(Servo.class, "stopper");
        stopper.setPosition(transferConstants.GATE_CLOSED); // keep closed since shooting disabled

        intake.setPower(0.0);
        intake2.setPower(0.0);
    }

    @Override
    public void start() {
        buildAutonSteps();
        runner = new StepRunner(steps);
    }

    @Override
    public void loop() {
        follower.update();

        // Apply intake logic every loop (shooting disabled, so nothing blocks intake except intakeNow flag)
        autoIntakeControl(intakeNow, reverseIntake);

        // Run the step queue
        runner.run();

        if (runner.isDone() && !follower.isBusy()) {
            requestOpModeStop();
        }

        telemetry.addData("Step", runner.getIndex());
        telemetry.addData("IntakeNow", intakeNow);
        telemetry.update();
    }

    @Override
    public void stop() {
        intake.setPower(0.0);
        intake2.setPower(0.0);
        if (stopper != null) stopper.setPosition(transferConstants.GATE_CLOSED);
    }

    // -------------------------
    // Color-sensor intake logic (copied from your TeleOp intake())
    // driven by auton intent instead of triggers
    // -------------------------
    private void autoIntakeControl(boolean wantIntake, boolean wantReverse) {
        if (!wantIntake) {
            intake.setPower(0.0);
            intake2.setPower(0.0);
            return;
        }

        String lowerColor = getColor(lower);
        String upperColor = getColor(upper);

        long now = System.currentTimeMillis();

        if (isGreenOrPurple(upperColor)) lastUpperDetectMs = now;
        if (isGreenOrPurple(lowerColor)) lastLowerDetectMs = now;

        boolean upperRecent = (now - lastUpperDetectMs) < 1000;
        boolean lowerRecent = (now - lastLowerDetectMs) < 1000;

        double p1;
        double p2;

        if (!wantReverse) {
            p1 = -transferConstants.intakePower;
            p2 = transferConstants.intakePower;
        } else {
            p1 = transferConstants.intakePower;
            p2 = -transferConstants.intakePower;
        }

        // EXACTLY like your TeleOp:
        // Stop bottom stage only if lowerRecent AND reversing
        if (lowerRecent && wantReverse) {
            p1 = 0.0;
        }

        // Stop upper stage if upper sees a ball recently
        if (upperRecent) {
            p2 = 0.0;
        }

        intake.setPower(p1);
        intake2.setPower(p2);
    }

    private String getColor(NormalizedColorSensor sensor) {
        NormalizedRGBA color = sensor.getNormalizedColors();
        double hue = JavaUtil.colorToHue(color.toColor());

        if (hue < 30) return "red";
        if (hue < 60) return "orange";
        if (hue < 90) return "yellow";
        if (hue < 150) return "green";
        if (hue < 225) return "green";
        if (hue < 350) return "purple";
        return "red";
    }

    private boolean isGreenOrPurple(String c) {
        return "green".equals(c) || "purple".equals(c);
    }
}