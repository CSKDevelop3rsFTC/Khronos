package org.firstinspires.ftc.teamcode.auto;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.pedropathing.Constants;
import org.firstinspires.ftc.teamcode.teleop.meet3.Meet3TeleOpB;
import org.firstinspires.ftc.teamcode.teleop.ShooterPID;

@Autonomous(name = "blueGoalREAL", group = "Autonomous")
public class blueGoalREAL extends OpMode {

    // Pedro follower
    private Follower follower;
    private Timer pathTimer;
    private int pathState;

    // Paths / PathChains
    private PathChain preload, preintake1, intake1, shoot1,
            preintake2, intake2, shoot2, preGate, gateOpen,
            preintake3, intake3, shoot3,
            preintake4, intake4, backup, intake5, shoot4,
            parkPath;

    // Hardware
    private DcMotor intakeMotor;
    private DcMotor turretMotor;
    private Servo kickerServo, stopperServo;
    private IMU imu; // currently not used in auto, but kept if needed later

    // Shooter (two flywheel motors: f1M, f2M)
    private ShooterPID shooter;

    // Intake logical state (when not in transfer)
    private boolean intakeNow = false;

    // Transfer state machine
    private boolean transferRunning = false;
    private int transferStep = 0;
    private final ElapsedTime transferTimer = new ElapsedTime();

    // Shooter “ready” tolerance
    private static final double SHOOTER_READY_TOLERANCE = 75.0; // ±75 RPM

    // -------- Turret control (same math as TeleOp) --------
    private static final double MOTOR_CPR        = 537.6;  // GoBILDA 312RPM encoder CPR
    private static final int    GEAR_INPUT_TEETH = 63;
    private static final int    GEAR_OUTPUT_TEETH= 190;

    private static final double GEAR_RATIO       = (double) GEAR_OUTPUT_TEETH / GEAR_INPUT_TEETH;
    private static final double TICKS_PER_TURRET_REV = MOTOR_CPR * GEAR_RATIO;
    private static final double TICKS_PER_DEG    = TICKS_PER_TURRET_REV / 360.0;
    private static final double DEG_PER_TICK     = 1.0 / TICKS_PER_DEG;

    private static final double TURRET_KP        = 0.01;
    private static final double TURRET_HOLD      = 0.05;
    // Tighter tolerance so it actually returns to the same angle
    private static final double TURRET_DEG_TOL   = 0.5;    // within 0.5° is “good”

    // Small bias so the turret truly looks centered when returning home
    // If it starts looking 2° to the RIGHT instead, flip this to -2.0.
    private static final double TURRET_RETURN_OFFSET_DEG = 2.0;

    // Turret angle management
    private double turretHomeDeg = 0.0;          // angle at start of auton (you align manually)
    private double turretTargetDeg = 0.0;        // what we’re driving to
    private boolean turretOffsetApplied = false; // whether we've done the -45° offset

    // Return-to-home logic at end of auton
    private boolean turretReturningHome = false;
    private final ElapsedTime turretReturnTimer = new ElapsedTime();

    // Poses (from your adjusted code)
    private final Pose startPose    = new Pose(123, 60, Math.toRadians(45));
    private final Pose shoot0       = new Pose(86, 24, Math.toRadians(45));
    private final Pose preintake    = new Pose(84, 27, Math.toRadians(90));
    private final Pose intakePose   = new Pose(84, 56, Math.toRadians(90));
    private final Pose shoot        = new Pose(86, 24, Math.toRadians(90));
    private final Pose preintakeOne = new Pose(60, 27, Math.toRadians(90));
    private final Pose intakePose1  = new Pose(60, 66, Math.toRadians(90));
    private final Pose preGate1  = new Pose(60, 50, Math.toRadians(90));
    // open gate
    private final Pose gate         = new Pose(72, 63, Math.toRadians(90));
    // new positions (4th cycle)
    private final Pose preintakeTwo = new Pose(36, 27, Math.toRadians(90));
    private final Pose intakePose2  = new Pose(36, 66, Math.toRadians(90));
    // intake base (last cycle)
    private final Pose preintakeThree  = new Pose(50, 73, Math.toRadians(135));
    private final Pose intakePose3  = new Pose(18, 70, Math.toRadians(135));
    private final Pose safetyIntake  = new Pose(19, 70, Math.toRadians(135));
    // parking position after everything is shot
    private final Pose parkPose     = new Pose(72, 48, Math.toRadians(0));

    // Pauses at each shooting position (1 second while intaking)
    private final ElapsedTime shoot0PauseTimer = new ElapsedTime();
    private boolean shoot0PauseActive = false;

    private final ElapsedTime shoot1PauseTimer = new ElapsedTime();
    private boolean shoot1PauseActive = false;

    private final ElapsedTime shoot2PauseTimer = new ElapsedTime();
    private boolean shoot2PauseActive = false;

    private final ElapsedTime shoot3PauseTimer = new ElapsedTime();
    private boolean shoot3PauseActive = false;

    private final ElapsedTime shoot4PauseTimer = new ElapsedTime();
    private boolean shoot4PauseActive = false;

    private void setPathState(int s) {
        pathState = s;
        pathTimer.resetTimer();
    }

    // --------------------------------------------------------------------
    // PATH BUILDING
    // --------------------------------------------------------------------
    private void buildPath() {

        preload = follower.pathBuilder()
                .addPath(new BezierLine(startPose, shoot0))
                .setLinearHeadingInterpolation(startPose.getHeading(), shoot0.getHeading())
                .build();

        preintake1 = follower.pathBuilder()
                .addPath(new BezierLine(shoot0, preintake))
                .setLinearHeadingInterpolation(shoot0.getHeading(), preintake.getHeading())
                .build();

        intake1 = follower.pathBuilder()
                .addPath(new BezierLine(preintake, intakePose))
                .setLinearHeadingInterpolation(preintake.getHeading(), intakePose.getHeading())
                .build();

        shoot1 = follower.pathBuilder()
                .addPath(new BezierLine(intakePose, shoot))
                .setLinearHeadingInterpolation(intakePose.getHeading(), shoot.getHeading())
                .build();

        preintake2 = follower.pathBuilder()
                .addPath(new BezierLine(shoot, preintakeOne))
                .setLinearHeadingInterpolation(shoot.getHeading(), preintakeOne.getHeading())
                .build();

        intake2 = follower.pathBuilder()
                .addPath(new BezierLine(preintakeOne, intakePose1))
                .setLinearHeadingInterpolation(preintakeOne.getHeading(), intakePose1.getHeading())
                .build();

        // gate sequence
        preGate  = follower.pathBuilder()
                .addPath(new BezierLine(intakePose1, preGate1))
                .setLinearHeadingInterpolation(intakePose1.getHeading(), preintakeOne.getHeading())
                .build();

        gateOpen = follower.pathBuilder()
                .addPath(new BezierLine(preGate1, gate))
                .setLinearHeadingInterpolation(preintakeOne.getHeading(), gate.getHeading())
                .build();

        shoot2 = follower.pathBuilder()
                .addPath(new BezierLine(gate, shoot))
                .setLinearHeadingInterpolation(gate.getHeading(), shoot.getHeading())
                .build();

        // 3rd cycle paths
        preintake3 = follower.pathBuilder()
                .addPath(new BezierLine(shoot, preintakeTwo))
                .setLinearHeadingInterpolation(shoot.getHeading(), preintakeTwo.getHeading())
                .build();

        intake3 = follower.pathBuilder()
                .addPath(new BezierLine(preintakeTwo, intakePose2))
                .setLinearHeadingInterpolation(preintakeTwo.getHeading(), intakePose2.getHeading())
                .build();

        shoot3 = follower.pathBuilder()
                .addPath(new BezierLine(intakePose2, shoot))
                .setLinearHeadingInterpolation(intakePose2.getHeading(), shoot.getHeading())
                .build();

        // last cycle
        preintake4 = follower.pathBuilder()
                .addPath(new BezierLine(shoot, preintakeThree))
                .setLinearHeadingInterpolation(shoot.getHeading(), preintakeThree.getHeading())
                .build();

        intake4 = follower.pathBuilder()
                .addPath(new BezierLine(preintakeThree, intakePose3))
                .setLinearHeadingInterpolation(preintakeThree.getHeading(), intakePose3.getHeading())
                .build();

        backup = follower.pathBuilder()
                .addPath(new BezierLine(intakePose3, safetyIntake))
                .setLinearHeadingInterpolation(intakePose3.getHeading(), safetyIntake.getHeading())
                .build();

        intake5 = follower.pathBuilder()
                .addPath(new BezierLine(safetyIntake, intakePose3))
                .setLinearHeadingInterpolation(safetyIntake.getHeading(), intakePose3.getHeading())
                .build();

        shoot4 = follower.pathBuilder()
                .addPath(new BezierLine(intakePose3, shoot))
                .setLinearHeadingInterpolation(intakePose3.getHeading(), shoot.getHeading())
                .build();

        // Parking path: from last shooting pose back to parkPose
        parkPath = follower.pathBuilder()
                .addPath(new BezierLine(shoot, parkPose))
                .setLinearHeadingInterpolation(shoot.getHeading(), parkPose.getHeading())
                .build();
    }

    // --------------------------------------------------------------------
    // INIT / START / LOOP
    // --------------------------------------------------------------------
    @Override
    public void init() {

        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(startPose);

        pathTimer = new Timer();

        intakeMotor  = hardwareMap.get(DcMotor.class, "intake");
        turretMotor  = hardwareMap.get(DcMotor.class, "turret");
        kickerServo  = hardwareMap.get(Servo.class, "kicker");
        stopperServo = hardwareMap.get(Servo.class, "stopper");
        imu          = hardwareMap.get(IMU.class, "imu"); // if you want later

        shooter = new ShooterPID(hardwareMap, "f1M", "f2M");

        intakeMotor.setPower(0.0);
        kickerServo.setPosition(0.5);
        stopperServo.setPosition(0.5);

        // Turret setup (encoder-based only, no IMU aiming in auto)
        turretMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        turretMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        turretMotor.setPower(0.0);

        buildPath();
    }

    @Override
    public void init_loop() {
        // keep servos safe during init
        if (kickerServo != null) kickerServo.setPosition(0.5);
        if (stopperServo != null) stopperServo.setPosition(0.5);
    }

    @Override
    public void start() {
        // Servo defaults at start
        kickerServo.setPosition(0.5);
        stopperServo.setPosition(0.5);

        // Capture turret "home" angle at start (you manually align it)
        int turretTicks = 0;
        try {
            turretTicks = turretMotor.getCurrentPosition();
        } catch (Exception ignored) {}
        turretHomeDeg = turretTicks * DEG_PER_TICK;
        turretTargetDeg = turretHomeDeg;
        turretOffsetApplied = false;
        turretReturningHome = false;

        shoot0PauseActive = false;
        shoot1PauseActive = false;
        shoot2PauseActive = false;
        shoot3PauseActive = false;
        shoot4PauseActive = false;

        // Spin up shooter based on initial position
        updateShooterTargetFromPose();
        shooter.update();

        setPathState(0);
    }

    @Override
    public void loop() {

        // Update follower (pose, path progress)
        follower.update();

        // Shooter RPM control based on distance to goal
        updateShooterTargetFromPose();
        shooter.update();

        // Turret holds / moves to target angle
        updateTurret();

        // Transfer sequence (3-ball shot) state machine
        runTransfer();

        // Normal intake control only when NOT in transfer
        if (!transferRunning) {
            intakeMotor.setPower(intakeNow ? Meet3TeleOpB.transferConstants.intakePower : 0.0);
        }

        // Path state machine
        pathUpdate();

        // Telemetry
        try {
            telemetry.addData("State", pathState);
            telemetry.addData("t", follower.getCurrentTValue());
            telemetry.addData("x", follower.getPose().getX());
            telemetry.addData("y", follower.getPose().getY());
            telemetry.addData("heading", Math.toDegrees(follower.getPose().getHeading()));
        } catch (Exception ignored) {}
        telemetry.addData("Shooter Target RPM", shooter.getTargetRPM());
        telemetry.addData("Shooter RPM", shooter.getCurrentRPM());
        telemetry.addData("Transfer Running", transferRunning);
        telemetry.addData("Turret Home Deg", turretHomeDeg);
        telemetry.addData("Turret Target Deg", turretTargetDeg);
        telemetry.addData("Shoot0Pause", shoot0PauseActive);
        telemetry.addData("Shoot1Pause", shoot1PauseActive);
        telemetry.addData("Shoot2Pause", shoot2PauseActive);
        telemetry.addData("Shoot3Pause", shoot3PauseActive);
        telemetry.addData("Shoot4Pause", shoot4PauseActive);
        telemetry.update();
    }

    @Override
    public void stop() {
        shooter.stop();
        intakeMotor.setPower(0.0);
        turretMotor.setPower(0.0);
        kickerServo.setPosition(0.5);
        stopperServo.setPosition(0.5);
    }

    // --------------------------------------------------------------------
    // PATH LOGIC (5 shooting cycles + park + turret return)
    // --------------------------------------------------------------------
    private void pathUpdate() {

        switch (pathState) {

            // ---- PRELOAD: startPose → shoot0, INTAKE ON from start ----
            case 0:
                intakeNow = true;
                follower.followPath(preload, true);
                setPathState(1);
                break;

            // At shoot0 → wait 1s WHILE INTAKING, then run first 3-ball transfer
            case 1:
                if (!follower.isBusy()) {
                    if (!shoot0PauseActive) {
                        shoot0PauseActive = true;
                        shoot0PauseTimer.reset();
                        intakeNow = true;  // keep feeding while robot settles
                    } else {
                        if (shoot0PauseTimer.seconds() >= 0.25 && !transferRunning) {
                            intakeNow = false; // transfer takes over intake
                            startTransfer();   // 🔥 first 3
                            shoot0PauseActive = false;
                            setPathState(2);
                        }
                    }
                }
                break;

            // ---- INTAKE 1 ---- (for second 3-shot)
            // Wait for transfer1 done → go to preintake1
            case 2:
                if (!transferRunning) {
                    intakeNow = true;
                    follower.followPath(preintake1, true);
                    setPathState(3);
                }
                break;

            // preintake1 → intake1 (collect 3 more)
            case 3:
                intakeNow = true;
                follower.followPath(intake1, true);
                setPathState(4);
                break;

            // After intake1 finished → drive back to shoot1, apply turret -45° while driving
            case 4:
                if (!follower.isBusy()) {
                    intakeNow = true; // intake still on while driving back

                    // First time we head back to shoot1: rotate turret 45° RIGHT (negative)
                    if (!turretOffsetApplied) {
                        turretTargetDeg = turretHomeDeg - 45.0;
                        turretOffsetApplied = true;
                    }

                    follower.followPath(shoot1, true);
                    setPathState(5);
                }
                break;

            // At shoot1 → wait 1s WHILE INTAKING, then run second 3-ball transfer
            case 5:
                if (!follower.isBusy()) {
                    if (!shoot1PauseActive) {
                        shoot1PauseActive = true;
                        shoot1PauseTimer.reset();
                        intakeNow = true;
                    } else {
                        if (shoot1PauseTimer.seconds() >= 0.25 && !transferRunning) {
                            intakeNow = false;
                            startTransfer();   // 🔥 second 3
                            shoot1PauseActive = false;
                            setPathState(6);
                        }
                    }
                }
                break;

            // ---- INTAKE 2 + GATE + SHOOT2 ----
            // Wait transfer2 done → preintake2
            case 6:
                if (!transferRunning) {
                    intakeNow = true;
                    follower.followPath(preintake2, true);
                    setPathState(7);
                }
                break;

            // preintake2 → intake2 (INTAKE ON)
            case 7:
                intakeNow = true;
                follower.followPath(intake2, true);
                setPathState(8);
                break;

            // After intake2 finished → preGate (INTAKE ON)
            case 8:
                if (!follower.isBusy()) {
                    intakeNow = true;
                    follower.followPath(preGate, true);
                    setPathState(9);
                }
                break;

            // preGate → gateOpen (INTAKE ON)
            case 9:
                if (!follower.isBusy()) {
                    intakeNow = true;
                    follower.followPath(gateOpen, true);
                    setPathState(10);
                }
                break;

            // gateOpen → shoot2 (INTAKE ON)
            case 10:
                if (!follower.isBusy()) {
                    intakeNow = true;
                    follower.followPath(shoot2, true);
                    setPathState(11);
                }
                break;

            // At shoot2 → wait 1s WHILE INTAKING, then run third 3-ball transfer
            case 11:
                if (!follower.isBusy()) {
                    if (!shoot2PauseActive) {
                        shoot2PauseActive = true;
                        shoot2PauseTimer.reset();
                        intakeNow = true; // still feeding
                    } else {
                        if (shoot2PauseTimer.seconds() >= 0.25 && !transferRunning) {
                            intakeNow = false;
                            startTransfer();   // 🔥 third 3
                            shoot2PauseActive = false;
                            setPathState(12);
                        }
                    }
                }
                break;

            // ---- INTAKE 3 ---- (for fourth 3-shot)
            // Wait transfer3 done → preintake3
            case 12:
                if (!transferRunning) {
                    intakeNow = true;
                    follower.followPath(preintake3, true);
                    setPathState(13);
                }
                break;

            // preintake3 → intake3
            case 13:
                intakeNow = true;
                follower.followPath(intake3, true);
                setPathState(14);
                break;

            // After intake3 finished → shoot3
            case 14:
                if (!follower.isBusy()) {
                    intakeNow = true;
                    follower.followPath(shoot3, true);
                    setPathState(15);
                }
                break;

            // At shoot3 → wait 1s WHILE INTAKING, then run fourth 3-ball transfer
            case 15:
                if (!follower.isBusy()) {
                    if (!shoot3PauseActive) {
                        shoot3PauseActive = true;
                        shoot3PauseTimer.reset();
                        intakeNow = true;
                    } else {
                        if (shoot3PauseTimer.seconds() >= 0.25 && !transferRunning) {
                            intakeNow = false;
                            startTransfer();   // 🔥 fourth 3
                            shoot3PauseActive = false;
                            setPathState(16);
                        }
                    }
                }
                break;

            // ---- INTAKE 4 ---- (for fifth and final 3-shot)
            // Wait transfer4 done → preintake4
            case 16:
                if (!transferRunning) {
                    intakeNow = true;
                    follower.followPath(preintake4, true);
                    setPathState(17);
                }
                break;

            // preintake4 → intake4 (INTAKE ON)
            case 17:
                intakeNow = true;
                follower.followPath(intake4, true);
                setPathState(18);
                break;

            // After intake4 finished → backup path (INTAKE OFF)
            case 18:
                if (!follower.isBusy()) {
                    intakeNow = false;
                    follower.followPath(backup, true);
                    setPathState(19);
                }
                break;

            // backup → intake5 (INTAKE ON again)
            case 19:
                if (!follower.isBusy()) {
                    intakeNow = true;
                    follower.followPath(intake5, true);
                    setPathState(20);
                }
                break;

            // After intake5 finished → go to final shoot4 position
            case 20:
                if (!follower.isBusy()) {
                    intakeNow = true; // keep feeding while going to shoot4
                    follower.followPath(shoot4, true);
                    setPathState(21);
                }
                break;

            // At shoot4 → wait 1s WHILE INTAKING, then run final 3-ball transfer
            case 21:
                if (!follower.isBusy()) {
                    if (!shoot4PauseActive) {
                        shoot4PauseActive = true;
                        shoot4PauseTimer.reset();
                        intakeNow = true;
                    } else {
                        if (shoot4PauseTimer.seconds() >= 0.25 && !transferRunning) {
                            intakeNow = false;
                            startTransfer();   // 🔥 fifth 3
                            shoot4PauseActive = false;
                            setPathState(22);
                        }
                    }
                }
                break;

            // Wait final transfer done → drive to park point
            case 22:
                if (!transferRunning) {
                    intakeNow = false;
                    follower.followPath(parkPath, true);
                    setPathState(23);
                }
                break;

            // Wait until parked at parkPose, then start turret return
            case 23:
                if (!follower.isBusy()) {
                    intakeNow = false;
                    if (!turretReturningHome) {
                        // Apply small offset so visually it returns dead center
                        turretTargetDeg = turretHomeDeg + TURRET_RETURN_OFFSET_DEG;
                        turretReturningHome = true;
                        turretReturnTimer.reset();
                    }
                    setPathState(24);
                }
                break;

            // Drive turret back to (home + offset) angle
            case 24:
                intakeNow = false;
                // give it up to ~1.5s or until it's within tighter tolerance
                if (turretReturnTimer.seconds() >= 1.5 || turretAtTarget()) {
                    setPathState(25);
                }
                break;

            // Finished – hold everything off and stop the OpMode
            case 25:
            default:
                intakeNow = false;
                requestOpModeStop();
                break;
        }
    }

    // --------------------------------------------------------------------
    // SHOOTER CONTROL (fixed RPM in auto right now)
    // --------------------------------------------------------------------
    private void updateShooterTargetFromPose() {
        // If you later want distance-based RPM, you already have dx/dy below.
        double rx = 0.0;
        double ry = 0.0;
        try {
            Pose p = follower.getPose();
            rx = p.getX();
            ry = p.getY();
        } catch (Exception ignored) {}

        double goalX = Meet3TeleOpB.Goal.X;
        double goalY = Meet3TeleOpB.Goal.Y;

        double dx = goalX - rx;
        double dy = goalY - ry;
        double dist = Math.hypot(dx, dy);

        // Currently using flat value – adjust if needed
        double rpm = 532;
        shooter.setTargetRPM(rpm);
    }

    private boolean isShooterAtSpeed() {
        double target = shooter.getTargetRPM();
        if (target < 100.0) return false;
        return Math.abs(shooter.getCurrentRPM() - target) <= SHOOTER_READY_TOLERANCE;
    }

    // --------------------------------------------------------------------
    // TURRET HELPERS (encoder-only; no IMU aiming in auto)
    // --------------------------------------------------------------------
    private static double wrapDeg(double a) {
        while (a > 180) a -= 360;
        while (a < -180) a += 360;
        return a;
    }

    private boolean driveTurretToDeg(double targetTurretDeg) {
        int ticks = turretMotor.getCurrentPosition();
        double turretDegNow = ticks * DEG_PER_TICK;
        double errDeg = wrapDeg(targetTurretDeg - turretDegNow);

        if (Math.abs(errDeg) <= TURRET_DEG_TOL) {
            turretMotor.setPower(0.0);
            return true;
        }

        double cmd = TURRET_KP * (errDeg * TICKS_PER_DEG);

        double maxPower = Meet3TeleOpB.Goal.TURRET_MAX_POWER; // same cap as TeleOp
        if (cmd > maxPower) cmd = maxPower;
        if (cmd < -maxPower) cmd = -maxPower;

        // gentle minimum hold so it doesn't coast
        if (Math.abs(cmd) < TURRET_HOLD) {
            cmd = Math.signum(cmd) * TURRET_HOLD;
        }

        turretMotor.setPower(cmd);
        return Math.abs(errDeg) <= TURRET_DEG_TOL;
    }

    private void updateTurret() {
        driveTurretToDeg(turretTargetDeg);
    }

    private boolean turretAtTarget() {
        int ticks = turretMotor.getCurrentPosition();
        double turretDegNow = ticks * DEG_PER_TICK;
        double errDeg = wrapDeg(turretTargetDeg - turretDegNow);
        return Math.abs(errDeg) <= TURRET_DEG_TOL;
    }

    // --------------------------------------------------------------------
    // TRANSFER (3 balls, TeleOp timings)
    // --------------------------------------------------------------------
    private void startTransfer() {
        transferRunning = true;
        transferStep = 0;
        transferTimer.reset();

        // Start transfer with stopper open like TeleOp
        kickerServo.setPosition(0.5);  // neutral
        stopperServo.setPosition(0.0); // open gate at transfer start
    }

    private void runTransfer() {
        if (!transferRunning) return;

        switch (transferStep) {
            // Step 0: Wait for shooter at speed
            case 0:
                if (isShooterAtSpeed()) {
                    // first intake burst
                    intakeMotor.setPower(Meet3TeleOpB.transferConstants.intakePower);
                    transferTimer.reset();
                    transferStep = 1;
                }
                break;

            // Step 1: keep intake on for firstWait, then stop
            case 1:
                if (transferTimer.milliseconds() >= 100) {
                    intakeMotor.setPower(0.0);
                    transferTimer.reset();
                    transferStep = 2;
                }
                break;

            // Step 2: pauseWait with intake off
            case 2:
                if (transferTimer.milliseconds() >= 150) {
                    // second intake burst
                    intakeMotor.setPower(Meet3TeleOpB.transferConstants.intakePower);
                    transferTimer.reset();
                    transferStep = 3;
                }
                break;

            // Step 3: after pauseWait2, stop intake and fire kicker
            case 3:
                if (transferTimer.milliseconds() >= 150) {
                    intakeMotor.setPower(0.0);
                    kickerServo.setPosition(0.0); // fire
                    transferTimer.reset();
                    transferStep = 4;
                }
                break;

            // Step 4: waitAfterKick, then reset servos and finish
            case 4:
                if (transferTimer.milliseconds() >= 100) {
                    kickerServo.setPosition(0.5);
                    stopperServo.setPosition(0.5);
                    intakeMotor.setPower(0.0);

                    transferRunning = false;
                    transferStep = 0;
                }
                break;
        }
    }
}