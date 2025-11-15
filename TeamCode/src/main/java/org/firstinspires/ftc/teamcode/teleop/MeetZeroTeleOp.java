package org.firstinspires.ftc.teamcode.teleop;

import com.bylazar.configurables.PanelsConfigurables;
import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.configurables.annotations.IgnoreConfigurable;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.VoltageSensor;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;

@TeleOp(name="MeetZeroTeleOp", group="Robot")
public class MeetZeroTeleOp extends LinearOpMode {

 // -------- DRIVE / MECH ------------
 private DcMotor frontLeftDrive, frontRightDrive, backLeftDrive, backRightDrive;
 private CRServo leftFeeder, rightFeeder;
 private DcMotorEx launcher;

 // -------- PINPOINT / AUTOPARK -----
 private GoBildaPinpointDriver pinpoint;

 // Field coordinates in inches
 private static final double START_X_IN = 96;
 private static final double START_Y_IN = 48;
 private static final double START_HEADING_DEG = 45.0;

 // Manual reseed positions
 private static final double POS1_X_IN = 7.25;
 private static final double POS1_Y_IN = -(72-7.5);
 private static final double POS1_HEADING_DEG = 0.0;

 private static final double POS2_X_IN = 7.25;
 private static final double POS2_Y_IN = (72-7.5);
 private static final double POS2_HEADING_DEG = 0.0;

 // Park target (default)
 private static final double PARK_X_IN = 31.0;
 private static final double PARK_Y_IN = 32.0;
 private static final double HOLD_HEADING_DEG = 0.0;

 // Added: preset park used after START reseed
 private static final double START_PARK_X_IN = 31;
 private static final double START_PARK_Y_IN = -32.0;
 private static final double START_PARK_HEADING_DEG = 0.0;

 // Track which reseed we last did
 private enum ReseedSource { NONE, START_BUTTON, BACK_BUTTON }
 private ReseedSource lastReseed = ReseedSource.NONE;

 // Auto control
 private static final double KP_XY = 0.035;
 private static final double KP_HDG = 1.2;
 private static final double MAX_TRANS_POWER = 0.7;
 private static final double MAX_TURN_POWER  = 0.6;
 private static final double MIN_MOVE_POWER  = 0.08;
 private static final double MIN_MOVE_POWER_Y = 0.12;
 private static final double POS_TOL_X_IN    = 1.0;
 private static final double POS_TOL_Y_IN    = 2.0;
 private static final double HDG_TOL_DEG     = 5.0;
 private static final long   SETTLE_MS       = 250;
 private static final long   AUTOPARK_TIMEOUT_MS = 9000;

 // Coordinate frame correction
 private static final int SIGN_X = 1;
 private static final int SIGN_Y = 1;
 private static final int SIGN_HDG =  1;

 private enum AutoParkState { OFF, RUNNING, SETTLING, DONE, CANCELLED, TIMED_OUT }
 private AutoParkState autoState = AutoParkState.OFF;

 private final com.qualcomm.robotcore.util.ElapsedTime autoTimer = new com.qualcomm.robotcore.util.ElapsedTime();
 private final com.qualcomm.robotcore.util.ElapsedTime settleTimer = new com.qualcomm.robotcore.util.ElapsedTime();

 private boolean prevStartPark  = false;
 private boolean prevStartAlign = false;
 private boolean prevCancel     = false;

 // Goals
 private double goalX = PARK_X_IN, goalY = PARK_Y_IN, goalHeadingDeg = HOLD_HEADING_DEG;

 // Simple velocity estimate
 private double lastX = Double.NaN, lastY = Double.NaN;
 private long lastTimeNs = 0;

 // -------- SHOOTER ------------
 private static final double SERVO_FEED_POWER = -1.0;

 private enum ShooterState { IDLE, SPIN_UP, FEED, INTERSHOT, SPIN_DOWN, COOLDOWN, REVERSE_BRAKE }
 private ShooterState shooterState = ShooterState.IDLE;
 private final com.qualcomm.robotcore.util.ElapsedTime shooterTimer =
         new com.qualcomm.robotcore.util.ElapsedTime();
 private final com.qualcomm.robotcore.util.ElapsedTime cooldownTimer =
         new com.qualcomm.robotcore.util.ElapsedTime();

 // edges
 private boolean prevTrigger = false;     // LT (CLOSE)
 private boolean prevRTrigger = false;    // Y button edge tracker (now FAR)
 private boolean prevBumper = false;      // LB (BASE)
 private boolean prevRBumper = false;     // RB edge tracker (now MED)

 private int burstShots = 0;
 private boolean firstShotDone = false;
 private int shotsInCycle = 0;

 // which power set to use for the current burst
 private enum PowerSet { BASE, MED, CLOSE, FAR }
 private PowerSet powerSet = PowerSet.BASE;

 private double getVoltage() {
  double min = Double.POSITIVE_INFINITY;
  for (VoltageSensor s : hardwareMap.getAll(VoltageSensor.class)) {
   double v = s.getVoltage();
   if (v > 0) min = Math.min(min, v);
  }
  return (min == Double.POSITIVE_INFINITY) ? launcherConstants.NOMINAL_VOLTAGE : min;
 }

 private double scalePower(double base, double voltage, double scale) {
  double diff   = voltage - launcherConstants.NOMINAL_VOLTAGE;
  double factor = 1.0 - (diff * (scale / 0.5));
  return Math.max(0, Math.min(1, base * factor));
 }

 private double scaleForSet(double base, double voltage, PowerSet set) {
  double s = (set == PowerSet.BASE)
          ? launcherConstants.VOLTAGE_SCALE_BASE
          : launcherConstants.VOLTAGE_SCALE_OTHER;
  return scalePower(base, voltage, s);
 }

 // --- per-mode timing knobs (configurable) ---
 @Configurable
 public static class launcherConstants {
  // Legacy globals
  public static long SHOT_SPACE_MS = 300;
  public static long INTERSHOT_MS  = 700;
  public static long CHARGE_UP_MS  = 2000;

  // Per-mode timings
  public static long BASE_SHOT_SPACE_MS   = 300;
  public static long BASE_INTERSHOT_MS    = 400;
  public static long BASE_CHARGE_UP_MS    = 1250;

  public static long CLOSE_SHOT_SPACE_MS  = 300;
  public static long CLOSE_INTERSHOT_MS   = 350;
  public static long CLOSE_CHARGE_UP_MS   = 600;

  public static long MED_SHOT_SPACE_MS    = 300;
  public static long MED_INTERSHOT_MS     = 300;
  public static long MED_CHARGE_UP_MS     = 700;

  public static long FAR_SHOT_SPACE_MS    = 300;
  public static long FAR_INTERSHOT_MS     = 250;
  public static long FAR_CHARGE_UP_MS     = 800;

  public static long POST_BURST_BRAKE_MS  = 300;

  // Power presets
  public static double BASE_POWER_1  = 1;
  public static double BASE_POWER_2  = 0.99;
  public static double BASE_POWER_34 = 0.96;

  public static double CLOSE_POWER_1  = 0.9;
  public static double CLOSE_POWER_2  = 0.89;
  public static double CLOSE_POWER_34 = 0.86;

  public static double MED_POWER_1  = 0.9;
  public static double MED_POWER_2  = 0.89;
  public static double MED_POWER_34 = 0.86;

  public static double FAR_POWER_1  = 0.97;
  public static double FAR_POWER_2  = 0.94;
  public static double FAR_POWER_34 = 0.8;

  // Align point
  public static double ALIGN_X_IN = 27.5;
  public static double ALIGN_Y_IN = 2.0;
  public static double ALIGN_HEADING_DEG = 26;

  // Align point used after BACK reseed
  public static double ALIGN_BACK_X_IN = 21.5;
  public static double ALIGN_BACK_Y_IN = 0.0;
  public static double ALIGN_BACK_HEADING_DEG = -30;

  // Battery model
  public static double NOMINAL_VOLTAGE = 12.6;
  public static double VOLTAGE_SCALE_BASE  = 0.06;
  public static double VOLTAGE_SCALE_OTHER = 0.06;
 }

 // Helpers to read timing for the active mode
 private long chargeUpMsFor(PowerSet set) {
  switch (set) {
   case CLOSE: return launcherConstants.CLOSE_CHARGE_UP_MS;
   case MED:   return launcherConstants.MED_CHARGE_UP_MS;
   case FAR:   return launcherConstants.FAR_CHARGE_UP_MS;
   default:    return launcherConstants.BASE_CHARGE_UP_MS;
  }
 }
 private long shotSpaceMsFor(PowerSet set) {
  switch (set) {
   case CLOSE: return launcherConstants.CLOSE_SHOT_SPACE_MS;
   case MED:   return launcherConstants.MED_SHOT_SPACE_MS;
   case FAR:   return launcherConstants.FAR_SHOT_SPACE_MS;
   default:    return launcherConstants.BASE_SHOT_SPACE_MS;
  }
 }
 private long intershotMsFor(PowerSet set) {
  switch (set) {
   case CLOSE: return launcherConstants.CLOSE_INTERSHOT_MS;
   case MED:   return launcherConstants.MED_INTERSHOT_MS;
   case FAR:   return launcherConstants.FAR_INTERSHOT_MS;
   default:    return launcherConstants.BASE_INTERSHOT_MS;
  }
 }

 @Override
 public void runOpMode() {
  initialize();
  initPinpointAndSeedStart();

  telemetry.addLine("Ready. X=Park, A=Align, Start=Pos1, Back=Pos2");
  telemetry.update();
  waitForStart();

  autoTimer.reset();

  while (opModeIsActive()) {
   if (pinpoint != null) {
    try { pinpoint.update(); } catch (Exception ignored) {}
   }

   // Manual reseed shortcuts
   if (gamepad1.start) {
    reseedPinpoint(POS1_X_IN, POS1_Y_IN, POS1_HEADING_DEG);
    lastReseed = ReseedSource.START_BUTTON;
    telemetry.addLine("Reseeded to Position 1");
   }
   if (gamepad1.back) {
    reseedPinpoint(POS2_X_IN, POS2_Y_IN, POS2_HEADING_DEG);
    lastReseed = ReseedSource.BACK_BUTTON;
    telemetry.addLine("Reseeded to Position 2");
   }

   handleAutoToggles();
   if (autoState == AutoParkState.RUNNING || autoState == AutoParkState.SETTLING) {
    autoStep();
   } else {
    driveController();   // slow mode on RT
   }

   shooter();
   telemetryPush();
   sleep(20);
  }
 }

 // ----------------- INIT -----------------
 private void initialize() {
  frontLeftDrive  = hardwareMap.get(DcMotor.class, "frontLeftDrive");
  frontRightDrive = hardwareMap.get(DcMotor.class, "frontRightDrive");
  backLeftDrive   = hardwareMap.get(DcMotor.class, "backLeftDrive");
  backRightDrive  = hardwareMap.get(DcMotor.class, "backRightDrive");

  launcher    = hardwareMap.get(DcMotorEx.class, "launcher");
  leftFeeder  = hardwareMap.get(CRServo.class, "left_feeder");
  rightFeeder = hardwareMap.get(CRServo.class, "right_feeder");

  rightFeeder.setDirection(DcMotorSimple.Direction.REVERSE);

  launcher.setDirection(DcMotorSimple.Direction.FORWARD);
  launcher.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

  frontLeftDrive.setDirection(DcMotorSimple.Direction.FORWARD);
  backLeftDrive.setDirection(DcMotorSimple.Direction.FORWARD);
  frontRightDrive.setDirection(DcMotorSimple.Direction.REVERSE);
  backRightDrive.setDirection(DcMotorSimple.Direction.REVERSE);

  leftFeeder.setPower(0);
  rightFeeder.setPower(0);
  launcher.setPower(0);

  try {
   pinpoint = hardwareMap.get(GoBildaPinpointDriver.class, "pinpoint");
  } catch (Exception e) {
   pinpoint = null;
  }
 }

 private void initPinpointAndSeedStart() {
  if (pinpoint == null) return;
  pinpoint.resetPosAndIMU();
  sleep(500);
  Pose2D start = new Pose2D(
          DistanceUnit.INCH, SIGN_X * START_X_IN, SIGN_Y * START_Y_IN,
          AngleUnit.RADIANS, Math.toRadians(START_HEADING_DEG) * SIGN_HDG
  );
  pinpoint.setPosition(start);
 }

 private void reseedPinpoint(double xIn, double yIn, double headingDeg) {
  if (pinpoint == null) return;
  try {
   pinpoint.resetPosAndIMU();
   sleep(300);
  } catch (Exception ignored) {}

  Pose2D start = new Pose2D(
          DistanceUnit.INCH, SIGN_X * xIn, SIGN_Y * yIn,
          AngleUnit.RADIANS, Math.toRadians(headingDeg) * SIGN_HDG
  );
  pinpoint.setPosition(start);

  lastX = Double.NaN;
  lastY = Double.NaN;
  lastTimeNs = 0;
 }

 // ----------------- DRIVER CONTROL -------------
 private void driveController() {
  boolean slow = gamepad1.right_trigger > 0.15;
  double speed = slow ? 0.4 : 1.0;
  double pivotScale = slow ? 0.25 : 0.5;

  double x = gamepad1.left_stick_x;
  double y = gamepad1.left_stick_y;
  double pivot = -gamepad1.right_stick_x * pivotScale;

  applyMecanum(y, x, pivot, speed);
 }

 private void applyMecanum(double y, double x, double pivot, double speed) {
  frontLeftDrive.setPower(((y - x) + pivot) * speed);
  backLeftDrive.setPower(((y + x) + pivot) * speed);
  frontRightDrive.setPower(((y + x) - pivot) * speed);
  backRightDrive.setPower(((y - x) - pivot) * speed);
 }

 // ----------------- AUTO MODES -----------------
 private void handleAutoToggles() {
  boolean startPark  = gamepad1.x;
  boolean startAlign = gamepad1.a;
  boolean cancel     = gamepad1.b;

  if (startPark && !prevStartPark) {
   if (pinpoint != null) {
    switch (lastReseed) {
     case START_BUTTON:
      goalX = START_PARK_X_IN;
      goalY = START_PARK_Y_IN;
      goalHeadingDeg = START_PARK_HEADING_DEG;
      break;
     case BACK_BUTTON:
      // Use the positive-side park target after BACK reseed
      goalX = PARK_X_IN;
      goalY = PARK_Y_IN;
      goalHeadingDeg = HOLD_HEADING_DEG;
      break;
     default:
      goalX = PARK_X_IN;
      goalY = PARK_Y_IN;
      goalHeadingDeg = HOLD_HEADING_DEG;
    }
    beginAuto();
   }
  }
  if (startAlign && !prevStartAlign) {
   if (pinpoint != null) {
    if (lastReseed == ReseedSource.BACK_BUTTON) {
     // special shooting position when reseeded with BACK
     goalX = launcherConstants.ALIGN_BACK_X_IN;
     goalY = launcherConstants.ALIGN_BACK_Y_IN;
     goalHeadingDeg = launcherConstants.ALIGN_BACK_HEADING_DEG;
    } else {
     // default shooting position
     goalX = launcherConstants.ALIGN_X_IN;
     goalY = launcherConstants.ALIGN_Y_IN;
     goalHeadingDeg = launcherConstants.ALIGN_HEADING_DEG;
    }
    beginAuto();
   }
  }

  if (cancel && !prevCancel) {
   autoState = AutoParkState.CANCELLED;
   stopDrive();
  }
  prevStartPark  = startPark;
  prevStartAlign = startAlign;
  prevCancel     = cancel;
 }

 private void beginAuto() {
  autoState = AutoParkState.RUNNING;
  autoTimer.reset();
  settleTimer.reset();
  lastX = Double.NaN; lastY = Double.NaN; lastTimeNs = 0;
 }

 private void autoStep() {
  Pose2D pose = (pinpoint != null) ? pinpoint.getPosition() : null;
  if (pose == null) { autoState = AutoParkState.CANCELLED; stopDrive(); return; }

  double curX = SIGN_X * pose.getX(DistanceUnit.INCH);
  double curY = SIGN_Y * pose.getY(DistanceUnit.INCH);
  double headingRad = pose.getHeading(AngleUnit.RADIANS) * SIGN_HDG;

  double dx = goalX - curX;
  double dy = goalY - curY;
  double dist = Math.hypot(dx, dy);

  double speedInPerSec = 999.0;
  long now = System.nanoTime();
  if (!Double.isNaN(lastX) && lastTimeNs != 0) {
   double dt = Math.max(1e-6, (now - lastTimeNs) / 1e9);
   speedInPerSec = Math.hypot(curX - lastX, curY - lastY) / dt;
  }
  lastX = curX; lastY = curY; lastTimeNs = now;

  double vx_field = clamp(dx * KP_XY, -MAX_TRANS_POWER, MAX_TRANS_POWER);
  double vy_field = clamp(dy * KP_XY, -MAX_TRANS_POWER, MAX_TRANS_POWER);

  if (Math.abs(dy) <= POS_TOL_Y_IN) vy_field = 0.0;

  double cos = Math.cos(headingRad), sin = Math.sin(headingRad);
  double y_robot =  (vx_field * cos + vy_field * sin);
  double x_robot = (-vx_field * sin + vy_field * cos);

  if (dist < 6.0) {
   y_robot = addDeadband(y_robot, MIN_MOVE_POWER_Y);
   x_robot = addDeadband(x_robot, MIN_MOVE_POWER);
  }

  double targetHdg = Math.toRadians(goalHeadingDeg) * SIGN_HDG;
  double hErr = normalizeRad(targetHdg - headingRad);
  double pivot = clamp(hErr * KP_HDG, -MAX_TURN_POWER, MAX_TURN_POWER);

  boolean posOK = Math.abs(dx) <= POS_TOL_X_IN && Math.abs(dy) <= POS_TOL_Y_IN;
  boolean hdgOK = Math.abs(Math.toDegrees(hErr)) <= HDG_TOL_DEG;
  boolean slowEnough = speedInPerSec < 0.5;

  if (posOK && hdgOK && slowEnough) {
   if (autoState != AutoParkState.SETTLING) {
    autoState = AutoParkState.SETTLING;
    settleTimer.reset();
   }
   if (settleTimer.milliseconds() >= SETTLE_MS) {
    autoState = AutoParkState.DONE;
    stopDrive();
    return;
   }
  } else {
   autoState = AutoParkState.RUNNING;
   settleTimer.reset();
  }

  if (autoTimer.milliseconds() >= AUTOPARK_TIMEOUT_MS) {
   autoState = AutoParkState.TIMED_OUT;
   stopDrive();
   return;
  }

  applyMecanum(-y_robot, -x_robot, pivot, 1.0);
 }

 private void stopDrive() { applyMecanum(0, 0, 0, 0); }

 private static double clamp(double v, double lo, double hi) {
  return Math.max(lo, Math.min(hi, v));
 }

 private static double addDeadband(double v, double min) {
  if (Math.abs(v) < 1e-6) return 0.0;
  double s = Math.signum(v);
  return s * Math.max(Math.abs(v), min);
 }

 private static double normalizeRad(double a) {
  while (a > Math.PI)  a -= 2*Math.PI;
  while (a < -Math.PI) a += 2*Math.PI;
  return a;
 }

 // ----------------- SHOOTER -----------------
 private void shooter() {
  boolean closeBtn = gamepad1.y;                 // CLOSE
  boolean farBtn   = readFarLT();                // FAR with hysteresis
  boolean baseBtn  = gamepad1.left_bumper;       // BASE
  boolean medBtn   = gamepad1.right_bumper;      // MED
  boolean kill     = gamepad1.right_stick_button;


  double voltage = getVoltage();

  boolean canStart = (shooterState == ShooterState.IDLE);

  if (canStart && baseBtn && !prevBumper) {
   powerSet = PowerSet.BASE;
   firstShotDone = false; shotsInCycle = 0; burstShots = 4;
   launcher.setPower(scaleForSet(launcherConstants.BASE_POWER_1, voltage, powerSet));
   shooterState = ShooterState.SPIN_UP; shooterTimer.reset();
  }
  if (canStart && farBtn && !prevRTrigger) { // Y now FAR
   powerSet = PowerSet.FAR;
   firstShotDone = false; shotsInCycle = 0; burstShots = 4;
   launcher.setPower(scaleForSet(launcherConstants.FAR_POWER_1, voltage, powerSet));
   shooterState = ShooterState.SPIN_UP; shooterTimer.reset();
  }
  if (canStart && medBtn && !prevRBumper) { // RB now MED
   powerSet = PowerSet.MED;
   firstShotDone = false; shotsInCycle = 0; burstShots = 4;
   launcher.setPower(scaleForSet(launcherConstants.MED_POWER_1, voltage, powerSet));
   shooterState = ShooterState.SPIN_UP; shooterTimer.reset();
  }
  if (canStart && closeBtn && !prevTrigger) {
   powerSet = PowerSet.CLOSE;
   firstShotDone = false; shotsInCycle = 0; burstShots = 4;
   launcher.setPower(scaleForSet(launcherConstants.CLOSE_POWER_1, voltage, powerSet));
   shooterState = ShooterState.SPIN_UP; shooterTimer.reset();
  }

  // update edges to match new mapping
  prevTrigger  = closeBtn; // LT
  prevRTrigger = farBtn;   // Y/FAR
  prevBumper   = baseBtn;  // LB/BASE
  prevRBumper  = medBtn;   // RB/MED

  double p2, p34;
  switch (powerSet) {
   case MED:
    p2  = scaleForSet(launcherConstants.MED_POWER_2,  voltage, powerSet);
    p34 = scaleForSet(launcherConstants.MED_POWER_34, voltage, powerSet);
    break;
   case CLOSE:
    p2  = scaleForSet(launcherConstants.CLOSE_POWER_2,  voltage, powerSet);
    p34 = scaleForSet(launcherConstants.CLOSE_POWER_34, voltage, powerSet);
    break;
   case FAR:
    p2  = scaleForSet(launcherConstants.FAR_POWER_2,  voltage, powerSet);
    p34 = scaleForSet(launcherConstants.FAR_POWER_34, voltage, powerSet);
    break;
   default: // BASE
    p2  = scaleForSet(launcherConstants.BASE_POWER_2,  voltage, powerSet);
    p34 = scaleForSet(launcherConstants.BASE_POWER_34, voltage, powerSet);
    break;
  }

  switch (shooterState) {
   case IDLE:
    leftFeeder.setPower(0);
    rightFeeder.setPower(0);
    launcher.setPower(0);
    break;

   case SPIN_UP:
    if (shooterTimer.milliseconds() >= chargeUpMsFor(powerSet)) {
     leftFeeder.setPower(SERVO_FEED_POWER);
     rightFeeder.setPower(SERVO_FEED_POWER);
     shooterTimer.reset();
     shooterState = ShooterState.FEED;
    }
    break;

   case FEED:
    if (shooterTimer.milliseconds() >= shotSpaceMsFor(powerSet)) {
     leftFeeder.setPower(0);
     rightFeeder.setPower(0);

     shotsInCycle++;

     if (shotsInCycle == 1) {
      firstShotDone = true;
      launcher.setPower(p2);
     } else if (shotsInCycle >= 2) {
      launcher.setPower(p34);
     }

     if (--burstShots > 0) {
      shooterState = ShooterState.INTERSHOT;
      shooterTimer.reset();
     } else {
      launcher.setPower(-1.0);
      shooterTimer.reset();
      shooterState = ShooterState.REVERSE_BRAKE;
     }
    }
    break;

   case INTERSHOT:
    if (shooterTimer.milliseconds() >= intershotMsFor(powerSet)) {
     leftFeeder.setPower(SERVO_FEED_POWER);
     rightFeeder.setPower(SERVO_FEED_POWER);
     shooterState = ShooterState.FEED;
     shooterTimer.reset();
    }
    break;

   case REVERSE_BRAKE:
    if (shooterTimer.milliseconds() >= launcherConstants.POST_BURST_BRAKE_MS) {
     launcher.setPower(0);
     shooterState = ShooterState.SPIN_DOWN;
    }
    break;

   case SPIN_DOWN:
    launcher.setPower(0);
    shooterState = ShooterState.COOLDOWN;
    cooldownTimer.reset();
    break;

   case COOLDOWN:
    if (cooldownTimer.milliseconds() >= 3000) {
     shooterState = ShooterState.IDLE;
    }
    break;
  }

  if (kill) {
   launcher.setPower(0);
   leftFeeder.setPower(0);
   rightFeeder.setPower(0);
   shooterState = ShooterState.IDLE;
   shotsInCycle = 0;
  }

  telemetry.addData("Voltage", "%.2f V", voltage);
  telemetry.addData("Mode", powerSet);
  telemetry.addData("Shot#", shotsInCycle);
  telemetry.addData("State", shooterState);
  telemetry.addData("Timers(ms)", "cUp=%d space=%d inter=%d",
          chargeUpMsFor(powerSet), shotSpaceMsFor(powerSet), intershotMsFor(powerSet));
 }

 // Hysteresis latch for LT (FAR)
 private boolean ltHigh = false;
 private boolean readFarLT() {
  double v = gamepad1.left_trigger;
  if (!ltHigh && v > 0.20) ltHigh = true;     // rise
  else if (ltHigh && v < 0.08) ltHigh = false; // fall
  return ltHigh;
 }

 // ----------------- TELEMETRY -----------------
 private void telemetryPush() {
  if (pinpoint != null) {
   Pose2D p = pinpoint.getPosition();
   double fx = SIGN_X * p.getX(DistanceUnit.INCH);
   double fy = SIGN_Y * p.getY(DistanceUnit.INCH);
   double fh = Math.toDegrees(p.getHeading(AngleUnit.RADIANS) * SIGN_HDG);
   telemetry.addData("Pose X,Y (in)", "%.1f, %.1f", fx, fy);
   telemetry.addData("Heading (deg)", "%.1f", fh);
  } else {
   telemetry.addLine("Pinpoint not found");
  }
  telemetry.addData("Auto", autoState);
  telemetry.addData("Goal X,Y,H", "%.1f, %.1f, %.1f", goalX, goalY, goalHeadingDeg);
  telemetry.addData("Last Reseed", lastReseed);
  telemetry.addData("Timers(ms)", "cUp=%d space=%d inter=%d",
          chargeUpMsFor(powerSet), shotSpaceMsFor(powerSet), intershotMsFor(powerSet));
  telemetry.update();
 }
}
