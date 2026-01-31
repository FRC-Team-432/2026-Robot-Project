// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveModule.SteerRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.ctre.phoenix6.swerve.utility.WheelForceCalculator;
import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.numbers.N2;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.autonomous.AutoCommands;
import frc.robot.autonomous.AutoRoutines;
import frc.robot.autonomous.LinearPathRequest;
import frc.robot.commands.drive.VisionLockDriveCommand;
import frc.robot.constants.AllianceConstants;
import frc.robot.constants.AllianceConstants.Alliance;
import frc.robot.constants.AutoConstants;
import frc.robot.constants.DriveConstants;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.Superstructure;
import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.arm.ArmSIM;
import frc.robot.subsystems.climb.Climb;
import frc.robot.subsystems.feeder.Feeder;
import frc.robot.subsystems.feeder.FeederSIM;
import frc.robot.subsystems.flywheel.Flywheel;
import frc.robot.subsystems.flywheel.FlywheelSIM;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.intake.IntakeSIM;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.shooter.ShooterSIM;
import frc.robot.subsystems.vision.AprilTagTracker;
import frc.robot.subsystems.vision.LimelightSubsystem;

/**
 * RobotContainer - The central hub that connects all robot systems and controls.
 *
 * <h2>What is RobotContainer?</h2>
 * <p>Think of this class as the "brain" that wires everything together:
 * <ul>
 *   <li>Creates all the robot's subsystems (drivetrain, shooter, intake, etc.)</li>
 *   <li>Sets up controller buttons and what they do</li>
 *   <li>Configures autonomous routines</li>
 *   <li>Handles the hardware vs. simulation switch</li>
 * </ul>
 *
 * <h2>Two Controller Setup</h2>
 * <p>We use TWO Xbox controllers for better control division:
 *
 * <pre>
 *   ┌─────────────────────────────────────────────────────────────────────┐
 *   │                         CONTROLLER LAYOUT                          │
 *   ├─────────────────────────────────────────────────────────────────────┤
 *   │                                                                     │
 *   │   DRIVER (Port 0)                  OPERATOR (Port 1)               │
 *   │   ───────────────                  ─────────────────               │
 *   │   Controls:                        Controls:                       │
 *   │   • Robot movement                 • Intake                        │
 *   │   • Vision targeting               • Shooter                       │
 *   │   • Alliance selection             • Feeder                        │
 *   │                                    • Climb                         │
 *   │                                                                     │
 *   │   The DRIVER focuses on            The OPERATOR focuses on         │
 *   │   WHERE the robot goes             WHAT the robot does             │
 *   │                                                                     │
 *   └─────────────────────────────────────────────────────────────────────┘
 * </pre>
 *
 * <h2>Hardware vs Simulation</h2>
 * <p>The code automatically uses the right version of each subsystem:
 * <ul>
 *   <li><b>Real robot:</b> Uses actual motor controllers and sensors</li>
 *   <li><b>Simulation:</b> Uses physics simulation for desktop testing</li>
 * </ul>
 *
 * <p>Run {@code ./gradlew simulateJava} to test in simulation!
 *
 * @see DriveConstants for driver control settings
 * @see AllianceConstants for vision targeting configuration
 */
@Logged
public class RobotContainer {

  // ==================== Speed Configuration ====================

  /** Maximum driving speed in meters per second. */
  private final double maxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond);

  /** Maximum rotation speed in radians per second. */
  private final double maxAngularRate = RotationsPerSecond.of(1).in(RadiansPerSecond);

  // ==================== Drive Request ====================

  /**
   * Field-centric drive request.
   *
   * <p>"Field-centric" means pushing the stick forward ALWAYS drives toward
   * the far end of the field, regardless of which way the robot is facing.
   * This is MUCH easier to drive than robot-centric!
   */
  private final SwerveRequest.FieldCentric fieldCentricDrive =
      new SwerveRequest.FieldCentric()
          .withDriveRequestType(DriveRequestType.Velocity)
          .withSteerRequestType(SteerRequestType.MotionMagicExpo);

  /** Whether we're currently in field-centric mode (vs robot-centric). */
  private boolean isFieldCentric = true;

  // ==================== Controllers ====================

  /**
   * DRIVER controller - Controls robot movement and vision targeting.
   *
   * <p>Port 0 = first controller plugged in.
   */
  private final CommandXboxController driverController =
      new CommandXboxController(DriveConstants.DRIVER_CONTROLLER_PORT);

  /**
   * OPERATOR controller - Controls intake, shooter, feeder, and climb.
   *
   * <p>Port 1 = second controller plugged in.
   */
  private final CommandXboxController operatorController =
      new CommandXboxController(DriveConstants.OPERATOR_CONTROLLER_PORT);

  // ==================== Drivetrain ====================

  /**
   * The swerve drivetrain subsystem.
   *
   * <p>This is generated by CTRE Tuner X and handles all the complex
   * swerve drive math. We just tell it "go this direction at this speed"
   * and it figures out how to move each of the 4 modules.
   */
  public final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();

  // ==================== Autonomous ====================

  public final AutoCommands autoCommands =
      new AutoCommands(
          drivetrain,
          new LinearPathRequest(
              new Constraints(
                  AutoConstants.MAX_LINEAR_VELOCITY_MPS, AutoConstants.MAX_LINEAR_ACCELERATION_MPS2),
              new Constraints(
                  AutoConstants.MAX_ANGULAR_VELOCITY_RAD_S,
                  AutoConstants.MAX_ANGULAR_ACCELERATION_RAD_S2),
              new WheelForceCalculator(
                  drivetrain.getModuleLocations(),
                  Pounds.of(AutoConstants.ROBOT_MASS_LBS),
                  KilogramSquareMeters.of(AutoConstants.MOMENT_OF_INERTIA_KG_M2))));

  // ==================== Mechanism Subsystems ====================
  // These use the Hardware/SIM pattern - automatically uses simulation
  // when running on desktop, real hardware when running on robot.

  /** Arm subsystem (for game piece manipulation). */
  public final Arm arm = RobotBase.isSimulation() ? new ArmSIM() : new Arm();

  /** Old flywheel (keeping for Superstructure compatibility). */
  public final Flywheel flywheel = RobotBase.isSimulation() ? new FlywheelSIM() : new Flywheel();

  /** Intake subsystem - picks up balls from the floor. */
  public final Intake intake = RobotBase.isSimulation() ? new IntakeSIM() : new Intake();

  /** Shooter subsystem - dual flywheel for launching balls. */
  public final Shooter shooter = RobotBase.isSimulation() ? new ShooterSIM() : new Shooter();

  /** Feeder subsystem - moves balls from hopper to shooter. */
  public final Feeder feeder = RobotBase.isSimulation() ? new FeederSIM() : new Feeder();

  /** Climb subsystem - for end-game climbing (STUB - not yet implemented). */
  public final Climb climb = new Climb();

  /** Superstructure coordinator (for arm + flywheel + intake coordination). */
  private final Superstructure superstructure = new Superstructure(arm, flywheel, intake);

  // ==================== Vision Subsystems ====================

  /** Limelight subsystem - for robot localization using MegaTag. */
  public final LimelightSubsystem limelight = new LimelightSubsystem("limelight", drivetrain);

  /**
   * AprilTag tracker - for targeting specific tags (fuel hub).
   *
   * <p>This is separate from the Limelight subsystem because it serves
   * a different purpose: targeting vs localization.
   */
  public final AprilTagTracker aprilTagTracker = new AprilTagTracker("limelight");

  // ==================== Autonomous Chooser ====================

  private final SendableChooser<Command> autoChooser;
  private final AutoRoutines autoRoutines;

  // ==================== Constructor ====================

  /**
   * Creates the RobotContainer and sets up all subsystems and controls.
   *
   * <p>This runs once when the robot code starts. It:
   * <ol>
   *   <li>Creates all subsystems (done above as field initializers)</li>
   *   <li>Sets up autonomous mode selector</li>
   *   <li>Configures all button bindings</li>
   *   <li>Sets default commands for subsystems</li>
   * </ol>
   */
  public RobotContainer() {
    // Set up autonomous routines
    autoChooser = new SendableChooser<>();
    autoRoutines = new AutoRoutines(autoCommands, superstructure);

    autoChooser.addOption("Mobility Auto", autoRoutines.mobilityAuto());
    SmartDashboard.putData("Auto Mode", autoChooser);

    // Set default alliance targeting
    aprilTagTracker.setAlliance(AllianceConstants.DEFAULT_ALLIANCE);

    // Configure all button bindings
    configureDriverBindings();
    configureOperatorBindings();
  }

  // ==================== Driver Controls ====================

  /**
   * Configures the DRIVER controller button bindings.
   *
   * <p>The driver controls:
   * <ul>
   *   <li>Robot movement (left stick = translation, right stick = rotation)</li>
   *   <li>Vision lock driving (LT = auto-aim while driving)</li>
   *   <li>Slow mode (RT = precision driving)</li>
   *   <li>Alliance selection (LB = blue, RB = red)</li>
   *   <li>Reset heading (Start)</li>
   *   <li>Toggle field/robot centric (Back)</li>
   * </ul>
   */
  private void configureDriverBindings() {
    // ========== DEFAULT COMMAND: Normal Driving ==========
    // This runs whenever no other command is using the drivetrain.
    // Left stick = forward/backward and strafe
    // Right stick = rotation
    drivetrain.setDefaultCommand(
        drivetrain.applyRequest(
            () -> {
              // Get raw inputs
              double forward = -driverController.getLeftY(); // Inverted for correct direction
              double strafe = -driverController.getLeftX(); // Inverted for correct direction
              double rotation = -driverController.getRightX(); // Inverted for correct direction

              // Apply deadband
              forward = MathUtil.applyDeadband(forward, DriveConstants.JOYSTICK_DEADBAND);
              strafe = MathUtil.applyDeadband(strafe, DriveConstants.JOYSTICK_DEADBAND);
              rotation = MathUtil.applyDeadband(rotation, DriveConstants.JOYSTICK_DEADBAND);

              // Apply input curve (squaring for more precision at low speeds)
              forward = Math.copySign(Math.pow(forward, DriveConstants.INPUT_CURVE_EXPONENT), forward);
              strafe = Math.copySign(Math.pow(strafe, DriveConstants.INPUT_CURVE_EXPONENT), strafe);
              rotation = Math.copySign(Math.pow(rotation, DriveConstants.INPUT_CURVE_EXPONENT), rotation);

              // Apply slow mode if RT is held
              double speedMultiplier =
                  driverController.getRightTriggerAxis() > 0.5
                      ? DriveConstants.SLOW_MODE_MULTIPLIER
                      : DriveConstants.NORMAL_MODE_MULTIPLIER;

              // Calculate velocities
              double velocityX = forward * maxSpeed * speedMultiplier;
              double velocityY = strafe * maxSpeed * speedMultiplier;
              double rotationRate = rotation * maxAngularRate * speedMultiplier;

              return fieldCentricDrive
                  .withVelocityX(velocityX)
                  .withVelocityY(velocityY)
                  .withRotationalRate(rotationRate);
            }));

    // ========== LT: Vision Lock Driving ==========
    // Hold LT to drive while automatically facing the alliance fuel hub.
    // The robot will rotate to face the hub while you control translation.
    driverController
        .leftTrigger()
        .whileTrue(
            new VisionLockDriveCommand(
                drivetrain,
                aprilTagTracker,
                () -> -driverController.getLeftY(),
                () -> -driverController.getLeftX(),
                () -> {
                  // Apply slow mode multiplier if RT is also held
                  double multiplier =
                      driverController.getRightTriggerAxis() > 0.5
                          ? DriveConstants.SLOW_MODE_MULTIPLIER
                          : DriveConstants.NORMAL_MODE_MULTIPLIER;
                  return maxSpeed * multiplier;
                }));

    // ========== LB: Target BLUE Alliance Hub ==========
    // Press to switch vision tracking to the blue alliance fuel hub.
    driverController
        .leftBumper()
        .onTrue(
            drivetrain
                .runOnce(() -> aprilTagTracker.setAlliance(Alliance.BLUE))
                .withName("TargetBlueHub"));

    // ========== RB: Target RED Alliance Hub ==========
    // Press to switch vision tracking to the red alliance fuel hub.
    driverController
        .rightBumper()
        .onTrue(
            drivetrain
                .runOnce(() -> aprilTagTracker.setAlliance(Alliance.RED))
                .withName("TargetRedHub"));

    // ========== Start: Reset Robot Heading ==========
    // Press to reset the gyro - "forward" becomes the current direction.
    // Use this when the robot's "forward" doesn't match the field.
    driverController
        .start()
        .onTrue(
            drivetrain.runOnce(
                () -> drivetrain.resetPose(new Pose2d(Feet.of(0), Feet.of(0), Rotation2d.kZero))));

    // ========== Back: Toggle Field-Centric / Robot-Centric ==========
    // Press to switch between field-centric and robot-centric driving.
    // Field-centric (default): Forward is always toward the far field end.
    // Robot-centric: Forward is wherever the robot is pointing.
    driverController
        .back()
        .onTrue(
            drivetrain.runOnce(
                () -> {
                  isFieldCentric = !isFieldCentric;
                  System.out.println(
                      "Drive mode: " + (isFieldCentric ? "Field-Centric" : "Robot-Centric"));
                }));
  }

  // ==================== Operator Controls ====================

  /**
   * Configures the OPERATOR controller button bindings.
   *
   * <p>The operator controls:
   * <ul>
   *   <li>Intake (LT = intake in, LB = intake out/eject)</li>
   *   <li>Shooter (RT = spin up flywheel)</li>
   *   <li>Feeder (RB = feed balls to shooter)</li>
   *   <li>Climb (Y = flip up, A = flip down, X = lift, B = drop)</li>
   * </ul>
   */
  private void configureOperatorBindings() {

    // ========================= INTAKE CONTROLS =========================

    // ========== LT: Intake IN ==========
    // Hold to run intake and pick up balls from the floor.
    operatorController.leftTrigger().whileTrue(intake.intakeCommand());

    // ========== LB: Intake OUT (Eject) ==========
    // Hold to run intake in reverse - ejects balls.
    operatorController.leftBumper().whileTrue(intake.outtakeCommand());

    // ========================= SHOOTER CONTROLS =========================

    // ========== RT: Spin Up Shooter ==========
    // Hold to spin up the shooter flywheels.
    // IMPORTANT: Hold this BEFORE and WHILE feeding balls!
    //
    // Workflow:
    // 1. Hold RT (shooter spins up)
    // 2. Wait ~1 second for full speed
    // 3. Press RB to feed a ball
    // 4. Keep holding RT for more shots
    // 5. Release RT when done
    operatorController.rightTrigger().whileTrue(shooter.spinUpCommand());

    // ========== RB: Feed Ball to Shooter ==========
    // Hold to run the feeder - pushes balls into the shooter.
    // Only use this AFTER the shooter is spun up (holding RT)!
    operatorController.rightBumper().whileTrue(feeder.feedCommand());

    // ========================= CLIMB CONTROLS =========================
    // NOTE: These are STUBS - they don't do anything yet!
    // See Climb.java for implementation instructions.

    // ========== Y: Flip Up ==========
    // Hold to flip the climb arm upward (reach for bar).
    operatorController.y().whileTrue(climb.flipUpCommand());

    // ========== A: Flip Down ==========
    // Hold to flip the climb arm downward (retract).
    operatorController.a().whileTrue(climb.flipDownCommand());

    // ========== X: Lift Robot ==========
    // Hold to winch up / climb.
    operatorController.x().whileTrue(climb.liftRobotCommand());

    // ========== B: Drop Robot ==========
    // Hold to lower / descend.
    operatorController.b().whileTrue(climb.dropRobotCommand());
  }

  // ==================== Autonomous ====================

  /**
   * Gets the autonomous command selected on the dashboard.
   *
   * @return The selected autonomous command
   */
  public Command getAutonomousCommand() {
    return autoChooser.getSelected();
  }

  // ==================== Utility Methods ====================

  /**
   * Rescales translation input with deadband and squaring.
   *
   * @deprecated Use inline processing in default command instead
   */
  public Vector<N2> rescaleTranslation(double x, double y) {
    Vector<N2> scaledJoyStick = VecBuilder.fill(x, y);
    scaledJoyStick = MathUtil.applyDeadband(scaledJoyStick, DriveConstants.JOYSTICK_DEADBAND);
    return MathUtil.copyDirectionPow(scaledJoyStick, DriveConstants.INPUT_CURVE_EXPONENT);
  }

  /**
   * Rescales rotation input with deadband.
   *
   * @deprecated Use inline processing in default command instead
   */
  public double rescaleRotation(double rotation) {
    return Math.copySign(MathUtil.applyDeadband(rotation, DriveConstants.JOYSTICK_DEADBAND), 2);
  }
}
