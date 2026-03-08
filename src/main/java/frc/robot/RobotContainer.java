// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveModule.SteerRequestType;
import com.ctre.phoenix6.swerve.utility.WheelForceCalculator;
import com.ctre.phoenix6.swerve.SwerveRequest;
import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.numbers.N2;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import java.util.Set;
import frc.robot.autonomous.AutoCommands;
import frc.robot.autonomous.AutoRoutines;
import frc.robot.autonomous.LinearPathRequest;
import frc.robot.commands.DriveAndLockCommand;
import frc.robot.commands.FaceTagCommand;
import frc.robot.constants.AutoConstants;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.Superstructure;
import frc.robot.subsystems.climb.Climb;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.shooter.Feeder;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.vision.LimelightSubsystem;

/**
 * RobotContainer - Sets up all the robot's parts and controls.
 *
 * <p>This class handles:
 * <ul>
 *   <li>Creating all subsystems (drive, climb, shooter, feeder, intake, vision, etc.)
 *   <li>Setting up controller buttons
 *   <li>Building autonomous routines
 *   <li>Setting default actions for each subsystem
 * </ul>
 */
@Logged
public class RobotContainer {
  private double MaxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond);
  private double MaxAngularRate =
      RotationsPerSecond.of(1).in(RadiansPerSecond);

  /* Configure field-centric driving (forward is always away from driver) */
  private final SwerveRequest.FieldCentric drive =
      new SwerveRequest.FieldCentric()
          .withDriveRequestType(DriveRequestType.Velocity)
          .withSteerRequestType(SteerRequestType.MotionMagicExpo);

  private final CommandXboxController joystick = new CommandXboxController(0);

  public final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();

  public final AutoCommands autoCommands = new AutoCommands(
      drivetrain,
      new LinearPathRequest(
          new Constraints(AutoConstants.MAX_LINEAR_VELOCITY_MPS, AutoConstants.MAX_LINEAR_ACCELERATION_MPS2),
          new Constraints(AutoConstants.MAX_ANGULAR_VELOCITY_RAD_S, AutoConstants.MAX_ANGULAR_ACCELERATION_RAD_S2),
          new WheelForceCalculator(
              drivetrain.getModuleLocations(),
              Pounds.of(AutoConstants.ROBOT_MASS_LBS),
              KilogramSquareMeters.of(AutoConstants.MOMENT_OF_INERTIA_KG_M2))));

  /* Subsystems */
  public final Climb climb = new Climb();
  public final Shooter shooter = new Shooter();
  public final Feeder feeder = new Feeder();
  public final Intake intake = new Intake();

  private final Superstructure superstructure = new Superstructure(shooter, feeder);

  // Front camera — AprilTag odometry, hub tracking, and climb tag detection
  public final LimelightSubsystem limelight = new LimelightSubsystem("limelight", drivetrain);

  /* Autonomous mode selector */
  private final SendableChooser<Command> autoChooser;
  private final AutoRoutines autoRoutines;

  public RobotContainer() {
    // ==================== PathPlanner Setup ====================
    // Configure AutoBuilder so PathPlanner can control the drivetrain.
    // Uses the same PID gains already in AutoConstants.
    SwerveRequest.ApplyRobotSpeeds autoRequest = new SwerveRequest.ApplyRobotSpeeds();
    try {
      AutoBuilder.configure(
          drivetrain::getPose,
          drivetrain::resetPose,
          drivetrain::getRobotSpeeds,
          (speeds, feedforwards) -> drivetrain.setControl(
              autoRequest.withSpeeds(speeds)
                  .withWheelForceFeedforwardsX(feedforwards.robotRelativeForcesXNewtons())
                  .withWheelForceFeedforwardsY(feedforwards.robotRelativeForcesYNewtons())),
          new PPHolonomicDriveController(
              new PIDConstants(AutoConstants.X_CONTROLLER_KP, 0, 0),   // translation
              new PIDConstants(AutoConstants.THETA_CONTROLLER_KP, 0, 0) // rotation
          ),
          RobotConfig.fromGUISettings(),
          () -> DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Red,
          drivetrain
      );
    } catch (Exception e) {
      DriverStation.reportError("PathPlanner AutoBuilder setup failed: " + e.getMessage(), e.getStackTrace());
    }

    // ==================== PathPlanner Named Commands ====================
    // These let PathPlanner trigger robot actions by name via Event Markers.
    NamedCommands.registerCommand("shoot",
        superstructure.speakerCloseAndWaitCommand()
            .andThen(superstructure.shootCommand())
            .andThen(superstructure.stowCommand()));

    autoChooser = new SendableChooser<>();
    autoRoutines = new AutoRoutines(autoCommands, superstructure, drivetrain, limelight);

    // ---- Vision Autos (primary) ----
    // Alliance is read at enable time — robot backs up until it sees the hub AprilTag,
    // locks on, then shoots. No pre-planned path needed.
    autoChooser.setDefaultOption("Center Start", autoRoutines.centerStartAuto());
    autoChooser.addOption("Left Start", autoRoutines.leftStartAuto());
    autoChooser.addOption("Right Start", autoRoutines.rightStartAuto());

    // ---- PathPlanner Autos (backup) ----
    // Pre-planned paths. Use these if vision auto is not working on the day.
    autoChooser.addOption("PathPlanner - Center", Commands.defer(
        () -> DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Blue
            ? AutoBuilder.buildAuto("StartCenterAutoBlue")
            : AutoBuilder.buildAuto("StartCenterAutoRed"),
        Set.of(drivetrain)));

    autoChooser.addOption("PathPlanner - Left", Commands.defer(
        () -> DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Blue
            ? AutoBuilder.buildAuto("LeftStartAutoBlue")
            : AutoBuilder.buildAuto("LeftStartAutoRed"),
        Set.of(drivetrain)));

    autoChooser.addOption("PathPlanner - Right", Commands.defer(
        () -> DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Blue
            ? AutoBuilder.buildAuto("RightStartAutoBlue")
            : AutoBuilder.buildAuto("RightStartAutoRed"),
        Set.of(drivetrain)));

    SmartDashboard.putData("Auto Mode", autoChooser);

    configureBindings();
  }

  private void configureBindings() {
    // ==================== Drive ====================
    drivetrain.setDefaultCommand(
        drivetrain.applyRequest(
            () -> {
              Vector<N2> scaledInputs = rescaleTranslation(joystick.getLeftY(), joystick.getLeftX());
              return drive
                  .withVelocityX(-scaledInputs.get(0, 0) * MaxSpeed)
                  .withVelocityY(-scaledInputs.get(1, 0) * MaxSpeed)
                  .withRotationalRate(-rescaleRotation(joystick.getRightX()) * MaxAngularRate);
            }));

    // Reset field-centric heading
    joystick
        .start()
        .onTrue(
            drivetrain.runOnce(
                () -> drivetrain.resetPose(new Pose2d(Feet.of(0), Feet.of(0), Rotation2d.kZero))));

    // ==================== Climb ====================
    // Y — hold to climb up. Motor brakes and holds position on release.
    joystick.y().whileTrue(climb.climbUpCommand());

    // A — hold to climb down. Motor brakes and holds position on release.
    joystick.a().whileTrue(climb.climbDownCommand());

    // ==================== Shooter ====================
    // Right trigger — hold to shoot with area-based speed adjustment.
    // Shooter speed adjusts based on AprilTag size (closer = slower, farther = faster).
    // Both shooter wheels + feeder run simultaneously. Stops on trigger release.
    joystick.rightTrigger(0.1).whileTrue(superstructure.teleOpShootWithAreaCommand(limelight::getTargetArea));

    // ==================== Intake ====================
    // Left trigger — hold to intake. Left bumper — hold to eject.
    joystick.leftTrigger().whileTrue(intake.intake());
    joystick.leftBumper().whileTrue(intake.eject());

    // ==================== Vision ====================
    // Right bumper — face the hub AprilTag (searches clockwise if not in view)
    joystick.rightBumper().whileTrue(new FaceTagCommand(drivetrain, limelight, -1.0));

    // X — drive normally while keeping rotation locked on hub tag
    joystick.x().whileTrue(new DriveAndLockCommand(
        drivetrain,
        limelight,
        () -> -rescaleTranslation(joystick.getLeftY(), joystick.getLeftX()).get(0, 0) * MaxSpeed,
        () -> -rescaleTranslation(joystick.getLeftY(), joystick.getLeftX()).get(1, 0) * MaxSpeed
    ));
  }

  public Command getAutonomousCommand() {
    return autoChooser.getSelected();
  }

  public Vector<N2> rescaleTranslation(double x, double y) {
    Vector<N2> scaledJoyStick = VecBuilder.fill(x, y);
    scaledJoyStick = MathUtil.applyDeadband(scaledJoyStick, 0.1);
    return MathUtil.copyDirectionPow(scaledJoyStick, 2);
  }

  public double rescaleRotation(double rotation) {
    double deadbanded = MathUtil.applyDeadband(rotation, 0.1);
    return Math.copySign(deadbanded * deadbanded, deadbanded);
  }
}
