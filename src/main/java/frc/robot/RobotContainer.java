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
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.autonomous.AutoCommands;
import frc.robot.autonomous.AutoRoutines;
import frc.robot.autonomous.LinearPathRequest;
import frc.robot.constants.AutoConstants;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.Superstructure;
import frc.robot.subsystems.climb.Climb;
import frc.robot.subsystems.shooter.Feeder;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.vision.LimelightSubsystem;

/**
 * RobotContainer - Sets up all the robot's parts and controls.
 *
 * <p>This class handles:
 * <ul>
 *   <li>Creating all subsystems (drive, climb, shooter, feeder, vision, etc.)
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

  private final Superstructure superstructure = new Superstructure(shooter, feeder);

  // Vision camera for tracking robot position and measuring distance to AprilTags
  public final LimelightSubsystem limelight = new LimelightSubsystem("limelight", drivetrain);

  /* Autonomous mode selector */
  private final SendableChooser<Command> autoChooser;
  private final AutoRoutines autoRoutines;

  public RobotContainer() {
    autoChooser = new SendableChooser<>();
    autoRoutines = new AutoRoutines(autoCommands, superstructure);

    autoChooser.addOption("Mobility Auto", autoRoutines.mobilityAuto());

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

    // Right trigger — hold to shoot.
    // Both shooter wheels and feeder run simultaneously.
    // Everything stops the moment the trigger is released.
    joystick.rightTrigger(0.1).whileTrue(superstructure.teleOpShootCommand());

    // --- Distance-based shooting (bonus) ---
    // Uncomment the line below (and comment out the line above) to enable automatic
    // speed adjustment based on AprilTag distance from the Limelight.
    // joystick.rightTrigger(0.1).whileTrue(superstructure.teleOpShootWithDistanceCommand(limelight::getAvgTagDistance));
  }

  public Command getAutonomousCommand() {
    return autoChooser.getSelected();
  }

  public Vector<N2> rescaleTranslation(double x, double y) {
    Vector<N2> scaledJoyStick = VecBuilder.fill(x, y);
    scaledJoyStick = MathUtil.applyDeadband(scaledJoyStick, 0.1);
    return MathUtil.copyDirectionPow(scaledJoyStick, 2);
  }

  public double rescaleRotation(double rotation){
    return Math.copySign(MathUtil.applyDeadband(rotation, 1), 2);
  }
}
