package frc.robot.autonomous;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveModule.SteerRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.commands.FaceTagCommand;
import frc.robot.constants.VisionConstants;
import frc.robot.constants.Waypoints;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.Superstructure;
import frc.robot.subsystems.vision.LimelightSubsystem;
import java.util.Set;


public class AutoRoutines {

  private final AutoCommands autoCommands;
  private final Superstructure superstructure;
  private final CommandSwerveDrivetrain drivetrain;
  private final LimelightSubsystem limelight;

  // Backward drive request (robot-relative, -X = backward)
  // If the robot needs to move forward instead, change -0.5 to +0.5
  private final SwerveRequest.RobotCentric driveBackward =
      new SwerveRequest.RobotCentric()
          .withDriveRequestType(DriveRequestType.Velocity)
          .withSteerRequestType(SteerRequestType.MotionMagicExpo)
          .withVelocityX(-0.5);

  public AutoRoutines(
      AutoCommands autoCommands,
      Superstructure superstructure,
      CommandSwerveDrivetrain drivetrain,
      LimelightSubsystem limelight) {
    this.autoCommands = autoCommands;
    this.superstructure = superstructure;
    this.drivetrain = drivetrain;
    this.limelight = limelight;
  }

  // ============================================================
  // VISION AUTO  —  shared logic for all starting positions
  //
  //   1. Reset pose to starting position
  //   2. Drive backward until the hub AprilTag is visible (max 4 sec)
  //   3. Lock rotation onto the hub tag + spin up shooter simultaneously
  //   4. Shoot
  //   5. Stow
  // ============================================================
  private Command visionDriveAndShoot(Pose2d startPose) {
    return Commands.defer(() -> {
      boolean isBlue = DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Blue;
      int[] hubTags = isBlue ? VisionConstants.BLUE_HUB_TAG_IDS : VisionConstants.RED_HUB_TAG_IDS;

      return Commands.sequence(
          autoCommands.resetPose(startPose),

          // Drive backward until a hub tag appears, then stop (4-second timeout)
          Commands.race(
              drivetrain.applyRequest(() -> driveBackward),
              Commands.waitUntil(() -> limelight.getTXForTags(hubTags).isPresent())
          ).withTimeout(4.0),

          // Lock rotation onto the hub tag and spin up shooter at the same time
          Commands.parallel(
              new FaceTagCommand(drivetrain, limelight, -1.0).withTimeout(2.0),
              superstructure.speakerCloseAndWaitCommand().withTimeout(3.0)
          ),

          // Fire and stow
          superstructure.shootCommand(),
          superstructure.stowCommand()
      );
    }, Set.of(drivetrain));
  }

  // ============================================================
  // LEFT START  —  robot begins at the left side of the alliance wall
  //   Starting pose: START_LEFT  (x=7.15, y=6.05, heading=-120°)
  // ============================================================
  public Command leftStartAuto() {
    return visionDriveAndShoot(Waypoints.START_LEFT);
  }

  // ============================================================
  // CENTER START  —  robot begins in the center of the alliance wall
  //   Starting pose: START_CENTER  (x=7.15, y=4.05, heading=0°)
  // ============================================================
  public Command centerStartAuto() {
    return visionDriveAndShoot(Waypoints.START_CENTER);
  }

  // ============================================================
  // RIGHT START  —  robot begins at the right side of the alliance wall
  //   Starting pose: START_RIGHT  (x=7.15, y=2.05, heading=120°)
  // ============================================================
  public Command rightStartAuto() {
    return visionDriveAndShoot(Waypoints.START_RIGHT);
  }

  /*
  public Command sequentialScoringAuto() {
    return Commands.sequence(
        Commands.print("=== Sequential Scoring Auto ==="),
        autoCommands.resetPose(Pose2d.kZero),
        superstructure.speakerCloseAndWaitCommand(),
        superstructure.shootCommand(),
        autoCommands.driveTo(new Pose2d(3.0, 0, Rotation2d.kZero)),
        superstructure.intakeGroundAndWaitCommand(),
        Commands.waitSeconds(0.5),
        autoCommands.driveTo(new Pose2d(4.0, 2.0, Rotation2d.kZero)),
        superstructure.speakerFarAndWaitCommand(),
        superstructure.shootCommand(),
        superstructure.stowAndWaitCommand(),
        Commands.print("=== Complete ==="));
  }

  public Command parallelPreparationAuto() {
    return Commands.sequence(
        Commands.print("=== Parallel Preparation Auto ==="),
        autoCommands.resetPose(Pose2d.kZero),
        superstructure.speakerCloseCommand(),
        Commands.waitSeconds(1.0),
        superstructure.shootCommand(),
        Commands.parallel(
            autoCommands.driveTo(new Pose2d(3.0, 0, Rotation2d.kZero)),
            Commands.sequence(
                Commands.waitSeconds(0.5),
                superstructure.intakeGroundCommand())),
        Commands.waitSeconds(0.5),
        Commands.parallel(
            autoCommands.driveTo(new Pose2d(1.0, 4.0, Rotation2d.kZero)),
            Commands.sequence(
                Commands.waitSeconds(0.5),
                superstructure.ampScoreCommand())),
        superstructure.shootCommand(),
        superstructure.stowCommand(),
        Commands.print("=== Complete ==="));
  }

  public Command mobilityAuto() {
    return Commands.sequence(
        Commands.print("=== Mobility Auto ==="),
        superstructure.stowCommand(),
        autoCommands.driveTo(Waypoints.MIDFIELD_CENTER),
        Commands.print("=== Complete ==="));
  }

  public Command driveBackAndForthAuto() {
    return Commands.sequence(
        Commands.print("=== Drive Back and Forth Auto ==="),
        autoCommands.resetPose(Pose2d.kZero),
        superstructure.stowCommand(),
        Commands.sequence(
                Commands.print("Driving forward..."),
                autoCommands.driveTo(Waypoints.MIDFIELD_CENTER),
                Commands.print("Driving back..."),
                autoCommands.driveTo(Pose2d.kZero))
            .repeatedly());
  }

  public Command demonstrationAuto() {
    return Commands.sequence(
        Commands.print("=== Demonstration Auto ==="),
        autoCommands.resetPose(Pose2d.kZero),
        Commands.print("1. Stowing for transport"),
        superstructure.stowAndWaitCommand(),
        Commands.waitSeconds(0.5),
        Commands.print("2. Amp scoring (slow speed)"),
        superstructure.ampScoreAndWaitCommand(),
        Commands.waitSeconds(0.5),
        Commands.print("3. Close speaker shot (medium speed)"),
        superstructure.speakerCloseAndWaitCommand(),
        Commands.waitSeconds(0.5),
        Commands.print("4. Far speaker shot (fast speed)"),
        superstructure.speakerFarAndWaitCommand(),
        Commands.waitSeconds(0.5),
        Commands.print("5. Ground intake position"),
        superstructure.intakeGroundAndWaitCommand(),
        Commands.waitSeconds(0.5),
        Commands.print("Returning to stow"),
        superstructure.stowAndWaitCommand(),
        Commands.print("=== Complete ==="));
  }
  */
}
