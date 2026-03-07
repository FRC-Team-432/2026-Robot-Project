package frc.robot.autonomous;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.constants.VisionConstants;
import frc.robot.constants.Waypoints;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.Superstructure;
import frc.robot.subsystems.climb.Climb;
import frc.robot.subsystems.vision.LimelightSubsystem;
import java.util.Set;


public class AutoRoutines {

  private final AutoCommands autoCommands;
  private final Superstructure superstructure;
  private final CommandSwerveDrivetrain drivetrain;
  private final LimelightSubsystem limelight;
  private final Climb climb;

  public AutoRoutines(
      AutoCommands autoCommands,
      Superstructure superstructure,
      CommandSwerveDrivetrain drivetrain,
      LimelightSubsystem limelight,
      Climb climb) {
    this.autoCommands = autoCommands;
    this.superstructure = superstructure;
    this.drivetrain = drivetrain;
    this.limelight = limelight;
    this.climb = climb;
  }

  // ============================================================
  // VISION AUTO  —  shared logic for all starting positions
  //
  //   1. Reset pose to starting position
  //   2. Drive backward until any AprilTag is visible (max 6 sec), then STOP.
  //   3. Rotate in place until the hub tag is centered (max 5 sec), then STOP.
  //   4. Spin up shooter at area-based speed (max 3 sec)
  //   5. Shoot + Stow
  //   6. Mandatory blind spin then search for climb tags
  //   7. Climb up (max 4 sec)
  // ============================================================
  private Command visionDriveAndShoot(Pose2d startPose) {
    return Commands.defer(() -> {
      // Resolve tag IDs at schedule time so alliance is known.
      boolean isBlue = DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Blue;
      int[] hubTagIds = isBlue
          ? VisionConstants.BLUE_HUB_ALL_TAG_IDS
          : VisionConstants.RED_HUB_ALL_TAG_IDS;
      int centerTagId = isBlue
          ? VisionConstants.BLUE_HUB_CENTER_TAG_IDS[0]
          : VisionConstants.RED_HUB_CENTER_TAG_IDS[0];
      int[] climbTagIds = isBlue
          ? VisionConstants.BLUE_CLIMB_TAG_IDS
          : VisionConstants.RED_CLIMB_TAG_IDS;

      return Commands.sequence(
          // Setup
          autoCommands.resetPose(startPose),
          autoCommands.log("AUTO: Phase 1 - driving backward until tag visible"),

          // Phase 1: Drive backward until hub tag visible
          autoCommands.driveBackwardUntilTag(limelight, hubTagIds, 0.5, 6.0),
          autoCommands.log("AUTO: Phase 2 - aligning to hub center tag"),

          // Phase 2: Rotate to center on hub tag
          autoCommands.alignToHubTag(limelight, centerTagId, 5.0),
          autoCommands.log("AUTO: Phase 3 - spinning up shooter"),

          // Phase 3: Spin up shooter at area-based speed, wait until ready
          superstructure.spinUpForAreaAndWaitCommand(limelight::getTargetArea).withTimeout(3.0),
          autoCommands.log("AUTO: Phase 4 - shooting"),

          // Phase 4: Fire and stow
          superstructure.shootCommand(),
          superstructure.stowCommand(),
          autoCommands.log("AUTO: Phase 5 - searching for climb tags"),

          // Phase 5: Mandatory blind spin then search for climb tags
          autoCommands.blindSpin(0.6, 1.5),
          autoCommands.spinToFindTag(limelight, climbTagIds, 0.6, 8.5),
          autoCommands.log("AUTO: Phase 6 - climbing"),

          // Phase 6: Climb
          climb.climbUpCommand().withTimeout(4.0),
          autoCommands.log("AUTO: Complete")
      );
    }, Set.of(drivetrain, climb));
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
