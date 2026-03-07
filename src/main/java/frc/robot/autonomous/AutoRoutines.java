package frc.robot.autonomous;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveModule.SteerRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.Debouncer.DebounceType;
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
      LimelightSubsystem limelight,
      Climb climb) {
    this.autoCommands = autoCommands;
    this.superstructure = superstructure;
    this.drivetrain = drivetrain;
    this.limelight = limelight;
    this.climb = climb;
  }

  // Combined drive+rotate request — reused each loop inside the lambda below
  private final SwerveRequest.RobotCentric driveAndAlign =
      new SwerveRequest.RobotCentric()
          .withDriveRequestType(DriveRequestType.Velocity)
          .withSteerRequestType(SteerRequestType.MotionMagicExpo);

  // ============================================================
  // VISION AUTO  —  shared logic for all starting positions
  //
  //   1. Reset pose to starting position
  //   2. Drive backward until any AprilTag is visible (max 6 sec), then STOP.
  //   3. Rotate in place until the hub tag is centered (max 2 sec), then STOP.
  //   4. Spin up shooter (max 3 sec)
  //   5. Shoot + Stow
  //   6. Spin CCW until alliance climb tags are visible (max 5 sec)
  //   7. Climb up (max 4 sec)
  // ============================================================
  private Command visionDriveAndShoot(Pose2d startPose) {
    return Commands.defer(() -> {
      // Require the robot to be centered on the tag for this long continuously before
      // proceeding. Prevents stopping the instant the tag first flickers into view.
      Debouncer centeredDebouncer = new Debouncer(0.05, DebounceType.kRising);

      // Resolve tag IDs at schedule time so alliance is known.
      boolean isBlue = DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Blue;
      int[] hubTagIds = isBlue
          ? VisionConstants.BLUE_HUB_ALL_TAG_IDS
          : VisionConstants.RED_HUB_ALL_TAG_IDS;
      // The center hub tag is the final alignment target before shooting.
      // Blue = tag 26 (front face), Red = tag 10 (front face).
      int centerTagId = isBlue
          ? VisionConstants.BLUE_HUB_CENTER_TAG_IDS[0]
          : VisionConstants.RED_HUB_CENTER_TAG_IDS[0];
      int[] climbTagIds = isBlue
          ? VisionConstants.BLUE_CLIMB_TAG_IDS
          : VisionConstants.RED_CLIMB_TAG_IDS;

      return Commands.sequence(
          autoCommands.resetPose(startPose),
          Commands.print("AUTO: Phase 1 — driving backward until tag visible"),

          // STEP 1 — Drive straight backward until a hub tag appears (max 6 sec).
          // Uses hasSpecificTag() with hub IDs — checks rawfiducials + tid, proven reliable.
          drivetrain.applyRequest(() -> driveBackward)
              .until(() -> limelight.hasSpecificTag(hubTagIds))
              .withTimeout(6.0),
          Commands.print("AUTO: Phase 2 — rotating to center on hub tag"),

          // STEP 2 — Turn toward whichever hub tag is visible, then P-control once the
          // center tag (26 blue / 10 red) is the primary target.
          //
          // Three-way logic evaluated each loop cycle:
          //   a) Center tag is already primary  → P-control to align robot center on it.
          //   b) A side tag is primary (id != 0) → spin TOWARD it using its TX sign.
          //        TX > 0 (tag right of camera center) → rotate CW  (−) toward it.
          //        TX < 0 (tag left  of camera center) → rotate CCW (+) toward it.
          //      The center tag is on the face just past the side face, so spinning
          //      toward the side tag naturally brings the center tag into view.
          //      This is correct for all starting positions — direction is data-driven,
          //      not hardcoded, so left/center/right starts all work without special cases.
          //   c) No tag visible → slow CCW fallback spin.
          drivetrain.applyRequest(() -> {
            double rotRate;
            int primaryId = limelight.getTargetId();
            if (primaryId == centerTagId) {
              // Center tag is primary — P-control to align robot center on it
              double error = limelight.getTargetTX() - VisionConstants.CAMERA_TX_OFFSET_DEG;
              rotRate = MathUtil.clamp(
                  -error * VisionConstants.DRIVE_TO_TAG_TURN_KP,
                  -VisionConstants.DRIVE_TO_TAG_MAX_ROTATION_RAD_S,
                  VisionConstants.DRIVE_TO_TAG_MAX_ROTATION_RAD_S);
            } else if (primaryId != 0) {
              // Side tag visible — spin toward it; the center tag is just past it.
              // TX sign tells us which way "toward" is.
              double tx = limelight.getTargetTX();
              rotRate = (tx > 0) ? -0.3 : 0.3;
            } else {
              // No tag visible — slow fallback spin
              rotRate = 0.3;
            }
            return driveAndAlign.withVelocityX(0.0).withRotationalRate(rotRate);
          }).until(() -> centeredDebouncer.calculate(
              limelight.getTargetId() == centerTagId
                  && Math.abs(limelight.getTargetTX() - VisionConstants.CAMERA_TX_OFFSET_DEG)
                      <= 3.0)) // exit only once tag 26/10 is centered for 0.05 s
          .withTimeout(5.0),

          Commands.print("AUTO: Phase 3 — spinning up shooter"),

          // STEP 3 — Spin up the shooter (robot holds position via default command)
          superstructure.speakerCloseAndWaitCommand().withTimeout(3.0),
          Commands.print("AUTO: Phase 4 — shooting"),

          // STEP 4 — Fire and stow
          superstructure.shootCommand(),
          superstructure.stowCommand(),
          Commands.print("AUTO: Phase 5 — spinning to find climb tags"),

          // STEP 5 — Spin CCW to find climb tags (31/32 blue, 15/16 red).
          // Split into two parts to prevent instant exit when climb tags are already visible
          // from the hub shooting position:
          //   Part A: mandatory 1.5 s blind spin to rotate away from the hub.
          //           At 0.6 rad/s this is ~52° — enough to clear any hub-area sightlines.
          //   Part B: continue spinning and exit as soon as a climb tag is seen (max 8.5 s).
          //           8.5 s + 1.5 s = 10 s total, same budget as before.
          Commands.print("AUTO: Phase 5A — mandatory spin away from hub (1.5 s)"),
          drivetrain.applyRequest(() ->
              driveAndAlign
                  .withVelocityX(0.0)
                  .withRotationalRate(0.6))
              .withTimeout(1.5),
          Commands.print("AUTO: Phase 5B — spinning to find climb tags"),
          drivetrain.applyRequest(() ->
              driveAndAlign
                  .withVelocityX(0.0)
                  .withRotationalRate(0.6))
              .until(() -> limelight.hasSpecificTag(climbTagIds))
              .withTimeout(8.5)
              .andThen(Commands.print("AUTO: Phase 5 COMPLETE — climb tag detected, stopping spin")),

          Commands.print("AUTO: Phase 6 — climbing"),

          // STEP 6 — Climb up for a fixed duration.
          // TODO: Tune the timeout once the climb mechanism is characterized.
          climb.climbUpCommand().withTimeout(4.0),
          Commands.print("AUTO: Complete")
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
