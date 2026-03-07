package frc.robot.autonomous;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveModule.SteerRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.constants.VisionConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.vision.LimelightSubsystem;

/**
 * Pre-built driving commands for autonomous mode.
 *
 * <p>Ready-to-use commands for common tasks:
 *
 * <ul>
 *   <li>Drive to a location (driveTo)
 *   <li>Drive while doing something else (driveToWithAction)
 *   <li>Drive then do something when you arrive (driveToThenExecute)
 *   <li>Start actions when close to target (distanceCommand)
 *   <li>Reset the robot's starting position (resetPose)
 *   <li>Vision-driven blocks (driveBackwardUntilTag, alignToHubTag, spinToFindTag, blindSpin)
 *   <li>Utility blocks (waitSeconds, log)
 * </ul>
 *
 * <p><b>Example:</b>
 * <pre>{@code
 * // Drive to a spot while running the intake
 * driveToWithAction(targetPose, intake.intakeCommand())
 *
 * // Start spinning flywheel when 1.5 meters from shooting position
 * distanceCommand(1.5, shootPose, spinFlywheelCommand())
 * }</pre>
 */
public class AutoCommands {

  // Subsystems
  private final CommandSwerveDrivetrain drivetrain;

  private final LinearPathRequest pathRequest;

  // Robot-centric request for vision-driven blocks (backward drive, spin, align)
  private final SwerveRequest.RobotCentric robotCentric =
      new SwerveRequest.RobotCentric()
          .withDriveRequestType(DriveRequestType.Velocity)
          .withSteerRequestType(SteerRequestType.MotionMagicExpo);

  /**
   * Creates AutoCommands using your robot's drive system.
   *
   * @param drivetrain The robot's swerve drive
   * @param pathRequest Helper that plans smooth paths between positions
   */
  public AutoCommands(CommandSwerveDrivetrain drivetrain, LinearPathRequest pathRequest) {
    this.drivetrain = drivetrain;
    this.pathRequest = pathRequest;
  }

  // ==================== Drive Commands ====================

  /**
   * Command to drive to a specific location and angle.
   *
   * @param pose Where to drive (x, y coordinates and rotation)
   * @return Command that drives the robot there
   */
  public Command driveTo(Pose2d pose) {
    return drivetrain.runOnce(() -> pathRequest.reset(drivetrain.getPose(), drivetrain.getFieldSpeeds()))
        .andThen(
            drivetrain.applyRequest(() -> pathRequest.withTargetPose(pose))
                .until(() -> pathRequest.isFinished()))
        .withName("DriveTo");
  }

  /**
   * Reset the robot's starting position.
   *
   * @param pose Where the robot should think it is (x, y, rotation)
   * @return Command that resets the position
   */
  public Command resetPose(Pose2d pose) {
    return drivetrain.runOnce(() -> drivetrain.resetPose(pose));
  }

  // ==================== Drive with Parallel Actions ====================

  /**
   * Drive to a location while doing something else at the same time.
   *
   * <p>Both actions start together. Use this when you want to save time by preparing
   * while driving (like raising the arm, spinning up the shooter, or intaking).
   *
   * @param targetPose Where to drive (x, y, and rotation)
   * @param parallelCommand The other thing to do while driving
   * @return Command that does both things at once
   */
  public Command driveToWithAction(Pose2d targetPose, Command parallelCommand) {
    return Commands.parallel(driveTo(targetPose), parallelCommand);
  }

  /**
   * Drive to a location, then do something when you get there.
   *
   * <p>The second action only starts after the robot arrives. Use this for actions that
   * need the robot to be still (like scoring, precise alignment, or shooting).
   *
   * @param targetPose Where to drive (x, y, and rotation)
   * @param afterCommand What to do after arriving
   * @return Command that drives then executes the action
   */
  public Command driveToThenExecute(Pose2d targetPose, Command afterCommand) {
    return Commands.sequence(driveTo(targetPose), afterCommand);
  }

  /**
   * Start a command when the robot gets close to a target location.
   *
   * <p>Use this to prepare ahead of time. For example, start spinning up the shooter
   * before you reach the shooting spot. The command watches the distance and starts
   * the action when you're close enough.
   *
   * <p><b>Example:</b>
   * <pre>{@code
   * // Start spinning flywheel when 1.5 meters away from shooting position
   * Commands.parallel(
   *   driveTo(shootPose),
   *   distanceCommand(1.5, shootPose, spinFlywheelCommand())
   * )
   * }</pre>
   *
   * @param triggerDistance How close to get before starting (in meters)
   * @param targetPose The target location to measure distance from
   * @param command What to do when you get close
   * @return Command that executes when you reach the distance
   */
  public Command distanceCommand(double triggerDistance, Pose2d targetPose, Command command) {
    return Commands.waitUntil(() -> {
      Pose2d currentPose = drivetrain.getPose();
      double distance = currentPose.getTranslation().getDistance(targetPose.getTranslation());
      return distance <= triggerDistance;
    }).andThen(command);
  }

  // ==================== Vision-Drive Blocks ====================

  /**
   * Drive backward (robot-relative, -X) until any of the specified tags is visible.
   * Stops driving when a tag is found OR when timeout expires.
   *
   * @param limelight Vision subsystem to check for tags
   * @param tagIds Array of AprilTag IDs to look for
   * @param speedMps Backward driving speed in m/s (positive value, will be negated)
   * @param timeoutSeconds Maximum time to drive before giving up
   */
  public Command driveBackwardUntilTag(LimelightSubsystem limelight, int[] tagIds,
      double speedMps, double timeoutSeconds) {
    return drivetrain.applyRequest(() -> robotCentric.withVelocityX(-speedMps))
        .until(() -> limelight.hasSpecificTag(tagIds))
        .withTimeout(timeoutSeconds)
        .withName("DriveBackwardUntilTag");
  }

  /**
   * Rotate in place to center on a specific hub tag.
   *
   * <p>Three-way logic each loop cycle:
   *   a) Center tag is primary target -> P-control on TX with camera offset correction
   *   b) A different tag is primary (side tag) -> spin toward it using TX sign
   *   c) No tag visible -> slow fallback spin
   *
   * <p>Exits when the center tag is centered within 3 degrees for 0.05 seconds (Debouncer).
   *
   * @param limelight Vision subsystem
   * @param centerTagId The center hub tag ID to align to (26 blue, 10 red)
   * @param timeoutSeconds Maximum alignment time
   */
  public Command alignToHubTag(LimelightSubsystem limelight, int centerTagId,
      double timeoutSeconds) {
    Debouncer centeredDebouncer = new Debouncer(0.05, DebounceType.kRising);
    return drivetrain.applyRequest(() -> {
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
            double tx = limelight.getTargetTX();
            rotRate = (tx > 0) ? -0.3 : 0.3;
          } else {
            // No tag visible — slow fallback spin
            rotRate = 0.3;
          }
          return robotCentric.withVelocityX(0.0).withRotationalRate(rotRate);
        })
        .until(() -> centeredDebouncer.calculate(
            limelight.getTargetId() == centerTagId
                && Math.abs(limelight.getTargetTX() - VisionConstants.CAMERA_TX_OFFSET_DEG)
                    <= 3.0))
        .withTimeout(timeoutSeconds)
        .withName("AlignToHubTag");
  }

  /**
   * Spin in place (no forward/backward movement) until any of the specified tags
   * is visible, or timeout expires.
   *
   * @param limelight Vision subsystem
   * @param tagIds Array of tag IDs to search for
   * @param spinRateRadS Rotation speed in rad/s (positive = CCW)
   * @param timeoutSeconds Maximum search time
   */
  public Command spinToFindTag(LimelightSubsystem limelight, int[] tagIds,
      double spinRateRadS, double timeoutSeconds) {
    return drivetrain.applyRequest(() ->
            robotCentric.withVelocityX(0.0).withRotationalRate(spinRateRadS))
        .until(() -> limelight.hasSpecificTag(tagIds))
        .withTimeout(timeoutSeconds)
        .withName("SpinToFindTag");
  }

  /**
   * Blind spin (no exit condition other than time). Used to rotate away from
   * a known area before starting a tag search, preventing false-positive exits.
   *
   * @param spinRateRadS Rotation speed in rad/s (positive = CCW)
   * @param durationSeconds How long to spin
   */
  public Command blindSpin(double spinRateRadS, double durationSeconds) {
    return drivetrain.applyRequest(() ->
            robotCentric.withVelocityX(0.0).withRotationalRate(spinRateRadS))
        .withTimeout(durationSeconds)
        .withName("BlindSpin");
  }

  // ==================== Utility Blocks ====================

  /** Wait for a specified duration. Convenience wrapper. */
  public Command waitSeconds(double seconds) {
    return Commands.waitSeconds(seconds).withName("Wait");
  }

  /** Print a message to console. Useful for tracing auto execution. */
  public Command log(String message) {
    return Commands.print(message).withName("Log");
  }
}
