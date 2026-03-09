package frc.robot.autonomous;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveModule.SteerRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.constants.VisionConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.vision.LimelightSubsystem;
import java.util.OptionalDouble;

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
 *   <li>Vision-drive building blocks (driveBackwardUntilTag, alignToHubTag, etc.)
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

  // Robot-centric request for vision-drive building blocks
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
   * <p>Bug 4 fix: flushes swerve state with an Idle request before resetting pose,
   * preventing stale rotation rates from a previous run from persisting.
   *
   * @param pose Where the robot should think it is (x, y, rotation)
   * @return Command that resets the position
   */
  public Command resetPose(Pose2d pose) {
    return drivetrain.runOnce(() -> {
      drivetrain.setControl(new SwerveRequest.Idle());
      drivetrain.resetPose(pose);
    });
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

  // ==================== Vision-Drive Building Blocks ====================

  /**
   * Drive backward (robot-relative, -X) until any of the specified tags is visible.
   * Ignores tags for the first {@code minDriveSeconds} to avoid false-positive exits
   * from nearby side tags. Stops when a tag is found OR when timeout expires.
   *
   * <p>Explicitly zeros RotationalRate and VelocityY on every loop cycle,
   * preventing stale values from a previous Phase 2 from causing an arc.
   *
   * @param limelight Vision subsystem to check for tags
   * @param tagIds Array of AprilTag IDs to look for
   * @param speedMps Backward driving speed in m/s (positive value, will be negated)
   * @param minDriveSeconds Minimum time to drive before checking for tags
   * @param timeoutSeconds Maximum total time to drive before giving up
   */
  public Command driveBackwardUntilTag(LimelightSubsystem limelight, int[] tagIds,
      double speedMps, double minDriveSeconds, double timeoutSeconds) {
    // Drive blindly for minDriveSeconds, then check for tags
    return drivetrain.applyRequest(() ->
            robotCentric.withVelocityX(-speedMps).withRotationalRate(0.0).withVelocityY(0.0))
        .withTimeout(minDriveSeconds)
        .andThen(
            drivetrain.applyRequest(() ->
                    robotCentric.withVelocityX(-speedMps).withRotationalRate(0.0).withVelocityY(0.0))
                .until(() -> limelight.hasSpecificTag(tagIds))
                .withTimeout(timeoutSeconds - minDriveSeconds))
        .withName("DriveBackwardUntilTag");
  }

  /**
   * Rotate in place to center on a specific hub tag.
   *
   * <p>Uses getTXForTags (reads from rawFiducialsCache) to avoid Limelight tracker lag.
   * Applies EMA filtering, brake-hold cycles, and a deadband to prevent oscillation.
   * Computes a dynamic camera TX offset based on tag distance for accurate alignment.
   *
   * <p>Two-way logic each loop cycle:
   * <ul>
   *   <li>Center tag visible: P-control on filtered TX with dynamic offset correction
   *   <li>Center tag lost (brake hold active): continue P-control on last filtered TX
   *   <li>No tag, brake expired: fallback spin toward last known tag direction
   * </ul>
   *
   * <p>Exits when the center tag is centered within 3 degrees for 0.1 seconds (Debouncer).
   *
   * @param limelight Vision subsystem
   * @param centerTagId The center hub tag ID to align to (26 blue, 10 red)
   * @param timeoutSeconds Maximum alignment time
   */
  public Command alignToHubTag(LimelightSubsystem limelight, int centerTagId,
      double timeoutSeconds) {
    Debouncer centeredDebouncer = new Debouncer(0.1, DebounceType.kRising);
    // Mutable state for lambda captures (arrays satisfy effectively-final requirement)
    double[] filteredTX = {0.0};
    boolean[] hadTagLastLoop = {false};
    int[] brakeHoldCycles = {0};
    double[] lastKnownTX = {0.0};
    int[] centerTagArr = new int[]{centerTagId};

    return drivetrain.applyRequest(() -> {
          double rotRate;
          OptionalDouble txOpt = limelight.getTXForTags(centerTagArr);

          // Dynamic camera TX offset: use distance-based atan when tag is visible
          double distMeters = limelight.getNearestTagDistance();
          double dynamicOffset = (distMeters > 0.1)
              ? -Math.toDegrees(Math.atan(0.241 / distMeters))
              : VisionConstants.CAMERA_TX_OFFSET_DEG;

          if (txOpt.isPresent()) {
            double rawTX = txOpt.getAsDouble();

            // EMA filter — seed on first detection, smooth thereafter
            if (!hadTagLastLoop[0]) {
              filteredTX[0] = rawTX;
            } else {
              filteredTX[0] =
                  VisionConstants.FACE_TAG_TX_FILTER_ALPHA * rawTX
                      + (1.0 - VisionConstants.FACE_TAG_TX_FILTER_ALPHA) * filteredTX[0];
            }
            hadTagLastLoop[0] = true;
            brakeHoldCycles[0] = VisionConstants.FACE_TAG_BRAKE_HOLD_CYCLES;
            lastKnownTX[0] = filteredTX[0];

            // P-control with deadband
            double error = filteredTX[0] - dynamicOffset;
            if (Math.abs(error) < VisionConstants.DRIVE_TO_TAG_TX_TOLERANCE_DEG) {
              rotRate = 0.0;
            } else {
              rotRate = MathUtil.clamp(
                  -error * VisionConstants.DRIVE_TO_TAG_TURN_KP,
                  -VisionConstants.DRIVE_TO_TAG_MAX_ROTATION_RAD_S,
                  VisionConstants.DRIVE_TO_TAG_MAX_ROTATION_RAD_S);
            }

            SmartDashboard.putNumber("AlignHub/RawTX", rawTX);
            SmartDashboard.putNumber("AlignHub/FilteredTX", filteredTX[0]);
            SmartDashboard.putNumber("AlignHub/DynamicOffset", dynamicOffset);
            SmartDashboard.putString("AlignHub/Status", "TRACKING");
          } else if (brakeHoldCycles[0] > 0) {
            // Tag briefly lost — keep P-control on last filteredTX
            brakeHoldCycles[0]--;
            hadTagLastLoop[0] = false;

            double error = filteredTX[0] - dynamicOffset;
            if (Math.abs(error) < VisionConstants.DRIVE_TO_TAG_TX_TOLERANCE_DEG) {
              rotRate = 0.0;
            } else {
              rotRate = MathUtil.clamp(
                  -error * VisionConstants.DRIVE_TO_TAG_TURN_KP,
                  -VisionConstants.DRIVE_TO_TAG_MAX_ROTATION_RAD_S,
                  VisionConstants.DRIVE_TO_TAG_MAX_ROTATION_RAD_S);
            }

            SmartDashboard.putString("AlignHub/Status", "BRAKING");
          } else {
            // No tag visible, brake hold expired — fallback spin
            hadTagLastLoop[0] = false;

            // Spin in the direction that would bring last known TX toward offset
            double lastError = lastKnownTX[0] - dynamicOffset;
            if (lastError != 0.0) {
              rotRate = (lastError > 0) ? -0.3 : 0.3;
            } else {
              rotRate = VisionConstants.DRIVE_TO_TAG_SEARCH_DIRECTION * 0.3;
            }

            SmartDashboard.putString("AlignHub/Status", "SEARCHING");
          }

          SmartDashboard.putNumber("AlignHub/RotRate", rotRate);
          return robotCentric.withVelocityX(0.0).withVelocityY(0.0).withRotationalRate(rotRate);
        })
        .until(() -> {
          OptionalDouble txOpt = limelight.getTXForTags(centerTagArr);
          if (txOpt.isEmpty()) {
            centeredDebouncer.calculate(false);
            return false;
          }
          double distMeters = limelight.getNearestTagDistance();
          double dynamicOffset = (distMeters > 0.1)
              ? -Math.toDegrees(Math.atan(0.241 / distMeters))
              : VisionConstants.CAMERA_TX_OFFSET_DEG;
          return centeredDebouncer.calculate(
              Math.abs(filteredTX[0] - dynamicOffset) <= 3.0);
        })
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
    return Commands.print(message);
  }
}
